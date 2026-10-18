#include "Tremor_Metric.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad_t;

typedef struct {
    Biquad_t stage[TREMOR_STAGES];
} Cascade_t;

static Cascade_t s_cX, s_cY, s_cZ;

// Owned by Core 1 (IMU task) only
static float s_severityMS = 0.0f;
static float s_totalMS    = 0.0f;

// Published values (read by Core 0). Use sequence number for consistent reads.
static volatile float    s_severity  = 0.0f;
static volatile uint8_t  s_level     = TREMOR_CALM;
static volatile uint32_t s_sampleSeq = 0;
static volatile float    s_ratio     = 1.0f;

static TremorLevel_t s_cand  = TREMOR_CALM;
static uint32_t      s_dwell = 0;

static uint32_t s_lastGetSeq   = 0;
static uint32_t s_lastRatioSeq = 0;

static void Biquad_Design(Biquad_t *b, float al, float cs) {
    const float a0 = 1.0f + al;
    b->b0 =  al / a0;
    b->b1 =  0.0f;
    b->b2 = -al / a0;
    b->a1 = (-2.0f * cs) / a0;
    b->a2 = (1.0f - al) / a0;
    b->z1 = 0.0f;
    b->z2 = 0.0f;
}

static float Biquad_Mag(const Biquad_t *b, float c1, float s1, float c2, float s2) {
    const float nr = b->b0 + b->b2 * c2;
    const float ni = -b->b2 * s2;
    const float dr = 1.0f + b->a1 * c1 + b->a2 * c2;
    const float di = -b->a1 * s1 - b->a2 * s2;
    float num = nr * nr + ni * ni;
    float den = dr * dr + di * di;
    if (den < 1e-12f) return 0.0f;
    return sqrtf(num / den);
}

static inline float Biquad_Step(Biquad_t *b, float x) {
    const float y = b->b0 * x + b->z1;
    b->z1 = b->b1 * x + b->z2 - b->a1 * y;
    b->z2 = b->b2 * x - b->a2 * y;
    return y;
}

static inline float Cascade_Step(Cascade_t *c, float x) {
    for (int i = 0; i < TREMOR_STAGES; i++) {
        x = Biquad_Step(&c->stage[i], x);
    }
    return x;
}

static void Tremor_FiltersInit(void) {
    const float f0 = sqrtf(TREMOR_BAND_LO_HZ * TREMOR_BAND_HI_HZ);
    const float w0 = 2.0f * (float)M_PI * f0 / TREMOR_SAMPLE_RATE_HZ;
    const float sw0 = sinf(w0), cw0 = cosf(w0);
    const float sw02 = sinf(2.0f * w0), cw02 = cosf(2.0f * w0);

    const float wl = 2.0f * (float)M_PI * TREMOR_BAND_LO_HZ / TREMOR_SAMPLE_RATE_HZ;
    const float swl = sinf(wl), cwl = cosf(wl);
    const float swl2 = sinf(2.0f * wl), cwl2 = cosf(2.0f * wl);

    const float wh = 2.0f * (float)M_PI * TREMOR_BAND_HI_HZ / TREMOR_SAMPLE_RATE_HZ;
    const float swh = sinf(wh), cwh = cosf(wh);
    const float swh2 = sinf(2.0f * wh), cwh2 = cosf(2.0f * wh);

    Biquad_t s;
    float qlo = 0.10f, qhi = 50.0f;
    for (int i = 0; i < 48; i++) {
        const float q = 0.5f * (qlo + qhi);
        Biquad_Design(&s, sw0 / (2.0f * q), cw0);
        const float peak = Biquad_Mag(&s, cw0, sw0, cw02, sw02);
        if (peak < 1e-6f) { qlo = q; continue; }
        const float rlo = Biquad_Mag(&s, cwl, swl, cwl2, swl2) / peak;
        const float rhi = Biquad_Mag(&s, cwh, swh, cwh2, swh2) / peak;
        const float clo = powf(rlo, (float)TREMOR_STAGES);
        const float chi = powf(rhi, (float)TREMOR_STAGES);
        if (0.5f * (clo + chi) > 0.70710678f) qlo = q;
        else qhi = q;
    }

    const float q = 0.5f * (qlo + qhi);
    Biquad_Design(&s, sw0 / (2.0f * q), cw0);
    const float peak = Biquad_Mag(&s, cw0, sw0, cw02, sw02);
    if (peak > 1e-6f) {
        s.b0 /= peak;
        s.b2 /= peak;
    }

    for (int i = 0; i < TREMOR_STAGES; i++) {
        s_cX.stage[i] = s;
        s_cY.stage[i] = s;
        s_cZ.stage[i] = s;
    }
}

void Tremor_Metric_Init(void) {
    memset(&s_cX, 0, sizeof(s_cX));
    memset(&s_cY, 0, sizeof(s_cY));
    memset(&s_cZ, 0, sizeof(s_cZ));
    Tremor_FiltersInit();
    s_severityMS   = 0.0f;
    s_totalMS      = 0.0f;
    s_severity     = 0.0f;
    s_level        = TREMOR_CALM;
    s_sampleSeq    = 0;
    s_lastGetSeq   = 0;
    s_lastRatioSeq = 0;
    s_ratio        = 1.0f;
    s_cand         = TREMOR_CALM;
    s_dwell        = 0;
}

static const float s_edge[3] = {
    TREMOR_CALM_MAX_DPS, TREMOR_MILD_MAX_DPS, TREMOR_MODERATE_MAX_DPS
};

static TremorLevel_t Band_Decide(TremorLevel_t cur, float sev) {
    for (int step = -1; step <= 1; step += 2) {
        const int idx = (int)cur + step;
        if (idx < 0 || idx > 3) continue;
        if (step > 0) {
            if (sev >= s_edge[idx - 1] * (1.0f + TREMOR_BAND_HYSTERESIS))
                return (TremorLevel_t)idx;
        } else {
            if (sev < s_edge[idx] * (1.0f - TREMOR_BAND_HYSTERESIS))
                return (TremorLevel_t)idx;
        }
    }
    return cur;
}

static uint32_t Dwell_Needs(TremorLevel_t from, TremorLevel_t to) {
    const float secs = (to > from) ? TREMOR_DWELL_ESCALATE_S : TREMOR_DWELL_DEESCALATE_S;
    uint32_t n = (uint32_t)(secs * TREMOR_SAMPLE_RATE_HZ + 0.5f);
    return n ? n : 1u;
}

void Tremor_Metric_Feed(float gx_dps, float gy_dps, float gz_dps) {
    const float fx = Cascade_Step(&s_cX, gx_dps);
    const float fy = Cascade_Step(&s_cY, gy_dps);
    const float fz = Cascade_Step(&s_cZ, gz_dps);

    const float magSq = fx * fx + fy * fy + fz * fz;
    const float sevNewMs = TREMOR_EMA_ALPHA * magSq + (1.0f - TREMOR_EMA_ALPHA) * s_severityMS;

    const float totalSq = gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps;
    const float totNewMs = TREMOR_EMA_ALPHA * totalSq + (1.0f - TREMOR_EMA_ALPHA) * s_totalMS;

    const float sevNew    = sqrtf(sevNewMs);
    const float narrowNew = (totNewMs > 1e-3f) ? sqrtf(sevNewMs / totNewMs) : 1.0f;

    const TremorLevel_t cur  = s_cand > (TremorLevel_t)s_level ? s_cand : (TremorLevel_t)s_level;
    TremorLevel_t want = Band_Decide((TremorLevel_t)s_level, sevNew);
    if (want > (TremorLevel_t)s_level && narrowNew < TREMOR_NARROWBAND_MIN) {
        want = (TremorLevel_t)s_level;
    }

    TremorLevel_t newLevel = (TremorLevel_t)s_level;
    TremorLevel_t newCand  = s_cand;
    uint32_t      newDwell = s_dwell;

    if (want == (TremorLevel_t)s_level) {
        newCand  = want;
        newDwell = 0;
        newLevel = want;
    } else if (want != s_cand) {
        newCand  = want;
        newDwell = 0;
        newLevel = (TremorLevel_t)s_level;
    } else {
        uint32_t nd = s_dwell + 1;
        if (nd >= Dwell_Needs((TremorLevel_t)s_level, want)) {
            newLevel = want;
            newCand  = want;
            newDwell = 0;
        } else {
            newLevel = (TremorLevel_t)s_level;
            newCand  = s_cand;
            newDwell = nd;
        }
    }

    s_severityMS = sevNewMs;
    s_totalMS    = totNewMs;

    s_severity  = sevNew;
    s_ratio     = narrowNew;
    s_level     = (uint8_t)newLevel;
    s_cand      = newCand;
    s_dwell     = newDwell;
    s_sampleSeq++;
}

bool Tremor_Metric_Get(TremorLevel_t *out_level, float *out_severity_dps) {
    uint32_t seq1 = s_sampleSeq;
    if (seq1 == s_lastGetSeq) return false;
    float sev = s_severity;
    uint8_t lvl = s_level;
    uint32_t seq2 = s_sampleSeq;
    if (seq2 != seq1) return false;
    s_lastGetSeq = seq1;
    if (out_level)        *out_level = (TremorLevel_t)lvl;
    if (out_severity_dps) *out_severity_dps = sev;
    return true;
}

bool Tremor_Metric_GetNarrowband(float *out_ratio) {
    uint32_t seq1 = s_sampleSeq;
    if (seq1 == s_lastRatioSeq) return false;
    float nb = s_ratio;
    uint32_t seq2 = s_sampleSeq;
    if (seq2 != seq1) return false;
    s_lastRatioSeq = seq1;
    if (out_ratio) *out_ratio = nb;
    return true;
}

const char *Tremor_Level_Text(TremorLevel_t level) {
    switch (level) {
        case TREMOR_CALM:     return "CALM";
        case TREMOR_MILD:     return "MILD";
        case TREMOR_MODERATE: return "MODERATE";
        case TREMOR_SEVERE:   return "SEVERE";
        default:              return "CALM";
    }
}

uint32_t Tremor_Level_Color(TremorLevel_t level) {
    switch (level) {
        case TREMOR_CALM:     return 0x00E676;
        case TREMOR_MILD:     return 0xFFD600;
        case TREMOR_MODERATE: return 0xFFA726;
        case TREMOR_SEVERE:   return 0xF44336;
        default:              return 0x00E676;
    }
}
