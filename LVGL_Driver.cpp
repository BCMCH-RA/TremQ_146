#include "LVGL_Driver.h"
#include <string.h>

static lv_display_t *disp_handle = NULL;
static lv_indev_t *indev_handle = NULL;
static uint8_t draw_buf1[ LVGL_BUF_LEN * 2 ];
static uint8_t draw_buf2[ LVGL_BUF_LEN * 2 ];
static volatile uint32_t last_touch_ms = 0;

lv_obj_t *lbl_severity = NULL;
lv_obj_t *lbl_ist_time = NULL;
lv_obj_t *lbl_ist_date = NULL;
lv_obj_t *lbl_tremor_rms = NULL;
lv_obj_t *lbl_ble_status = NULL;
lv_obj_t *lbl_rec_status = NULL;
lv_obj_t *lbl_sample_count = NULL;
lv_obj_t *lbl_battery = NULL;
lv_obj_t *lbl_lowbat = NULL;
lv_obj_t *img_logo = NULL;

// Logo slot geometry and the generated bitmap itself. logo_believers.h defines
// LOGO_SIZE, so swapping the source artwork and regenerating keeps the header,
// the descriptor and the slot in step automatically.
#include "logo_believers.h"
static_assert(sizeof(logoBelievers) == LOGO_SIZE * LOGO_SIZE * sizeof(uint16_t),
              "logo_believers.h is not LOGO_SIZE*LOGO_SIZE RGB565 pixels");

void Lvgl_print(const char * buf)
{
}

static void lv_tick_cb(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void Lvgl_Display_LCD(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *px_map)
{
    LCD_addWindow(area->x1, area->y1, area->x2, area->y2, (uint16_t *)px_map);
    lv_display_flush_ready(disp_drv);
}

void Lvgl_Touchpad_Read(lv_indev_t * indev_drv, lv_indev_data_t * data)
{
    uint16_t x[1] = {0};
    uint16_t y[1] = {0};
    uint8_t cnt = 0;
    bool pressed = Touch_Get_xy(&x[0], &y[0], NULL, &cnt, 1);
    static bool was_touched = false;
    if (pressed && (cnt > 0)) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        if (!was_touched) {
            last_touch_ms = millis();
            was_touched = true;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        was_touched = false;
    }
}

uint32_t Lvgl_Get_LastTouchMs(void)
{
    return last_touch_ms;
}

void Lvgl_Init(void)
{
    lv_init();
    disp_handle = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
    lv_display_set_buffers(disp_handle, draw_buf1, draw_buf2, sizeof(draw_buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(disp_handle, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp_handle, Lvgl_Display_LCD);

    indev_handle = lv_indev_create();
    lv_indev_set_type(indev_handle, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_handle, Lvgl_Touchpad_Read);

    esp_timer_handle_t lvgl_tick_timer;
    esp_timer_create_args_t tick_args = {
        .callback = &lv_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_create(&tick_args, &lvgl_tick_timer);
    esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Layout notes (screen is 412x412):
    //  - The severity and clock fonts were reduced so the bottom block could be
    //    laid out two rows instead of four. Every label below the clock is
    //    bigger than it used to be, and the paired rows use the horizontal
    //    space the 412px width leaves, so nothing is lost by pairing them.
    //  - Severity text is black on a filled band, so all four band colours have
    //    to stay light enough to read black text against.
    // Title sits top-CENTERED so it is visually balanced with the logo on the right.
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "BCMCH Tremor");
    lv_obj_set_style_text_color(title, lv_color_hex(0x9E9E9E), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);   // <-- CENTERED

    // Reserved logo slot. img_logo is an empty container until a bitmap is
    // installed with Lvgl_SetLogo(), so the layout is correct either way and
    // no asset is hard-coded here.
    img_logo = lv_obj_create(scr);
    lv_obj_set_size(img_logo, LOGO_SIZE, LOGO_SIZE);
    lv_obj_set_style_bg_opa(img_logo, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(img_logo, 0, 0);
    lv_obj_set_style_pad_all(img_logo, 0, 0);
    lv_obj_remove_flag(img_logo, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(img_logo, LV_ALIGN_TOP_RIGHT, -14, 8);

    lbl_severity = lv_label_create(scr);
    lv_label_set_text(lbl_severity, "CALM");
    lv_obj_set_width(lbl_severity, LCD_WIDTH - 24);
    lv_obj_set_style_bg_color(lbl_severity, lv_color_hex(0x00E676), 0);
    lv_obj_set_style_bg_opa(lbl_severity, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(lbl_severity, 12, 0);
    lv_obj_set_style_pad_all(lbl_severity, 12, 0);
    lv_obj_set_style_text_color(lbl_severity, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(lbl_severity, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(lbl_severity, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_severity, LV_ALIGN_TOP_MID, 0, 52);

    lbl_ist_time = lv_label_create(scr);
    lv_label_set_text(lbl_ist_time, "IST: --:--:--");
    lv_obj_set_style_text_color(lbl_ist_time, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_ist_time, &lv_font_montserrat_32, 0);
    lv_obj_align(lbl_ist_time, LV_ALIGN_TOP_MID, 0, 132);

    lbl_ist_date = lv_label_create(scr);
    lv_label_set_text(lbl_ist_date, "---- -- -- --");
    lv_obj_set_style_text_color(lbl_ist_date, lv_color_hex(0xB0BEC5), 0);
    lv_obj_set_style_text_font(lbl_ist_date, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_ist_date, LV_ALIGN_TOP_MID, 0, 184);

    lbl_tremor_rms = lv_label_create(scr);
    lv_label_set_text(lbl_tremor_rms, "RMS: 0.0 dps");
    lv_obj_set_style_text_color(lbl_tremor_rms, lv_color_hex(0x00E676), 0);
    lv_obj_set_style_text_font(lbl_tremor_rms, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl_tremor_rms, LV_ALIGN_TOP_MID, 0, 224);

    // Bottom block: two paired rows at a larger font than the 16px they used
    // to be. Left/right pairing keeps each field's own colour, which a single
    // merged string could not.
    lbl_ble_status = lv_label_create(scr);
    lv_label_set_text(lbl_ble_status, "BLE: Disconnected");
    lv_obj_set_style_text_color(lbl_ble_status, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_text_font(lbl_ble_status, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_ble_status, LV_ALIGN_TOP_LEFT, 14, 278);

    lbl_rec_status = lv_label_create(scr);
    lv_label_set_text(lbl_rec_status, "Recording: OFF");
    lv_obj_set_style_text_color(lbl_rec_status, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(lbl_rec_status, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_rec_status, LV_ALIGN_TOP_RIGHT, -14, 278);

    lbl_sample_count = lv_label_create(scr);
    lv_label_set_text(lbl_sample_count, "Samples: 0");
    lv_obj_set_style_text_color(lbl_sample_count, lv_color_hex(0x44AAFF), 0);
    lv_obj_set_style_text_font(lbl_sample_count, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_sample_count, LV_ALIGN_TOP_LEFT, 14, 318);

    lbl_battery = lv_label_create(scr);
    lv_label_set_text(lbl_battery, "Bat: --.--V --%");
    lv_obj_set_style_text_color(lbl_battery, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(lbl_battery, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_battery, LV_ALIGN_TOP_RIGHT, -14, 318);

    lbl_lowbat = lv_label_create(scr);
    lv_label_set_text(lbl_lowbat, "LOW BATTERY");
    lv_obj_set_width(lbl_lowbat, LCD_WIDTH - 24);
    lv_obj_set_style_bg_color(lbl_lowbat, lv_color_hex(0xF44336), 0);
    lv_obj_set_style_bg_opa(lbl_lowbat, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(lbl_lowbat, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl_lowbat, &lv_font_montserrat_24, 0);
    lv_obj_set_style_radius(lbl_lowbat, 14, 0);
    lv_obj_set_style_pad_all(lbl_lowbat, 12, 0);
    lv_obj_set_style_text_align(lbl_lowbat, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_lowbat, LV_ALIGN_TOP_MID, 0, 262);
    lv_obj_add_flag(lbl_lowbat, LV_OBJ_FLAG_HIDDEN);
}

// Install a logo bitmap into the reserved top-right slot. The buffer must be
// LOGO_SIZE*LOGO_SIZE RGB565 pixels and must outlive the call (LVGL keeps the
// pointer, it does not copy). Passing NULL clears the slot.
void Lvgl_SetLogo(const uint16_t *rgb565)
{
    if (!img_logo) return;
    if (!rgb565) {
        lv_image_set_src(img_logo, NULL);
        return;
    }
    static lv_image_dsc_t dsc;
    dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    dsc.header.flags = 0;
    dsc.header.w = LOGO_SIZE;
    dsc.header.h = LOGO_SIZE;
    // stride is in bytes, and LVGL relies on it to walk the rows.
    dsc.header.stride = LOGO_SIZE * (uint32_t)sizeof(uint16_t);
    dsc.header.reserved_2 = 0;
    dsc.data_size = LOGO_SIZE * LOGO_SIZE * (uint32_t)sizeof(uint16_t);
    dsc.data = (const uint8_t *)rgb565;
    dsc.reserved = NULL;
    lv_image_set_src(img_logo, &dsc);
}

// lv_label_set_text() re-renders the label and invalidates its area every call,
// even when the string is byte-identical to the one already shown. The display
// task calls this repeatedly, so caching the last text per label turns the
// steady state into a handful of strcmp()s and no redraws at all. That is where
// most of the saving from throttling the update rate actually comes from.
static void set_text_cached(lv_obj_t *lbl, const char *txt, char *cache, size_t cap)
{
    if (!lbl || !txt) return;
    if (cache[0] == txt[0] && strncmp(cache, txt, cap) == 0) return;
    strlcpy(cache, txt, cap);
    lv_label_set_text(lbl, txt);
}

void Lvgl_Update_Display(const char *ist_time, const char *ist_date,
                         const char *ble_status, const char *rec_status,
                         uint32_t samples, float battery_v, uint8_t battery_pct,
                         const char *sev_text, uint32_t sev_rgb, float tremor_rms_dps)
{
    static char c_sev[12], c_ist[12], c_date[16], c_rms[24];
    static char c_ble[24], c_rec[20], c_smp[24], c_bat[24];

    // Own colour caches rather than reading back LVGL's style structs: poking at
    // spec_attr is version-dependent and would break on an LVGL upgrade, while
    // remembering what we last set is just as effective.
    static uint32_t col_sev = 0xFFFFFFFFu, col_ble = 0xFFFFFFFFu;
    static uint32_t col_rec = 0xFFFFFFFFu, col_bat = 0xFFFFFFFFu;

    // ─── Throttle slow-changing fields to 2 seconds ──────────────────────
    // RMS, Samples, and Battery change slowly enough that updating them at
    // 10 Hz wastes SPI bandwidth and LCD controller time. Throttling to 2 s
    // cuts their redraw rate by 95%, which directly saves battery because
    // each redraw requires a full SPI pixel transfer.
    static uint32_t last_slow_update = 0;
    const uint32_t now = millis();
    const bool slow_update = (now - last_slow_update >= 2000);
    if (slow_update) last_slow_update = now;

    // The severity band is redrawn only when the label or its colour actually
    // changes, which after the dwell is at most a couple of times a minute.
    if (lbl_severity && sev_text) {
        if (c_sev[0] != sev_text[0] || strncmp(c_sev, sev_text, sizeof(c_sev)) != 0) {
            strlcpy(c_sev, sev_text, sizeof(c_sev));
            lv_label_set_text(lbl_severity, sev_text);
        }
    }
    if (lbl_severity && sev_rgb != col_sev) {
        col_sev = sev_rgb;
        lv_obj_set_style_bg_color(lbl_severity, lv_color_hex(sev_rgb), 0);
    }

    // IST time and date update every call so the seconds tick smoothly.
    set_text_cached(lbl_ist_time, ist_time, c_ist, sizeof(c_ist));
    set_text_cached(lbl_ist_date, ist_date, c_date, sizeof(c_date));

    // ─── RMS: only update every 2 seconds ────────────────────────────────
    if (slow_update && lbl_tremor_rms) {
        char buf[24];
        snprintf(buf, sizeof(buf), "RMS: %.1f dps", tremor_rms_dps);
        set_text_cached(lbl_tremor_rms, buf, c_rms, sizeof(c_rms));
    }

    // BLE and Recording status update every call so state changes are instant.
    if (lbl_ble_status && ble_status) {
        set_text_cached(lbl_ble_status, ble_status, c_ble, sizeof(c_ble));
        const uint32_t col = strstr(ble_status, "Connected") ? 0x00FF00u : 0xFF4444u;
        if (col != col_ble) {
            col_ble = col;
            lv_obj_set_style_text_color(lbl_ble_status, lv_color_hex(col), 0);
        }
    }
    if (lbl_rec_status && rec_status) {
        set_text_cached(lbl_rec_status, rec_status, c_rec, sizeof(c_rec));
        const uint32_t col = strstr(rec_status, "ON") ? 0xFF4444u : 0x888888u;
        if (col != col_rec) {
            col_rec = col;
            lv_obj_set_style_text_color(lbl_rec_status, lv_color_hex(col), 0);
        }
    }

    // ─── Samples: only update every 2 seconds ────────────────────────────
    if (slow_update && lbl_sample_count) {
        char buf[24];
        snprintf(buf, sizeof(buf), "Samples: %lu", (unsigned long)samples);
        set_text_cached(lbl_sample_count, buf, c_smp, sizeof(c_smp));
    }

    // ─── Battery: only update every 2 seconds ────────────────────────────
    // ADC readings drift slowly; 2 s is plenty fast enough to see the
    // voltage drop as the battery discharges, and it saves a redraw every
    // 100 ms that was just re-rendering the same two-decimal number.
    if (slow_update && lbl_battery) {
        char buf[24];
        snprintf(buf, sizeof(buf), "Bat: %.2fV %u%%", battery_v, battery_pct);
        set_text_cached(lbl_battery, buf, c_bat, sizeof(c_bat));
        uint32_t col = 0x00E676u;
        if (battery_pct <= 20)      col = 0xFF4444u;
        else if (battery_pct <= 40) col = 0xFFA726u;
        if (col != col_bat) {
            col_bat = col;
            lv_obj_set_style_text_color(lbl_battery, lv_color_hex(col), 0);
        }
    }
}

void Lvgl_Set_LowBat(bool low, uint8_t pct)
{
    if (!lbl_lowbat) return;
    if (low) {
        char buf[32];
        snprintf(buf, sizeof(buf), "LOW BATTERY %u%%", pct);
        lv_label_set_text(lbl_lowbat, buf);
        lv_obj_clear_flag(lbl_lowbat, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(lbl_lowbat, LV_OBJ_FLAG_HIDDEN);
    }
}

void Lvgl_Loop(void)
{
    lv_timer_handler();
}
