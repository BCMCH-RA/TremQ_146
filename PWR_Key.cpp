#include "PWR_Key.h"

static uint8_t BAT_State = 0;
static uint8_t Device_State = 0;
static uint16_t Long_Press = 0;

void PWR_Loop(void)
{
  if (BAT_State) {
    if (!digitalRead(PWR_KEY_Input_PIN)) {
      if (BAT_State == 2) {
        Long_Press++;
        if (Long_Press >= Device_Sleep_Time) {
          if (Long_Press >= Device_Sleep_Time && Long_Press < Device_Restart_Time)
            Device_State = 1;
          else if (Long_Press >= Device_Restart_Time && Long_Press < Device_Shutdown_Time)
            Device_State = 2;
          else if (Long_Press >= Device_Shutdown_Time)
            Shutdown();
        }
      }
    }
    else {
      if (BAT_State == 1)
        BAT_State = 2;
      Long_Press = 0;
    }
  }
}

void Fall_Asleep(void)
{
}

void Restart(void)
{
}

void Shutdown(void)
{
  // Best-effort data safety before we cut our own power: close the latch only
  // after telling the world, and give the caller (loop) a chance to stop the
  // recorder first. The FAT layer needs an explicit close/flush or the last
  // buffered CSV rows die with the card power.
  printf("PWR: powering off\r\n");
  Set_Backlight(0);
  digitalWrite(PWR_Control_PIN, LOW);
}

void PWR_Init(void)
{
  pinMode(PWR_KEY_Input_PIN, INPUT);
  pinMode(PWR_Control_PIN, OUTPUT);
  digitalWrite(PWR_Control_PIN, LOW);
  // Wait in milliseconds, not raw ticks: "100" was ~100 s at the default
  // 100 Hz tick rate (and is ambiguous across builds), which delayed boot and
  // could miss a power-button press held during startup.
  vTaskDelay(pdMS_TO_TICKS(100));
  if (!digitalRead(PWR_KEY_Input_PIN)) {
    BAT_State = 1;
    digitalWrite(PWR_Control_PIN, HIGH);
    printf("PWR: button pressed at boot, power latched\r\n");
  } else {
    printf("PWR: button not pressed at boot (USB power?)\r\n");
  }
}