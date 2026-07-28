#include "usb_telemetry.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"   // NVIC_SystemReset
#include <stdio.h>
#include <string.h>

volatile float g_pos_kp = 0.0f;
volatile float g_pos_ki = 0.0f;
volatile float g_pos_kd = 0.0f;

volatile float g_ang_kp = 0.0f;
volatile float g_ang_ki = 0.0f;
volatile float g_ang_kd = 0.0f;

volatile float g_pos_sp = 0.0f;

// ARM/DISARM flag - NEW!
volatile uint8_t g_outer_loop_armed = 0;  // 0 = disarmed (angle SP = 0), 1 = armed (PID active)


void USB_Telemetry_Send(float d, float angle, float omega, float angsp, float kp, double possp)
{
    char msg[64];

    // Include ARM status in telemetry - NEW!
    int len = snprintf(msg, sizeof(msg),
    	      "D:%.3f,A:%.2f,W:%.3f,AS:%.2f,OKP:%.4f,PS:%.4f,ARM:%d\n",
    	      d, angle, omega, (float)angsp, kp, possp, g_outer_loop_armed);

    CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
}

static void strip_crlf(char *s)
{
    for (size_t i = 0; s[i]; i++) {
        if (s[i] == '\r' || s[i] == '\n') { s[i] = '\0'; break; }
    }
}

void USB_Command_Process(uint8_t *buf, uint32_t len)
{
    if (len == 0) return;

    char cmd[96];
    if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
    memcpy(cmd, buf, len);
    cmd[len] = '\0';
    strip_crlf(cmd);

    // --- reset ---
    if (strcmp(cmd, "RST") == 0) {
        NVIC_SystemReset();
        return;
    }

    // --- ARM/DISARM commands - NEW! ---
    if (strcmp(cmd, "ARM") == 0) {
        g_outer_loop_armed = 1;
        return;
    }

    if (strcmp(cmd, "DISARM") == 0 || strcmp(cmd, "STOP") == 0) {
        g_outer_loop_armed = 0;
        return;
    }

    // --- setpoints ---
    float v;
    if (sscanf(cmd, "POSSP %f", &v) == 1) {
        if (v >  1.0f) v =  1.0f;
        if (v < -1.0f) v = -1.0f;
        g_pos_sp = v;
        return;
    }

    // --- PID updates (obey tuning lock) ---
    float kp, ki, kd;

    if (sscanf(cmd, "PIDP KP=%f KI=%f KD=%f", &kp, &ki, &kd) == 3) {
        g_pos_kp = kp; g_pos_ki = ki; g_pos_kd = kd;
        return;
    }

    if (sscanf(cmd, "PIDA KP=%f KI=%f KD=%f", &kp, &ki, &kd) == 3) {
        g_ang_kp = kp; g_ang_ki = ki; g_ang_kd = kd;
        return;
    }
}
