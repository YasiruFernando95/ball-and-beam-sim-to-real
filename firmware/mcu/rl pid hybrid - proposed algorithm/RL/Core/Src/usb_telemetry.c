#include "usb_telemetry.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"   // NVIC_SystemReset
#include <stdio.h>
#include <string.h>

volatile float g_ang_kp = 0.0f;
volatile float g_ang_ki = 0.0f;
volatile float g_ang_kd = 0.0f;

volatile float g_pos_sp = 0.0f;

volatile float g_thsp_cmd = 0.0f;          // latest motor command from PC, normalized [-1..+1]
volatile uint32_t g_thsp_cmd_ms = 0;       // HAL_GetTick() when last command arrived




void USB_Telemetry_Send(float d, float angle, float omega, float angsp, float kp, double possp)
{
    char msg[64];

    int len = snprintf(msg, sizeof(msg),
    	      "D:%.3f,A:%.2f,W:%.3f,AS:%.2f,OKP:%.4f,PS:%.4f\n",
    	      d, angle, omega, (float)angsp, kp, possp);

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

    float v;

	// ---- RL command ----
	if (sscanf(cmd, "THSP %f", &v) == 1) {
		if (v >  25.0f) v =  25.0f;
		if (v < -25.0f) v = -25.0f;
		g_thsp_cmd = v;
		g_thsp_cmd_ms = HAL_GetTick();
		return;
	}

	// Optional emergency stop
	if (strcmp(cmd, "STOP") == 0) {
		g_thsp_cmd = 0.0f;
		g_thsp_cmd_ms = HAL_GetTick();
		return;
	}




}


