#pragma once
#include <stdint.h>

extern volatile float g_pos_kp, g_pos_ki, g_pos_kd;
extern volatile float g_ang_kp, g_ang_ki, g_ang_kd;

extern volatile float g_u_cmd;
extern volatile uint32_t g_u_cmd_ms;

// NEW: setpoints from GUI
extern volatile float g_pos_sp;      // normalized -1..+1
//extern volatile float g_ang_sp_deg;  // degrees

//extern volatile uint8_t g_tuning_enabled;


void USB_Telemetry_Send(float d, float angle, float omega, float angsp, float kp, double possp);
void USB_Command_Process(uint8_t *buf, uint32_t len);
