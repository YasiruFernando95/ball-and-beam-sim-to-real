#ifndef USB_TELEMETRY_H
#define USB_TELEMETRY_H

#include <stdint.h>

// Extern declarations for PID gains
extern volatile float g_pos_kp;
extern volatile float g_pos_ki;
extern volatile float g_pos_kd;

extern volatile float g_ang_kp;
extern volatile float g_ang_ki;
extern volatile float g_ang_kd;

extern volatile float g_pos_sp;

// ARM/DISARM flag - NEW!
extern volatile uint8_t g_outer_loop_armed;

// Functions
void USB_Telemetry_Send(float d, float angle, float omega, float angsp, float kp, double possp);
void USB_Command_Process(uint8_t *buf, uint32_t len);

#endif // USB_TELEMETRY_H
