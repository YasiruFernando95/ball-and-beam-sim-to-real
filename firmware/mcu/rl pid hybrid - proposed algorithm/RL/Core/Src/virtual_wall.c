/*
 * virtual_wall.c
 *
 *  Created on: Nov 16, 2025
 *      Author: USER-PC
 */

#include "virtual_wall.h"

float VirtualWall_Apply(float speed,
                        float angle_deg,
                        float min_deg,
                        float max_deg,
                        int   motor_to_angle_sign)
{
    // Ensure sign is either +1 or -1 to avoid surprises
    if (motor_to_angle_sign >= 0) {
        motor_to_angle_sign = +1;
    } else {
        motor_to_angle_sign = -1;
    }

    float signed_speed = speed * (float)motor_to_angle_sign;

    // Upper limit: angle at/above max, and command would push angle higher
    if (angle_deg >= max_deg && signed_speed > 0.0f) {
        return 0.0f;
    }

    // Lower limit: angle at/below min, and command would push angle lower
    if (angle_deg <= min_deg && signed_speed < 0.0f) {
        return 0.0f;
    }

    // Otherwise allow the command (including moving back towards the safe region)
    return speed;
}
