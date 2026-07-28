/*
 * virtual_wall.h
 *
 *  Created on: Nov 16, 2025
 *      Author: USER-PC
 */

#ifndef VIRTUAL_WALL_H
#define VIRTUAL_WALL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Apply a "virtual wall" on a speed command.
 *
 * @param speed                Commanded speed [-1, 1].
 * @param angle_deg            Current angle (degrees).
 * @param min_deg              Lower limit (degrees).
 * @param max_deg              Upper limit (degrees).
 * @param motor_to_angle_sign  +1 if positive speed increases angle,
 *                             -1 if positive speed decreases angle.
 *
 * @return Possibly modified speed. Returns 0 if command
 *         would push further past the limit.
 */
float VirtualWall_Apply(float speed,
                        float angle_deg,
                        float min_deg,
                        float max_deg,
                        int   motor_to_angle_sign);

#ifdef __cplusplus
}
#endif

#endif // VIRTUAL_WALL_H

