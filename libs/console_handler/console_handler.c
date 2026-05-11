#include "console_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"

extern mutex_t my_mutex;

void console_handle_input(pid_axis_t *roll, pid_axis_t *pitch, pid_axis_t *yaw, uint16_t *motor_test_val) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;

    static char buffer[32];
    static int idx = 0;

    if (c == '\n' || c == '\r') {
        buffer[idx] = '\0';

        if (idx > 0) {
            char type    = buffer[0];
            char subtype = buffer[1];
            float val    = atof(&buffer[2]);

            mutex_enter_blocking(&my_mutex); // ← protegemos el acceso a las structs

            switch (type) {

                case 'M': case 'm':
                    *motor_test_val = (uint16_t)atof(&buffer[1]);
                    printf(">> Motor Test: %d\n", *motor_test_val);
                    break;

                case 'R': case 'r':
                    if      (subtype == 'P' || subtype == 'p') roll->kp = val;
                    else if (subtype == 'I' || subtype == 'i') { roll->ki = val; roll->integral = 0.0f; } // ← reset integral al cambiar ki
                    else if (subtype == 'D' || subtype == 'd') roll->kd = val;
                    printf(">> Roll PID actualizado\n");
                    break;

                case 'P': case 'p':
                    if      (subtype == 'P' || subtype == 'p') pitch->kp = val;
                    else if (subtype == 'I' || subtype == 'i') { pitch->ki = val; pitch->integral = 0.0f; }
                    else if (subtype == 'D' || subtype == 'd') pitch->kd = val;
                    printf(">> Pitch PID actualizado\n");
                    break;

                case 'Y': case 'y':
                    if      (subtype == 'P' || subtype == 'p') yaw->kp = val;
                    else if (subtype == 'I' || subtype == 'i') { yaw->ki = val; yaw->integral = 0.0f; }
                    else if (subtype == 'D' || subtype == 'd') yaw->kd = val;
                    printf(">> Yaw PID actualizado\n");
                    break;

                case 'S': case 's':
                    printf("#%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        roll->kp,  roll->ki,  roll->kd,
                        pitch->kp, pitch->ki, pitch->kd,
                        yaw->kp,   yaw->ki,   yaw->kd);
                    break;

                case 'B': case 'b':
                    if (val >= 1 && val <= 5) {
                        *motor_test_val = 10000 + (uint16_t)val;
                        printf(">> Beacon nivel %d\n", (int)val);
                    } else {
                        *motor_test_val = 0;
                        printf(">> Beacon OFF\n");
                    }
                    break;
            }

            mutex_exit(&my_mutex);
        }
        idx = 0;

    } else if (idx < 31) {
        buffer[idx++] = (char)c;
    }
}