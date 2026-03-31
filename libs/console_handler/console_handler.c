#include "console_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

void console_handle_input(pid_axis_t *roll, pid_axis_t *pitch, uint16_t *motor_test_val) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;

    static char buffer[32];
    static int idx = 0;

    if (c == '\n' || c == '\r') {
        buffer[idx] = '\0';
        if (idx > 0) {
            char type = buffer[0];
            float val = atof(&buffer[1]);

            switch (type) {
                case 'M': case 'm':
                    *motor_test_val = (uint16_t)val;
                    printf(">> Motor Test Set: %d\n", *motor_test_val);
                    break;

                case 'P': case 'p':
                    roll->kp = pitch->kp = val;
                    printf(">> Kp actualizado: %.4f\n", val);
                    break;

                case 'I': case 'i':
                    roll->ki = pitch->ki = val;
                    roll->integral = pitch->integral = 0; // Reset por seguridad
                    printf(">> Ki actualizado: %.4f (Integral reseteada)\n", val);
                    break;

                case 'D': case 'd':
                    roll->kd = pitch->kd = val;
                    printf(">> Kd actualizado: %.4f\n", val);
                    break;

                case 'S': case 's':
                    printf("\n--- PID STATUS ---\nP: %.4f | I: %.4f | D: %.4f\n", 
                            roll->kp, roll->ki, roll->kd);
                    break;
                    case 'B': case 'b':
                    if (val >= 1 && val <= 5) {
                    // Usamos motor_test_val temporalmente para señalizar un comando
                    // O mejor, pasamos una bandera de comando. 
                    // Por ahora, usemos un valor especial que el main reconozca.
                    *motor_test_val = 10000 + (uint16_t)val; 
                    printf(">> DShot Beacon Nivel %d activado\n", (int)val);
                    } else {
                         *motor_test_val = 0;
                    printf(">> Beacon desactivado\n");
                    }
                     break;
            }
        }
        idx = 0;
    } else if (idx < 31) {
        buffer[idx++] = (char)c;
    }
}