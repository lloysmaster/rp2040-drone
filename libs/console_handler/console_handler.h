#ifndef CONSOLE_HANDLER_H
#define CONSOLE_HANDLER_H

#include "pid.h"
#include <stdint.h>

// Pasamos punteros a los PID y al valor de test de motores para que la función pueda modificarlos
void console_handle_input(pid_axis_t *roll, pid_axis_t *pitch, uint16_t *motor_test_val);

#endif