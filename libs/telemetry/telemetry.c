#include "telemetry.h"
#include <stdio.h>

void telemetry_init() {
    // Por ahora solo usamos el stdio inicializado en el main
}

void telemetry_send(const telemetry_data_t *data) {
    // Formato optimizado para parseo (CSV o prefijado)
    // Usamos un prefijo '$' para que tu JS sepa dónde empieza una trama de datos
    printf("$%.2f,%.2f,%.2f,%d,%d,%d,%d,%d\n",
           data->roll,
           data->pitch,
           data->yaw,
           data->motors[0],
           data->motors[1],
           data->motors[2],
           data->motors[3],
           data->throttle);
}