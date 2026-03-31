#ifndef FILTER_H
#define FILTER_H

typedef struct {
    float last_out;
    float alpha;
} ema_filter_t;

// Inicializa el filtro con un valor alpha (0.0 a 1.0)
void filter_init(ema_filter_t *f, float alpha);

// Aplica el filtro a un nuevo valor de entrada
float filter_apply(ema_filter_t *f, float input);

#endif