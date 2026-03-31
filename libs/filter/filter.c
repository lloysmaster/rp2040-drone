#include "filter.h"

void filter_init(ema_filter_t *f, float alpha) {
    f->alpha = alpha;
    f->last_out = 0.0f;
}

float filter_apply(ema_filter_t *f, float input) {
    f->last_out = (f->last_out * (1.0f - f->alpha)) + (input * f->alpha);
    return f->last_out;
}