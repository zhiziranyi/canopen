#ifndef FAST_TRIG_H
#define FAST_TRIG_H

/*
 * Bounded, allocation-free sine/cosine pair for the 20 kHz control ISR.
 * The input is in radians; both outputs have a worst-case error below 0.002.
 */
void fast_sin_cos(float angle_rad, float* sin_value, float* cos_value);

#endif /* FAST_TRIG_H */
