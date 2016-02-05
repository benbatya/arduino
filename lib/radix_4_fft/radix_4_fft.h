#ifndef __RADIX_4_FFT_H__
#define __RADIX_4_FFT_H__

void rev_bin( int16_t *fr, int fft_n);
void fft_radix4_I( int16_t *fr, int16_t *fi, int ldn);

#endif //  __RADIX_4_FFT_H__
