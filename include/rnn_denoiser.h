#pragma once

#include <stdint.h>

#define RNND_NFFT (64)
typedef float rnn_num_t;

#ifdef __cplusplus
extern "C" {
#endif

void rnn_denoiser_denoise(rnn_num_t x[RNND_NFFT], rnn_num_t g[RNND_NFFT]);

#ifdef __cplusplus
}
#endif
