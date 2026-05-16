#include "rnn_denoiser.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <iterator>
#include <numeric>

#include "fastgrnn_fc_params.h"
#include "fastgrnn_rnn_params.h"
#include "rnn_denoiser_cfg.h"

#define __time_critical_func(_a) _a

static const uint16_t last_bin = (FILTERBANK_OFF[MEL_BINS - 1] + FB_LEN);

static inline rnn_num_t sigmoidf(rnn_num_t x) { return (1 / (1 + expf(-x))); }
static inline rnn_num_t hardsigmoid(rnn_num_t x) {
  if (x <= -3.0f) {
    return 0.0f;
  } else if (x >= 3.0f) {
    return 1.0f;
  } else {
    return (x / 6.0f) + 0.5f;
  }
}

static void __time_critical_func(_rnn_process)(
    const rnn_num_t input[GRNN0_HIDD_DIM0],
    const rnn_num_t hidden[GRNN0_HIDD_DIM1],
    rnn_num_t output[GRNN0_HIDD_DIM1]) {
  rnn_num_t z;
  rnn_num_t c;

  for (uint16_t j = 0; j < GRNN0_HIDD_DIM1; j++) {
    for (uint16_t i = 0; i < GRNN0_HIDD_DIM0; i++) {
      output[j] += (GRNN0_W[j][i] * input[i]);
    }
  }

  for (uint16_t j = 0; j < GRNN0_HIDD_DIM1; j++) {
    for (uint16_t i = 0; i < GRNN0_HIDD_DIM1; i++) {
      output[j] += (GRNN0_U[j][i] * hidden[i]);
    }
  }

  for (uint16_t j = 0; j < GRNN0_HIDD_DIM1; j++) {
    z = output[j] + GRNN0_BIAS_GATE[j];
    z = sigmoidf(z);
    c = output[j] + GRNN0_BIAS_UPDATE[j];
    c = tanhf(c);

    output[j] =
        z * hidden[j] + (GRNN0_SIGM_ZETA * (1.0 - z) + GRNN0_SIGM_NU) * c;
    // output[j] = std::clamp(output[j], -3.0f, 3.0f);
  }
}

static void __time_critical_func(fc_process)(const rnn_num_t input[FC_IN_DIM],
                                             rnn_num_t output[FC_OUT_DIM]) {
  memset(output, 0, FC_OUT_DIM * sizeof(rnn_num_t));

  for (size_t j = 0; j < FC_OUT_DIM; j++) {
    for (size_t i = 0; i < FC_IN_DIM; i++) {
      output[j] += (input[i] * FC_W[j][i]);
    }
    output[j] += FC_B[j];
    output[j] = hardsigmoid(output[j]);
  }
}

static void rnn_process(const rnn_num_t input[GRNN0_HIDD_DIM0],
                        rnn_num_t output[GRNN0_HIDD_DIM1]) {
  static rnn_num_t hidden[GRNN0_HIDD_DIM1] = {0};

  memset(output, 0, sizeof(rnn_num_t) * GRNN0_HIDD_DIM1);
  _rnn_process(input, hidden, output);
  memcpy(hidden, output, sizeof(rnn_num_t) * GRNN0_HIDD_DIM1);
}

// static uint32_t __time_critical_func(avg)(uint16_t x[RNND_NFFT]) {
//   static uint32_t avg = 0;
//   uint32_t m = std::accumulate(x, x + RNND_NFFT, 0);
//   m /= RNND_NFFT;
//   avg += m - (avg / 128);

//   return avg;
// }

// static void interp(rnn_num_t g_in[MEL_BINS], rnn_num_t g_out[RNND_NFFT]) {
//   // interpolate gains
//   int16_t x1 = MEL_IDX[0];
//   rnn_num_t y1 = g_in[0];
//   uint16_t k = 0;

//   memset(g_out, 0, RNND_NFFT * sizeof(rnn_num_t));
//   for (uint16_t i = 1; i < MEL_BINS + 1; i++) {
//     const int16_t x0 = x1;
//     const rnn_num_t y0 = y1;
//     if (i < MEL_BINS) {
//       y1 = g_in[i];
//       x1 = MEL_IDX[i];
//     } else {
//       x1++;
//     }
//     const rnn_num_t d = (y1 - y0) / (x1 - x0);
//     for (uint16_t j = 0; j < (x1 - x0); j++) {
//       g_out[k] = std::clamp(y0 + j * d, 0.0f, 1.0f);
//       k++;
//     }
//   }
// }

static void interp(rnn_num_t g_in[MEL_BINS], rnn_num_t g_out[RNND_NFFT]) {
  memset(g_out, 0, RNND_NFFT * sizeof(rnn_num_t));
  for (uint16_t i = 0; i < MEL_BINS; i++) {
    for (uint16_t j = 0; j < FB_LEN; j++) {
      if (FILTERBANK_MAT[i][j] > 0.0f) {
        g_out[FILTERBANK_OFF[i] + j] += 2 * g_in[i] * FILTERBANK_MAT[i][j];
      }
    }
  }
}

// static void postfilter(uint16_t x[last_bin], float g[last_bin]) {
//   rnn_num_t gw[last_bin] = {0.0f};
//   rnn_num_t e0 = 0.0f;
//   rnn_num_t e1 = 0.0f;
//   for (uint16_t i = 0; i < last_bin; i++) {
//     gw[i] = g[i] * sinf((M_PI / 2) * g[i]);
//     const float a = (x[i] / 32767.0);
//     e0 += g[i] * a;
//     e1 += gw[i] * a;
//   }
//   if (e1 == 0.0f) {
//     e1 = 1e-10f;
//   }
//   const rnn_num_t beta = 0.02;
//   const rnn_num_t r = (e0 / e1);
//   const rnn_num_t G = sqrtf(((1 + beta) * (e0 / e1)) / (1 + beta * (r * r)));
//   for (uint16_t i = 0; i < last_bin; i++) {
//     g[i] = gw[i] * G;
//   }
// }

void rnn_denoiser_denoise(rnn_num_t x[RNND_NFFT], rnn_num_t g[RNND_NFFT]) {
  rnn_num_t out[GRNN0_HIDD_DIM1] = {0};
  rnn_num_t input[MEL_BINS];

  for (uint16_t i = 0; i < MEL_BINS; i++) {
    rnn_num_t s = 0;
    for (uint16_t j = 0; j < FB_LEN; j++) {
      if (FILTERBANK_MAT[i][j] > 0) {
        s += FILTERBANK_MAT[i][j] * x[FILTERBANK_OFF[i] + j];
      }
    }

    input[i] = ((log10f(1e-8f + s) - MEAN_VEC[i]) / STD_VEC[i]);
  }

  rnn_process(input, out);
  fc_process(out, input);
  interp(input, g);
  // postfilter(x, g);
}
