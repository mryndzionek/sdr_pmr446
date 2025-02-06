#ifndef __SDR_PMR446_H__
#define __SDR_PMR446_H__

#include <stdint.h>
#include <stddef.h>

#include <SoapySDR/Device.h>

#include <liquid/liquid.h>

#include <rtaudio/rtaudio_c.h>

#define SDR_SAMPLERATE (1024000UL)
#define CTCSS_NUM_FREQS (38U)

typedef enum
{
    proc_scanning = 0,
    proc_tuned,
} proc_chain_state_e;

typedef enum
{
    lock_mode_start = 0,
    lock_mode_max,
} lock_mode_e;

struct arguments
{
    char *args[1];
    float frequency;
    float gain;
    float audio_gain;
    enum rtaudio_api audio_api;
    float squelch_level;
    size_t waterfall;
    bool lowpass;
    uint64_t channel_mask;
    lock_mode_e lock_mode;
};

typedef struct {
    float k[CTCSS_NUM_FREQS];
    float coef[CTCSS_NUM_FREQS];
    float u0[CTCSS_NUM_FREQS];
    float u1[CTCSS_NUM_FREQS];
    float power[CTCSS_NUM_FREQS];
    float max_power;
    int max_power_index;
    size_t samp_processed;
    bool tone_detected;
} ctcss_detector_t;

struct _proc_chain_t
{
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    rtaudio_t dac;
    iirfilt_crcf dcblock;
    msresamp_crcf resampler;
    nco_crcf nco;
    firpfbch_crcf channelizer;
    freqdem fm_demod;
    firfilt_rrrf ctcss_filt;
    wdelayf ctcss_lp_delay;
    iirfilt_rrrf ctcss_dcblock;
    firfilt_rrrf audio_filt;
#ifdef APP_FIR_DEEMPH
    firfilt_rrrf deemph;
#else
    iirfilt_rrrf deemph;
#endif
    cbuffercf resamp_buf;
    cbufferf audio_buf;
    asgramcf asgram;
    proc_chain_state_e state;
    ctcss_detector_t *ctcss_detector;
    struct arguments args;
    int active_chan;
    float rssi;
    float ctcss_freq;
};

#endif // __SDR_PMR446_H__