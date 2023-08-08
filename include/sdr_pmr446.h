#ifndef __SDR_PMR446_H__
#define __SDR_PMR446_H__

#include <stdint.h>
#include <stddef.h>

#include <SoapySDR/Device.h>

#include <liquid/liquid.h>

#include <rtaudio/rtaudio_c.h>

#define SDR_SAMPLERATE (1024000UL)

typedef enum
{
    proc_scanning = 0,
    proc_tuned,
} proc_chain_state_e;

struct arguments
{
    char *args[1];
    float frequency;
    float gain;
    float audio_gain;
    float squelch_level;
    size_t waterfall;
    bool lowpass;
    uint16_t channel_mask;
};

typedef struct
{
    iirfilt_rrrf out_filt;
    iirfilt_rrrf lock_filt;
    agc_rrrf agc;
    float ref_sig;
    float integral;
    float phase;
    float output;
    bool locked;
} pll_t;

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
    pll_t *pll;
    firfilt_rrrf ctcss_filt;
    firfilt_rrrf ctcss_lp_filt;
    iirfilt_rrrf ctcss_dcblock;
    firfilt_rrrf audio_filt;
#ifdef APP_FIR_DEEMPH
    firfilt_rrrf deemph;
#else
    iirfilt_rrrf deemph;
#endif
    cbufferf ctcss_buf;
    cbuffercf resamp_buf;
    cbufferf audio_buf;
    asgramcf asgram;
    proc_chain_state_e state;
    struct arguments args;
    int active_chan;
    float rssi;
    float ctcss_freq;
};

#endif // __SDR_PMR446_H__