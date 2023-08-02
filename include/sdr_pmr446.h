#ifndef __SDR_PMR446_H__
#define __SDR_PMR446_H__

#include <stdint.h>
#include <stddef.h>

#include <SoapySDR/Device.h>

#include <liquid/liquid.h>

#include <rtaudio/rtaudio_c.h>

#define SDR_SAMPLERATE (1000000UL)

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
    firfilt_rrrf audio_filt;
    firdecim_rrrf ctcss_decim;
    iirfilt_rrrf deemph;
    cbufferf ctcss_buf;
    cbuffercf resamp_buf;
    cbufferf audio_buf;
    spgramf ctcss_spgram;
    asgramcf asgram;
    proc_chain_state_e state;
    struct arguments args;
    int active_chan;
    float rssi;
    float ctcss_freq;
};

#endif // __SDR_PMR446_H__