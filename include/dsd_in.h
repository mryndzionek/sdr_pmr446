#ifndef __DSD_IN_H__
#define __DSD_IN_H__

#include <stdint.h>
#include <stddef.h>

#include <SoapySDR/Device.h>

#include <liquid/liquid.h>

#define SDR_SAMPLERATE (1000000UL)

struct arguments
{
    char *args[1];
    float gain;
    float frequency;
};

struct _proc_chain_t
{
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    iirfilt_crcf dcblock;
    msresamp_crcf res_down;
    msresamp_rrrf res_up;
    freqdem fm_demod;
    struct arguments args;
};

#endif // __SDR_PMR446_H__
