#include "shared.h"

#include <stdint.h>
#include <string.h>

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include "logging.h"

bool init_soapy(proc_chain_t *chain)
{
    int ret;
    size_t length;
    char *driver_name = NULL;

    SoapySDRKwargs *results = SoapySDRDevice_enumerate(NULL, &length);

    for (size_t i = 0; i < length; i++)
    {
        LOG(INFO, "Found device #%d: ", (int)i);
        for (size_t j = 0; j < results[i].size; j++)
        {
            LOG(INFO, "%s=%s, ", results[i].keys[j], results[i].vals[j]);
            if (strncmp(results[i].keys[j], "driver", sizeof("driver")) == 0)
            {
                driver_name = results[i].vals[j];
            }
        }
    }

    if (driver_name)
    {
        LOG(INFO, "Using %s device", driver_name);

        SoapySDRKwargs args = {0};
        SoapySDRKwargs_set(&args, "driver", driver_name);
        chain->sdr = SoapySDRDevice_make(&args);
        log_assert(chain->sdr);
        SoapySDRKwargs_clear(&args);

        SoapySDRRange *ranges = SoapySDRDevice_getFrequencyRange(chain->sdr, SOAPY_SDR_RX, 0, &length);
        LOG(INFO, "Rx freq ranges: ");
        for (size_t i = 0; i < length; i++)
            LOG(INFO, "[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
        free(ranges);

        size_t num_rxch = SoapySDRDevice_getNumChannels(chain->sdr, SOAPY_SDR_RX);
        LOG(INFO, "Rx num channels: %lu", num_rxch);
        log_assert(num_rxch == 1);

        ret = SoapySDRDevice_setSampleRate(chain->sdr, SOAPY_SDR_RX, 0, SDR_SAMPLERATE);
        log_assert(ret == 0);
        ret = SoapySDRDevice_setFrequency(chain->sdr, SOAPY_SDR_RX, 0, chain->args.frequency, NULL);
        log_assert(ret == 0);
        int err = SoapySDRDevice_setGain(chain->sdr, SOAPY_SDR_RX, 0, chain->args.gain);
        if (err != 0)
        {
            SoapySDRDevice_unmake(chain->sdr);
            return false;
        }
        chain->rxStream = SoapySDRDevice_setupStream(chain->sdr, SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL);
        log_assert(ret == 0);
        ret = SoapySDRDevice_activateStream(chain->sdr, chain->rxStream, 0, 0, 0);
        log_assert(ret == 0);
        SoapySDRKwargsList_clear(results, length);

        return true;
    }
    else
    {
        LOG(ERROR, "No Soapy SDR device found");
        SoapySDRKwargsList_clear(results, length);
        return false;
    }
}

void destroy_soapy(proc_chain_t *chain)
{
    int ret;

    ret = SoapySDRDevice_deactivateStream(chain->sdr, chain->rxStream, 0, 0);
    log_assert(ret == 0);
    ret = SoapySDRDevice_closeStream(chain->sdr, chain->rxStream);
    log_assert(ret == 0);

    SoapySDRDevice_unmake(chain->sdr);
}