#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pthread.h>
#include <signal.h>

#include <complex.h>
#include <math.h>

#include <argp.h>

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include <liquid/liquid.h>

#include "dsd_in.h"
#include "shared.h"
#include "logging.h"

#define AUDIO_SAMPLERATE (48000UL)
#define SIG_SAMPLERATE (12500UL)

#define SDR_INPUT_CHUNK (100000UL)
#define SDR_SAMPLERATE (1000000UL)
#define DEFAULT_SDR_FREQUENCY (160.0e6)
#define DEFAULT_SDR_GAIN (25.0)

#define xstr(s) str(s)
#define str(s) #s

static error_t parse_opt(int key, char *arg, struct argp_state *state);

static proc_chain_t g_chain = {
    .args = {
        .gain = DEFAULT_SDR_GAIN,
        .frequency = DEFAULT_SDR_FREQUENCY,
    }};

static char doc[] =
    "dsd_feeder -- DSD signal pre-processor";

static char args_doc[] = "";

static struct argp_option options[] = {
    {"gain", 'g', "G", 0, "The gain to set in the SDR receiver in [dB] (default: " xstr(DEFAULT_SDR_GAIN) ")"},
    {"frequency", 'f', "FQ", 0, "The receive frequency of the SDR (default: " xstr(DEFAULT_SDR_FREQUENCY) ")"},
    {0}};

static struct argp argp = {options, parse_opt, args_doc, doc};

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
    int ret;
    struct arguments *arguments = state->input;

    switch (key)
    {
    case 'g':
        ret = sscanf(arg, "%f", &arguments->gain);
        if (ret != 1)
        {
            LOG(ERROR, "Failed to parse gain level");
            argp_usage(state);
        }
        break;

    case 'f':
        ret = sscanf(arg, "%f", &arguments->frequency);
        if (ret != 1)
        {
            LOG(ERROR, "Failed to parse frequency");
            argp_usage(state);
        }
        break;

    case ARGP_KEY_ARG:
        if (state->arg_num >= 0)
            argp_usage(state);

        arguments->args[state->arg_num] = arg;
        break;

    case ARGP_KEY_END:
        if (state->arg_num < 0)
            argp_usage(state);
        break;

    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static bool init_liquid(proc_chain_t *chain)
{
    chain->dcblock = iirfilt_crcf_create_dc_blocker(0.0005);
    log_assert(chain->dcblock);

    chain->res_down = msresamp_crcf_create(((float)SIG_SAMPLERATE) / SDR_SAMPLERATE, 60.0f);
    log_assert(chain->res_down);
    // msresamp_crcf_print(chain->res_down);

    chain->res_up = msresamp_rrrf_create(((float)AUDIO_SAMPLERATE) / SIG_SAMPLERATE, 60.0f);
    log_assert(chain->res_up);
    // msresamp_rrrf_print(chain->res_up);

    chain->fm_demod = freqdem_create(0.5f);
    log_assert(chain->fm_demod);

    return true;
}

static void destroy_liquid(proc_chain_t *chain)
{
    liquid_error_code err;

    err = freqdem_destroy(chain->fm_demod);
    log_assert(err == LIQUID_OK);
    err = msresamp_crcf_destroy(chain->res_down);
    log_assert(err == LIQUID_OK);
    err = iirfilt_crcf_destroy(chain->dcblock);
    log_assert(err == LIQUID_OK);
}

int main(int argc, char *argv[])
{
    bool ret;
    int read, flags;
    long long timeNs;
    unsigned int ny;
    unsigned int nz;
    proc_chain_t *chain = &g_chain;

    complex float buffp[SDR_INPUT_CHUNK];
    void *buffs[] = {buffp};
    size_t res_size = (size_t)ceilf(1 + 2 * SDR_INPUT_CHUNK * ((float)SIG_SAMPLERATE / SDR_SAMPLERATE));
    size_t out_size = (size_t)ceilf(1 + 2 * res_size * ((float)AUDIO_SAMPLERATE / SIG_SAMPLERATE));
    complex float resamp_buf[res_size];
    float fm_out_buf[res_size];
    float out_buf[out_size];
    int16_t buf_out_s[res_size];

    logging_init();

    argp_parse(&argp, argc, argv, 0, 0, &chain->args);

    ret = init_liquid(chain);
    log_assert(ret);

    ret = init_soapy(chain);
    if (!ret)
    {
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        read = SoapySDRDevice_readStream(chain->sdr, chain->rxStream, buffs, SDR_INPUT_CHUNK, &flags, &timeNs, 200000);
        if (read < 0)
        {
            LOG(ERROR, "Reading stream failed with error code: %d", read);
            continue;
        }
        iirfilt_crcf_execute_block(chain->dcblock, buffp, read, buffp);
        msresamp_crcf_execute(chain->res_down, buffp, read, resamp_buf, &ny);
        freqdem_demodulate_block(chain->fm_demod, resamp_buf, ny, fm_out_buf);
        msresamp_rrrf_execute(chain->res_up, fm_out_buf, ny, out_buf, &nz);

        for (size_t i = 0; i < nz; i++)
        {
            buf_out_s[i] = out_buf[i] * INT16_MAX;
        }

        size_t written = fwrite(buf_out_s, 2, nz, stdout);
        log_assert(written == nz);
        fflush(stdout);
    }

    destroy_soapy(chain);
    destroy_liquid(chain);

    LOG(INFO, "Exiting");
    exit(EXIT_SUCCESS);
}