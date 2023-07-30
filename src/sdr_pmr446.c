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

#include <rtaudio/rtaudio_c.h>

#include "logging.h"

#define AUDIO_SAMPLERATE (12500UL)

#define CTCSS_DECIM (20) // 12.5kHz / 20 = 625Hz - just enough to keep CTCSS frequencies
#define CTCSS_FFT_SIZE (2048)
#define CTCSS_FS ((float)AUDIO_SAMPLERATE / CTCSS_DECIM)
#define CTCSS_FREQ_START (size_t)(50.0f * (CTCSS_FFT_SIZE / 2) / ((float)AUDIO_SAMPLERATE / CTCSS_DECIM))
#define CTCSS_BIN_TO_FREQ(_bin) (((float)_bin) / ((CTCSS_FFT_SIZE / 2) - 1)) * (CTCSS_FS / 2)
#define CTCSS_FREQ_TO_BIN(_freq) ((_freq / (CTCSS_FS / 2)) * ((CTCSS_FFT_SIZE / 2) - 1))

#define SDR_INPUT_CHUNK (100000UL)
#define SDR_SAMPLERATE (1000000UL)
#define SDR_RESAMPLERATE (200000UL)
#define SDR_FREQUENCY (446.1e6)
#define SDR_NUM_CHANNELS (16)
#define SDR_DEFAULT_GAIN (42.0f)
#define SDR_DEFAULT_SQUELCH_LEVEL (-5.0f)

#define HP_AUDIO_FILT_TAPS (297)
#define LP_AUDIO_FILT_TAPS (103)

#define CSAMPLE_SIZE sizeof(complex float)

typedef enum
{
    proc_scanning = 0,
    proc_tuned,
} proc_chain_state_e;

struct arguments
{
    char *args[1];
    float gain;
    float audio_gain;
    float squelch_level;
    size_t waterfall;
    bool lowpass;
    uint16_t channel_mask;
};

typedef struct
{
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    rtaudio_t dac;
    iirfilt_crcf dcblock;
    msresamp_crcf resampler;
    nco_crcf nco;
    firpfbch_crcf channelizer;
    freqdem fm_dems[SDR_NUM_CHANNELS];
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
} proc_chain_t;

static error_t parse_opt(int key, char *arg, struct argp_state *state);

static const float hp_audio_taps[HP_AUDIO_FILT_TAPS] = {
    0.005384938903483371, -0.005936825646586213, -0.0033436665991168797, -0.001765816520246501, -0.0008061590857562453, -0.0002208307554787034, 0.00013835619602251014, 0.00036230239900853195,
    0.0005051743448603659, 0.000600697837014515, 0.0006674456728712143, 0.000717389766798537, 0.0007555448510253917, 0.0007862046565071039, 0.0008092718685489552, 0.000825386260739842,
    0.0008334372055351342, 0.0008332510104074064, 0.0008229276314153384, 0.0008019106973748077, 0.0007691051694296978, 0.0007240794490829188, 0.0006659774729267424, 0.0005951577092357883,
    0.0005114991013383065, 0.00041531175377151864, 0.000307106425911715, 0.00018858293549375065, 6.096197445389482e-05, -7.413172402081411e-05, -0.00021456760688803015, -0.0003579537909905959,
    -0.0005018744016025006, -0.0006442874392247822, -0.0007813118265064591, -0.0009117446435547944, -0.0010308845279011882, -0.0011384902281097787, -0.0012273145427333073, -0.0013034623477523805,
    -0.001349032550374798, -0.0013883700034329877, -0.001382743592377297, -0.0013680809505238485, -0.0013246977327473263, -0.001241930868925889, -0.00114589043607186, -0.0010227365614696971,
    -0.0008678832789745377, -0.000687752279809724, -0.000494685161020134, -0.0002809328944984459, -4.838162615722162e-05, 0.00019942098375331488, 0.0004501003623223662, 0.0007033370359240964,
    0.0009560060036493782, 0.0012074016511012726, 0.0014481629137008299, 0.0016728949767929522, 0.0018741103018121532, 0.002051164474161556, 0.0021999150774560158, 0.0023180818022159517,
    0.0023980620560396988, 0.0024371816249535087, 0.002431267105069152, 0.002381246619974327, 0.0022850800997866213, 0.002144761517887448, 0.001958437980100096, 0.0017276793705736243,
    0.0014524378088807209, 0.0011375406354396814, 0.0007861907353318651, 0.0004044216341411993, -1.994604882637623e-06, -0.00042673623187590627, -0.0008637650292363917, -0.0013076405152675624,
    -0.001747936687336836, -0.00218100160101276, -0.0025925678075127303, -0.0029787618850982698, -0.003329223984866233, -0.003633193027989576, -0.003891114406568173, -0.00408665700214583,
    -0.004220114640266849, -0.004284827268253971, -0.0042760798326191415, -0.004186452253425127, -0.004020343868122673, -0.003771016990607998, -0.0034403623595460095, -0.0030289429983876,
    -0.002543598640452098, -0.0019851239892907362, -0.0013612443406613372, -0.0006789085607650197, 5.052183596765787e-05, 0.0008205379288353497, 0.0016188622702622392, 0.0024345100575554755,
    0.003251647974226035, 0.004058711769605269, 0.0048395117456365615, 0.0055812702805170455, 0.0062664351616240795, 0.006881418533959773, 0.00740916548600663, 0.007838155169242354,
    0.008153276359725044, 0.008342870857875728, 0.008392957564273312, 0.008295563798653515, 0.008041238911505993, 0.007623621118824982, 0.00703606038542802, 0.006276171340662674,
    0.00534187980794798, 0.004234000446379033, 0.002956679150286743, 0.0015151756022497257, -8.563521751863441e-05, -0.0018326176749520115, -0.0037189049314402026, -0.005725335339112428,
    -0.007841128999944632, -0.01004521575359379, -0.012321563211346307, -0.014647319862802757, -0.01700576795556414, -0.01936973429113779, -0.021721457185272484, -0.02403580875656689,
    -0.026290474358580986, -0.02846175221381245, -0.03053057173476118, -0.032473804929325945, -0.03427216645450346, -0.035905583068188064, -0.037359028816644275, -0.03861503182478766,
    -0.03966226915461611, -0.040488166736827674, -0.04108573768781846, -0.04144585434831453, 0.9584330658656965, -0.04144585434831453, -0.04108573768781846, -0.040488166736827674,
    -0.03966226915461611, -0.03861503182478766, -0.037359028816644275, -0.035905583068188064, -0.03427216645450346, -0.032473804929325945, -0.03053057173476118, -0.02846175221381245,
    -0.026290474358580986, -0.02403580875656689, -0.021721457185272484, -0.01936973429113779, -0.01700576795556414, -0.014647319862802757, -0.012321563211346307, -0.01004521575359379,
    -0.007841128999944632, -0.005725335339112428, -0.0037189049314402026, -0.0018326176749520115, -8.563521751863441e-05, 0.0015151756022497257, 0.002956679150286743, 0.004234000446379033,
    0.00534187980794798, 0.006276171340662674, 0.00703606038542802, 0.007623621118824982, 0.008041238911505993, 0.008295563798653515, 0.008392957564273312, 0.008342870857875728,
    0.008153276359725044, 0.007838155169242354, 0.00740916548600663, 0.006881418533959773, 0.0062664351616240795, 0.0055812702805170455, 0.0048395117456365615, 0.004058711769605269,
    0.003251647974226035, 0.0024345100575554755, 0.0016188622702622392, 0.0008205379288353497, 5.052183596765787e-05, -0.0006789085607650197, -0.0013612443406613372, -0.0019851239892907362,
    -0.002543598640452098, -0.0030289429983876, -0.0034403623595460095, -0.003771016990607998, -0.004020343868122673, -0.004186452253425127, -0.0042760798326191415, -0.004284827268253971,
    -0.004220114640266849, -0.00408665700214583, -0.003891114406568173, -0.003633193027989576, -0.003329223984866233, -0.0029787618850982698, -0.0025925678075127303, -0.00218100160101276,
    -0.001747936687336836, -0.0013076405152675624, -0.0008637650292363917, -0.00042673623187590627, -1.994604882637623e-06, 0.0004044216341411993, 0.0007861907353318651, 0.0011375406354396814,
    0.0014524378088807209, 0.0017276793705736243, 0.001958437980100096, 0.002144761517887448, 0.0022850800997866213, 0.002381246619974327, 0.002431267105069152, 0.0024371816249535087,
    0.0023980620560396988, 0.0023180818022159517, 0.0021999150774560158, 0.002051164474161556, 0.0018741103018121532, 0.0016728949767929522, 0.0014481629137008299, 0.0012074016511012726,
    0.0009560060036493782, 0.0007033370359240964, 0.0004501003623223662, 0.00019942098375331488, -4.838162615722162e-05, -0.0002809328944984459, -0.000494685161020134, -0.000687752279809724,
    -0.0008678832789745377, -0.0010227365614696971, -0.00114589043607186, -0.001241930868925889, -0.0013246977327473263, -0.0013680809505238485, -0.001382743592377297, -0.0013883700034329877,
    -0.001349032550374798, -0.0013034623477523805, -0.0012273145427333073, -0.0011384902281097787, -0.0010308845279011882, -0.0009117446435547944, -0.0007813118265064591, -0.0006442874392247822,
    -0.0005018744016025006, -0.0003579537909905959, -0.00021456760688803015, -7.413172402081411e-05, 6.096197445389482e-05, 0.00018858293549375065, 0.000307106425911715, 0.00041531175377151864,
    0.0005114991013383065, 0.0005951577092357883, 0.0006659774729267424, 0.0007240794490829188, 0.0007691051694296978, 0.0008019106973748077, 0.0008229276314153384, 0.0008332510104074064,
    0.0008334372055351342, 0.000825386260739842, 0.0008092718685489552, 0.0007862046565071039, 0.0007555448510253917, 0.000717389766798537, 0.0006674456728712143, 0.000600697837014515,
    0.0005051743448603659, 0.00036230239900853195, 0.00013835619602251014, -0.0002208307554787034, -0.0008061590857562453, -0.001765816520246501, -0.0033436665991168797, -0.005936825646586213,
    0.005384938903483371};

static const float lp_audio_taps[LP_AUDIO_FILT_TAPS] = {
    0.0024625252581829222, 0.006537983611116499, 0.0012087578704422634, -0.002873891861196485, 0.0020197139081498763, -0.00020231477562747817, -0.0015676437222722623, 0.00240511622707016,
    -0.0017624420520179636, -0.00011996736535666206, 0.0021759811087151085, -0.0030403593029908445, 0.0019289826761417109, 0.0006990827320644029, -0.0032511167000401735, 0.003935575635574618,
    -0.0019782367092011687, -0.0016641340641698839, 0.004706624061935969, -0.004940034625783954, 0.001776690600870344, 0.0031001267564166467, -0.00655946446160198, 0.005991648406762622,
    -0.001196288887995427, -0.005138564398146148, 0.008869316251457293, -0.0070365281802642, 7.751185646366202e-05, 0.007984935077137816, -0.011775750198667986, 0.008033399308766315,
    0.0018444889506240545, -0.012037893990034527, 0.015581565774550443, -0.008925070595746567, -0.005095511246867055, 0.018117892399705158, -0.020966201191718638, 0.009675021096500844,
    0.010867010402385538, -0.028409580103409535, 0.029962780352374836, -0.010234506287940365, -0.023106944235292075, 0.0509457430276393, -0.051208926794058175, 0.010582062891624998,
    0.0667446211902592, -0.158447357869086, 0.23238884805555016, 0.7392900466551452, 0.23238884805555016, -0.158447357869086, 0.0667446211902592, 0.010582062891624998,
    -0.051208926794058175, 0.0509457430276393, -0.023106944235292075, -0.010234506287940365, 0.029962780352374836, -0.028409580103409535, 0.010867010402385538, 0.009675021096500844,
    -0.020966201191718638, 0.018117892399705158, -0.005095511246867055, -0.008925070595746567, 0.015581565774550443, -0.012037893990034527, 0.0018444889506240545, 0.008033399308766315,
    -0.011775750198667986, 0.007984935077137816, 7.751185646366202e-05, -0.0070365281802642, 0.008869316251457293, -0.005138564398146148, -0.001196288887995427, 0.005991648406762622,
    -0.00655946446160198, 0.0031001267564166467, 0.001776690600870344, -0.004940034625783954, 0.004706624061935969, -0.0016641340641698839, -0.0019782367092011687, 0.003935575635574618,
    -0.0032511167000401735, 0.0006990827320644029, 0.0019289826761417109, -0.0030403593029908445, 0.0021759811087151085, -0.00011996736535666206, -0.0017624420520179636, 0.00240511622707016,
    -0.0015676437222722623, -0.00020231477562747817, 0.0020197139081498763, -0.002873891861196485, 0.0012087578704422634, 0.006537983611116499, 0.0024625252581829222};

static const float ctcss_freqs[38] = {
    67.0f, 71.9f, 74.4f, 77.0f, 79.7f, 82.5f, 85.4f, 88.5f, 91.5f, 94.8f, 97.4f, 100.0f, 103.5f, 107.2f,
    110.9f, 114.8f, 118.8f, 123.0f, 127.3f, 131.8f, 136.5f, 141.3f, 146.2f, 151.4f, 156.7f, 162.2f,
    167.9f, 173.8f, 179.9f, 186.2f, 192.8f, 203.5f, 210.7f, 218.1f, 225.7f, 233.6f, 241.8f, 250.3f};

static proc_chain_t g_chain = {
    .state = proc_scanning,
    .active_chan = -1,
    .ctcss_freq = -1.0,
    .args = {
        .gain = SDR_DEFAULT_GAIN,
        .audio_gain = 4.0f,
        .squelch_level = SDR_DEFAULT_SQUELCH_LEVEL,
        .waterfall = 0,
        .lowpass = false,
        .channel_mask = 0xFFFF}};

static pthread_mutex_t lock;
static bool exit_via_sig;

static char doc[] =
    "rtl_pmr446 -- a PMR446 band scanner/receiver";

static char args_doc[] = "";

static struct argp_option options[] = {
    {"gain", 'g', "G", 0, "The gain to set in the SDR receiver in [dB] (default: 42.0dB)"},
    {"squelch", 's', "SQ", 0, "The squelch level in [dB] (default: -5.0dB)"},
    {"waterfall", 'w', "WT", 0, "If specified an ASCII waterfall is printed on the screen"},
    {"lowpass", 'l', 0, 0, "Turn on 4.5kHz lowpass audio filter (might reduce noise)"},
    {"mask", 'm', "CM", 0, "Channel mask e.g. 1,2,8-16 (only listen to channels 1,2 and 8 to 16)"},
    {"audio-gain", 'a', "AG", 0, "The gain to set in the SDR receiver in (default: 4.0)"},
    {0}};

static struct argp argp = {options, parse_opt, args_doc, doc};

static void sighandler(int signum)
{
    if (signum == SIGPIPE)
    {
        signal(SIGPIPE, SIG_IGN);
    }
    else if (signum == SIGUSR1)
    {
        return;
    }
    else
    {
        fprintf(stderr, "Signal caught, exiting!\n");
    }
    exit_via_sig = true;
}

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
    int ret;
    struct arguments *arguments = state->input;

    switch (key)
    {
    case 'w':
        arguments->waterfall = atoll(arg);
        break;

    case 's':
        ret = sscanf(arg, "%f", &arguments->squelch_level);
        if (ret != 1)
        {
            LOG(ERROR, "Failed to parse the squelch level");
            argp_usage(state);
        }
        break;

    case 'g':
        ret = sscanf(arg, "%f", &arguments->gain);
        if (ret != 1)
        {
            LOG(ERROR, "Failed to parse gain level");
            argp_usage(state);
        }
        break;

    case 'a':
        ret = sscanf(arg, "%f", &arguments->audio_gain);
        if (ret != 1)
        {
            LOG(ERROR, "Failed to parse gain level");
            argp_usage(state);
        }
        break;

    case 'l':
        arguments->lowpass = true;
        break;

    case 'm':
    {
        long l, r;
        arguments->channel_mask = 0;

        while (*arg)
        {
            for (l = r = 0; *arg && isdigit(*arg); arg++)
                l = (l * 10) + (*arg - '0');

            if (*arg == '-')
            {
                arg++;
                for (; *arg && isdigit(*arg); arg++)
                    r = (r * 10) + (*arg - '0');
            }
            else
                r = l;

            if ((l < 1) || (l > 16))
            {
                LOG(ERROR, "The channels specified in channel mask must be in the range 1-16");
                argp_usage(state);
            }

            if ((r < 1) || (r > 16))
            {
                LOG(ERROR, "The channels specified in channel mask must be in the range 1-16");
                argp_usage(state);
            }

            for (; l <= r; l++)
            {
                arguments->channel_mask |= (1UL << (l - 1));
            }

            while (*arg && !isdigit(*arg))
                arg++;
        }
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

static int find_ctcss_code(float ctcss_freq)
{
    if ((ctcss_freq >= 67.0f) && (ctcss_freq <= 250.3f))
    {
        float min_dist = fabs(ctcss_freq - ctcss_freqs[0]);
        int min_i = 0;

        for (size_t i = 1; i < sizeof(ctcss_freqs) / sizeof(ctcss_freqs[0]); i++)
        {
            float dist = fabs(ctcss_freq - ctcss_freqs[i]);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_i = i;
            }
        }

        return min_i + 1;
    }
    else
    {
        return -1;
    }
}

static float average_power(complex float const *data, size_t len)
{
    float a = 0.0;
    for (size_t k = 0; k < len; k++)
    {
        a += cabsf(data[k]);
    }

    a /= len;
    return 20 * log10f(a);
}

static bool init_soapy(proc_chain_t *chain, float gain)
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
        ret = SoapySDRDevice_setFrequency(chain->sdr, SOAPY_SDR_RX, 0, SDR_FREQUENCY, NULL);
        log_assert(ret == 0);
        int err = SoapySDRDevice_setGain(chain->sdr, SOAPY_SDR_RX, 0, gain);
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

static void destroy_soapy(proc_chain_t *chain)
{
    int ret;

    ret = SoapySDRDevice_deactivateStream(chain->sdr, chain->rxStream, 0, 0);
    log_assert(ret == 0);
    ret = SoapySDRDevice_closeStream(chain->sdr, chain->rxStream);
    log_assert(ret == 0);

    SoapySDRDevice_unmake(chain->sdr);
}

static bool init_liquid(proc_chain_t *chain, size_t asgram_len,
                        size_t resamp_buf_size, size_t ctcss_buf_size)
{
    chain->dcblock = iirfilt_crcf_create_dc_blocker(0.0005);
    log_assert(chain->dcblock);

    chain->resampler = msresamp_crcf_create(((float)SDR_RESAMPLERATE) / SDR_SAMPLERATE, 60.0f);
    log_assert(chain->resampler);
    msresamp_crcf_print(chain->resampler);

    chain->nco = nco_crcf_create(LIQUID_VCO);
    log_assert(chain->nco);
    float offset = -0.5f * (float)(SDR_NUM_CHANNELS - 1) / (float)SDR_NUM_CHANNELS * 2 * M_PI;
    nco_crcf_set_frequency(chain->nco, offset);

    chain->channelizer = firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, SDR_NUM_CHANNELS, 13, 80.0);
    log_assert(chain->channelizer);

    for (size_t i = 0; i < SDR_NUM_CHANNELS; i++)
    {
        chain->fm_dems[i] = freqdem_create(0.5f);
        log_assert(chain->fm_dems[i]);
    }

    chain->ctcss_filt = firfilt_rrrf_create((float *)hp_audio_taps, HP_AUDIO_FILT_TAPS);
    log_assert(chain->ctcss_filt);

    chain->audio_filt = firfilt_rrrf_create((float *)lp_audio_taps, LP_AUDIO_FILT_TAPS);
    log_assert(chain->audio_filt);

    chain->deemph = iirfilt_rrrf_create_lowpass(2, 2000.0 / AUDIO_SAMPLERATE);
    // chain->deemph = iirfilt_rrrf_create((float[]){0.558390545416219, 0.558390545416219}, 2,
    //                                     (float[]){1.0, 0.11678109083243816}, 2);
    log_assert(chain->deemph);
    iirfilt_rrrf_print(chain->deemph);

    chain->resamp_buf = cbuffercf_create(resamp_buf_size);
    log_assert(chain->resamp_buf);

    chain->audio_buf = cbufferf_create(AUDIO_SAMPLERATE);
    log_assert(chain->audio_buf);

    chain->ctcss_decim = firdecim_rrrf_create_kaiser(CTCSS_DECIM, 8, 60.0f);
    log_assert(chain->ctcss_decim);

    chain->ctcss_buf = cbufferf_create(2 * ctcss_buf_size);
    log_assert(chain->ctcss_buf);

    chain->ctcss_spgram = spgramf_create_default(CTCSS_FFT_SIZE);
    log_assert(chain->ctcss_spgram);

    if (chain->args.waterfall > 0)
    {
        chain->asgram = asgramcf_create(asgram_len);
        log_assert(chain->asgram);
        asgramcf_set_scale(chain->asgram, -40.0f, 5.0f);
    }

    return true;
}

static void destroy_liquid(proc_chain_t *chain)
{
    liquid_error_code err;

    if (chain->args.waterfall > 0)
    {
        err = asgramcf_destroy(chain->asgram);
        log_assert(err == LIQUID_OK);
    }

    err = spgramf_destroy(chain->ctcss_spgram);
    log_assert(err == LIQUID_OK);
    err = cbufferf_destroy(chain->ctcss_buf);
    log_assert(err == LIQUID_OK);
    err = firdecim_rrrf_destroy(chain->ctcss_decim);
    log_assert(err == LIQUID_OK);
    err = cbufferf_destroy(chain->audio_buf);
    log_assert(err == LIQUID_OK);
    err = cbuffercf_destroy(chain->resamp_buf);
    log_assert(err == LIQUID_OK);
    err = iirfilt_rrrf_destroy(chain->deemph);
    log_assert(err == LIQUID_OK);
    err = firfilt_rrrf_destroy(chain->audio_filt);
    log_assert(err == LIQUID_OK);
    err = firfilt_rrrf_destroy(chain->ctcss_filt);
    log_assert(err == LIQUID_OK);

    for (size_t i = 0; i < SDR_NUM_CHANNELS; i++)
    {
        err = freqdem_destroy(chain->fm_dems[i]);
        log_assert(err == LIQUID_OK);
    }

    err = firpfbch_crcf_destroy(chain->channelizer);
    log_assert(err == LIQUID_OK);
    err = nco_crcf_destroy(chain->nco);
    log_assert(err == LIQUID_OK);
    err = msresamp_crcf_destroy(chain->resampler);
    log_assert(err == LIQUID_OK);
    err = iirfilt_crcf_destroy(chain->dcblock);
    log_assert(err == LIQUID_OK);
}

static int
audio_cb(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
         double stream_time, rtaudio_stream_status_t status, void *data)
{
    float *buffer = (float *)outputBuffer;
    cbufferf inBuffer = data;
    unsigned int num_read;
    float *rp;

    pthread_mutex_lock(&lock);
    cbufferf_read(inBuffer, nBufferFrames, &rp, &num_read);
    for (size_t i = 0; i < nBufferFrames; i++)
    {
        if (i < num_read)
        {
            buffer[i] = rp[i];
        }
        else
        {
            buffer[i] = 0.0;
        }
    }
    liquid_error_code err = cbufferf_release(inBuffer, num_read);
    log_assert(err == LIQUID_OK);
    pthread_mutex_unlock(&lock);

    return 0;
}

static void error_cb(rtaudio_error_t err, const char *msg)
{
    LOG(ERROR, "Error type: %d message: %s", err, msg);
}

static bool init_rtaudio(proc_chain_t *chain)
{
    unsigned int bufferFrames = AUDIO_SAMPLERATE / 10;
    const rtaudio_api_t api = rtaudio_compiled_api_by_name("pulse");
    LOG(INFO, "RTAudio API: %s", rtaudio_api_name(api));

    chain->dac = rtaudio_create(api);
    log_assert(chain->dac);
    int n = rtaudio_device_count(chain->dac);

    if (n == 0)
    {
        LOG(ERROR, "No audio devices available");
        return false;
    }
    else if (n == 1)
    {
        LOG(ERROR, "There is %d audio device available", n);
        return false;
    }
    else
    {
        LOG(INFO, "There are %d audio devices available:", n);
    }

    int dev = rtaudio_get_default_output_device(chain->dac);

    for (int i = 0; i < n; i++)
    {
        rtaudio_device_info_t info = rtaudio_get_device_info(chain->dac, i);
        LOG(INFO, "\t\"%s\"%s", info.name, dev == i ? " (default)" : "");
    }

    rtaudio_show_warnings(chain->dac, true);

    rtaudio_stream_parameters_t o_params = {
        .device_id = rtaudio_get_default_output_device(chain->dac),
        .first_channel = 0,
        .num_channels = 1};

    rtaudio_stream_options_t options = {
        .flags = RTAUDIO_FLAGS_HOG_DEVICE | RTAUDIO_FLAGS_MINIMIZE_LATENCY | RTAUDIO_FLAGS_NONINTERLEAVED};

    rtaudio_error_t err = rtaudio_open_stream(chain->dac, &o_params, NULL,
                                              RTAUDIO_FORMAT_FLOAT32,
                                              AUDIO_SAMPLERATE, &bufferFrames, &audio_cb,
                                              (void *)chain->audio_buf, &options,
                                              &error_cb);
    log_assert(err == 0);

    err = rtaudio_start_stream(chain->dac);
    log_assert(err == 0);

    return true;
}

static void destroy_rtaudio(proc_chain_t *chain)
{
    rtaudio_error_t err = rtaudio_stop_stream(chain->dac);
    log_assert(err == 0);
    if (rtaudio_is_stream_open(chain->dac))
    {
        rtaudio_close_stream(chain->dac);
    }

    rtaudio_destroy(chain->dac);
}

static void ctcss_execute(proc_chain_t *chain, float *x, unsigned int n)
{
    unsigned int num_read;
    float *rp;
    float psd[CTCSS_FFT_SIZE];

    // Write to temporary buffer in order to be able
    // to read 'CTCSS_DECIM' samples
    liquid_error_code err = cbufferf_write(chain->ctcss_buf, x, n);
    log_assert(err == LIQUID_OK);

    // Read max multiple of 'CTCSS_DECIM' samples
    unsigned int ctcss_size = cbufferf_size(chain->ctcss_buf) / CTCSS_DECIM;
    float ctcss_tmp[ctcss_size];
    cbufferf_read(chain->ctcss_buf, CTCSS_DECIM * ctcss_size, &rp, &num_read);
    log_assert(num_read == (CTCSS_DECIM * ctcss_size));

    // Decimate the signal to leave only the CTCSS band
    firdecim_rrrf_execute_block(chain->ctcss_decim, rp, ctcss_size, ctcss_tmp);

    // Free the space in the buffer
    err = cbufferf_release(chain->ctcss_buf, num_read);
    log_assert(err == LIQUID_OK);

    // Push the decimated signal to a spectral periodogram
    err = spgramf_write(chain->ctcss_spgram, ctcss_tmp, ctcss_size);
    log_assert(err == LIQUID_OK);

    // Get the spectrum
    spgramf_get_psd(chain->ctcss_spgram, psd);

    float max_psd = -100.0f;
    int max_bin = -1;

    // Find the power and the 'bin' (frequency) of the strongest signal
    // Looking just at the area of interest
    for (size_t k = (CTCSS_FFT_SIZE / 2) + CTCSS_FREQ_TO_BIN(50); k < CTCSS_FFT_SIZE; k++)
    {
        if (psd[k] > max_psd)
        {
            max_psd = psd[k];
            max_bin = k - (CTCSS_FFT_SIZE / 2);
        }
    }

    float ctcss_freq = -1.0f;
    if (max_psd > 5.0f)
    {
        ctcss_freq = CTCSS_BIN_TO_FREQ(max_bin);
    }

    if (chain->args.waterfall == 0)
    {
        if (ctcss_freq > 0.0f)
        {
            if (chain->ctcss_freq != ctcss_freq)
            {
                int code = find_ctcss_code(ctcss_freq);
                if (code > 0)
                {
                    LOG(INFO, "Acquired CTCSS code: %d (frequency: %3.2fHz)", code, ctcss_freq);
                }
                else
                {
                    LOG(INFO, "Acquired CTCSS frequency: %3.2fHz (unknown code)", ctcss_freq);
                }
            }
        }
    }

    chain->ctcss_freq = ctcss_freq;
}

static void refresh_footer(proc_chain_t *chain, char *const footer, size_t w_len)
{
    float ch_width = (float)w_len / (SDR_NUM_CHANNELS);

    for (size_t i = 0; i < SDR_NUM_CHANNELS; i++)
    {
        int pos;
        size_t rpos = roundf((i * ch_width) + (ch_width / 2) + 2);
        if (chain->active_chan == i)
        {
            log_assert(chain->args.channel_mask & (1UL << i));
            pos = snprintf(&footer[rpos], w_len, "%s", "^^");
        }
        else
        {
            if (chain->args.channel_mask & (1UL << i))
            {
                pos = snprintf(&footer[rpos], w_len, "%02ld", i + 1);
            }
            else
            {
                pos = snprintf(&footer[rpos], w_len, "%s", "--");
            }
        }
        footer[rpos + pos] = ' ';
    }

    if (chain->active_chan >= 0)
    {
        int ctcss_code = find_ctcss_code(chain->ctcss_freq);
        if (ctcss_code > 0)
        {
            sprintf(&footer[w_len + 6], "%8.3f MHz [%d]  [CTCSS:  %02d (%3.2fHz)]", SDR_FREQUENCY * 1e-6f,
                    chain->active_chan + 1, ctcss_code, chain->ctcss_freq);
        }
        else
        {
            if (chain->ctcss_freq > 0)
            {
                sprintf(&footer[w_len + 6], "%8.3f MHz [%d]  [CTCSS:  ?? (%3.2f)]",
                        SDR_FREQUENCY * 1e-6f,
                        chain->active_chan + 1, chain->ctcss_freq);
            }
            else
            {
                sprintf(&footer[w_len + 6], "%8.3f MHz [%d]",
                        SDR_FREQUENCY * 1e-6f,
                        chain->active_chan + 1);
            }
        }
    }
    else
    {
        sprintf(&footer[w_len + 6], "%8.3f MHz", SDR_FREQUENCY * 1e-6f);
    }
}

int main(int argc, char *argv[])
{
    bool ret;
    struct sigaction sigact;

    int read, flags;
    long long timeNs;
    unsigned int ny;
    proc_chain_t *chain = &g_chain;

    logging_init();

    argp_parse(&argp, argc, argv, 0, 0, &chain->args);

    LOG(INFO, "gain: %5.2f dB, audio_gain: %5.2f, squelch level: %5.2f dB, waterfall: %ld",
        chain->args.gain, chain->args.audio_gain,
        chain->args.squelch_level,
        chain->args.waterfall);

    LOG(INFO, "audio lowpass: %s, channel mask: 0x%04X",
        chain->args.lowpass ? "enabled" : "disabled",
        chain->args.channel_mask);

    if (chain->args.channel_mask == 0)
    {
        LOG(ERROR, "No channels enabled in channel mask !");
        exit(EXIT_FAILURE);
    }

    size_t res_size = (size_t)ceilf(1 + 2 * SDR_INPUT_CHUNK * ((float)SDR_RESAMPLERATE / SDR_SAMPLERATE));
    complex float buffp[SDR_INPUT_CHUNK];
    complex float resamp_buf[res_size];
    complex float tmp_chan_buf_out[SDR_NUM_CHANNELS];
    void *buffs[] = {buffp};

    size_t chan_size = (size_t)ceilf(res_size / SDR_NUM_CHANNELS);
    complex float chan_bufs[SDR_NUM_CHANNELS][chan_size];
    float fm_out_buf[chan_size];

    // assemble footer
    unsigned int footer_len = chain->args.waterfall + 32;
    char footer[footer_len + 1];

    if (chain->args.waterfall > 0)
    {
        for (size_t i = 0; i < footer_len; i++)
            footer[i] = ' ';
        footer[1] = '[';
        footer[chain->args.waterfall + 4] = ']';
        refresh_footer(chain, footer, chain->args.waterfall);
    }

    float maxval;
    float maxfreq;
    char ascii[chain->args.waterfall + 1];
    ascii[chain->args.waterfall] = '\0';

    read = pthread_mutex_init(&lock, NULL);
    log_assert(read == 0);

    ret = init_liquid(chain, chain->args.waterfall, res_size, chan_size);
    log_assert(ret);

    ret = init_soapy(chain, chain->args.gain);
    if (!ret)
    {
        exit(EXIT_FAILURE);
    }

    ret = init_rtaudio(chain);
    log_assert(ret);

    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigact, NULL);
    sigaction(SIGUSR1, &sigact, NULL);

    while (!exit_via_sig)
    {
        read = SoapySDRDevice_readStream(chain->sdr, chain->rxStream, buffs, SDR_INPUT_CHUNK, &flags, &timeNs, 200000);
        if (read < 0)
        {
            LOG(ERROR, "Reading stream failed with error code: %d", read);
            continue;
        }
        iirfilt_crcf_execute_block(chain->dcblock, buffp, read, buffp);
        msresamp_crcf_execute(chain->resampler, buffp, read, resamp_buf, &ny);
        liquid_error_code err = cbuffercf_write(chain->resamp_buf, resamp_buf, ny);
        log_assert(err == LIQUID_OK);

        size_t ns = 0;
        unsigned int num_read;
        complex float *rpc;

        while (cbuffercf_size(chain->resamp_buf) >= SDR_NUM_CHANNELS)
        {
            cbuffercf_read(chain->resamp_buf, SDR_NUM_CHANNELS, &rpc, &num_read);
            log_assert(num_read == SDR_NUM_CHANNELS);

            for (int i = 0; i < SDR_NUM_CHANNELS; i++)
            {
                complex float *x = &rpc[i];
                nco_crcf_mix_down(chain->nco, *x, x);
                nco_crcf_step(chain->nco);
            }

            firpfbch_crcf_analyzer_execute(chain->channelizer, rpc, tmp_chan_buf_out);
            err = cbuffercf_release(chain->resamp_buf, num_read);
            log_assert(err == LIQUID_OK);

            // transpose channels
            for (size_t i = 0; i < SDR_NUM_CHANNELS; i++)
            {
                chan_bufs[i][ns] = tmp_chan_buf_out[i];
            }
            ns++;
        }

        // Update chain state
        switch (chain->state)
        {
        case proc_scanning:
        {
            size_t s_ch;

            for (s_ch = 0; s_ch < SDR_NUM_CHANNELS; s_ch++)
            {
                if (chain->args.channel_mask & (1UL << s_ch))
                {
                    break;
                }
            }

            log_assert(s_ch < 16);

            float max_rssi = average_power(chan_bufs[s_ch], ns);
            size_t max_i = s_ch;

            for (size_t i = s_ch; i < SDR_NUM_CHANNELS; i++)
            {
                // Only take into consideration the channels
                // enabled in mask
                if (chain->args.channel_mask & (1UL << i))
                {
                    float rssi = average_power(chan_bufs[i], ns);
                    if (rssi > max_rssi)
                    {
                        max_rssi = rssi;
                        max_i = i;
                    }
                }
            }

            chain->rssi = max_rssi;
            if (chain->rssi > chain->args.squelch_level)
            {
                chain->active_chan = max_i;
                chain->state = proc_tuned;
                if (chain->args.waterfall == 0)
                {
                    LOG(INFO, "Tuned to channel %d (RSSI: %4.2fdB)", chain->active_chan + 1,
                        chain->rssi);
                }
            }
        }
        break;

        case proc_tuned:
        {
            chain->rssi = average_power(chan_bufs[chain->active_chan], ns);
            if (chain->rssi < (chain->args.squelch_level - 5.0))
            {
                if (chain->args.waterfall == 0)
                {
                    LOG(INFO, "Detuned from channel %d", chain->active_chan + 1);
                }
                chain->active_chan = -1;
                chain->state = proc_scanning;
                spgramf_reset(chain->ctcss_spgram);
            }
        }
        break;

        default:
            log_assert(0);
            break;
        }

        for (size_t i = 0; i < SDR_NUM_CHANNELS; i++)
        {
            if (chain->active_chan == i)
            {
                freqdem_demodulate_block(chain->fm_dems[chain->active_chan], chan_bufs[i], ns, fm_out_buf);
                ctcss_execute(chain, fm_out_buf, ns);
                firfilt_rrrf_execute_block(chain->ctcss_filt, fm_out_buf, ns, fm_out_buf);
                for (size_t k = 0; k < ns; k++)
                {
                    fm_out_buf[k] *= chain->args.audio_gain;
                }
                iirfilt_rrrf_execute_block(chain->deemph, fm_out_buf, ns, fm_out_buf);
                if (chain->args.lowpass)
                {
                    firfilt_rrrf_execute_block(chain->audio_filt, fm_out_buf, ns, fm_out_buf);
                }
                pthread_mutex_lock(&lock);
                err = cbufferf_write(chain->audio_buf, fm_out_buf, ns);
                log_assert(err == LIQUID_OK);
                pthread_mutex_unlock(&lock);
            }
        }

        if (chain->args.waterfall > 0)
        {
            asgramcf_write(chain->asgram, resamp_buf, ny);
            asgramcf_execute(chain->asgram, ascii, &maxval, &maxfreq);

            printf(" > %s < pk%5.1fdB [%5.2f] [rssi: %5.1fdB]        \n", ascii, maxval, maxfreq, chain->rssi);
            refresh_footer(chain, footer, chain->args.waterfall);
            printf("%s\r", footer);
            fflush(stdout);
        }
    }

    destroy_rtaudio(chain);
    destroy_soapy(chain);
    destroy_liquid(chain);

    pthread_mutex_destroy(&lock);

    LOG(INFO, "Exiting");
    exit(EXIT_SUCCESS);
}
