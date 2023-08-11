#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pthread.h>
#include <signal.h>

#include <complex.h>
#include <math.h>

#include <argp.h>

#include <SoapySDR/Device.h>

#include <liquid/liquid.h>

#include <rtaudio/rtaudio_c.h>

#include "sdr_pmr446.h"
#include "shared.h"
#include "logging.h"

#define FOOTER_TAIL_LEN (64UL)

#define AUDIO_SAMPLERATE (12500UL)

#define SDR_INPUT_CHUNK (100000UL)
#define SDR_RESAMPLERATE (200000UL)
#define SDR_FREQUENCY (446.1e6)
#define SDR_NUM_CHANNELS (16)
#define SDR_DEFAULT_GAIN (42.0)
#define SDR_DEFAULT_AUDIO_GAIN (4.0)
#define SDR_DEFAULT_SQUELCH_LEVEL (-5.0)

#define SDR_RESAMP_BUF_SIZE (39064)
#define SDR_CHANNEL_BUF_SIZE (2441)

#define HP_AUDIO_FILT_TAPS (377)
#define LP_AUDIO_FILT_TAPS (103)
#define LP_CTCSS_FILT_TAPS (315)
#ifdef APP_FIR_DEEMPH
#define DEEMPH_FILT_TAPS (101)
#endif

#define CTCSS_MID_FREQ_HZ (150.0f)

#define xstr(s) str(s)
#define str(s) #s

typedef complex float ch_buff_mat_t[SDR_NUM_CHANNELS][SDR_CHANNEL_BUF_SIZE];

static error_t parse_opt(int key, char *arg, struct argp_state *state);

// clang-format off
static const float hp_audio_taps[HP_AUDIO_FILT_TAPS] = {
    -0.00610107f,  0.00391676f,  0.00266218f,  0.00167537f,  0.00091408f,  0.00033810f, -0.00008332f, -0.00038047f,
    -0.00057490f, -0.00068889f, -0.00073739f, -0.00073641f, -0.00069558f, -0.00062648f, -0.00053506f, -0.00043000f,
    -0.00031466f, -0.00019579f, -0.00007544f,  0.00004086f,  0.00015223f,  0.00025372f,  0.00034510f,  0.00042207f,
     0.00048485f,  0.00052948f,  0.00055715f,  0.00056456f,  0.00055370f,  0.00052215f,  0.00047310f,  0.00040486f,
     0.00032220f,  0.00022396f,  0.00011684f,  0.00000041f, -0.00011969f, -0.00023528f, -0.00036035f, -0.00045462f,
    -0.00055752f, -0.00063867f, -0.00069138f, -0.00072720f, -0.00074352f, -0.00073043f, -0.00068551f, -0.00061566f,
    -0.00052415f, -0.00041128f, -0.00027567f, -0.00012230f,  0.00004259f,  0.00021043f,  0.00037704f,  0.00053688f,
     0.00068651f,  0.00081869f,  0.00092803f,  0.00100683f,  0.00105219f,  0.00106040f,  0.00103204f,  0.00096555f,
     0.00086322f,  0.00072470f,  0.00055435f,  0.00035501f,  0.00013467f, -0.00010118f, -0.00034187f, -0.00058131f,
    -0.00080836f, -0.00101705f, -0.00119876f, -0.00134443f, -0.00145269f, -0.00150880f, -0.00151771f, -0.00147167f,
    -0.00136820f, -0.00121415f, -0.00101046f, -0.00076114f, -0.00047418f, -0.00016074f,  0.00017216f,  0.00051372f,
     0.00085220f,  0.00117333f,  0.00146710f,  0.00172173f,  0.00192738f,  0.00207220f,  0.00214969f,  0.00215391f,
     0.00208363f,  0.00193645f,  0.00171542f,  0.00142398f,  0.00107118f,  0.00066574f,  0.00022115f, -0.00025085f,
    -0.00073375f, -0.00121240f, -0.00166794f, -0.00208507f, -0.00244541f, -0.00273657f, -0.00294354f, -0.00305704f,
    -0.00306830f, -0.00297137f, -0.00276761f, -0.00245649f, -0.00204759f, -0.00155007f, -0.00097536f, -0.00034322f,
     0.00032939f,  0.00102072f,  0.00170755f,  0.00236430f,  0.00296923f,  0.00349782f,  0.00392881f,  0.00424190f,
     0.00442290f,  0.00445786f,  0.00433928f,  0.00406265f,  0.00363191f,  0.00305345f,  0.00234105f,  0.00151132f,
     0.00058896f, -0.00040026f, -0.00142513f, -0.00245412f, -0.00345042f, -0.00437964f, -0.00520503f, -0.00589429f,
    -0.00641531f, -0.00674295f, -0.00685292f, -0.00673136f, -0.00636727f, -0.00575968f, -0.00491414f, -0.00384363f,
    -0.00257142f, -0.00112603f,  0.00045449f,  0.00212549f,  0.00383932f,  0.00553935f,  0.00717039f,  0.00867240f,
     0.00998642f,  0.01105318f,  0.01182001f,  0.01223528f,  0.01225579f,  0.01184400f,  0.01097392f,  0.00962636f,
     0.00779583f,  0.00548611f,  0.00271515f, -0.00049071f, -0.00409137f, -0.00803824f, -0.01227001f, -0.01672007f,
    -0.02131214f, -0.02596635f, -0.03059634f, -0.03511776f, -0.03944350f, -0.04349080f, -0.04717894f, -0.05043653f,
    -0.05319598f, -0.05540353f, -0.05701288f, -0.05799162f,  0.94167965f, -0.05799162f, -0.05701288f, -0.05540353f,
    -0.05319598f, -0.05043653f, -0.04717894f, -0.04349080f, -0.03944350f, -0.03511776f, -0.03059634f, -0.02596635f,
    -0.02131214f, -0.01672007f, -0.01227001f, -0.00803824f, -0.00409137f, -0.00049071f,  0.00271515f,  0.00548611f,
     0.00779583f,  0.00962636f,  0.01097392f,  0.01184400f,  0.01225579f,  0.01223528f,  0.01182001f,  0.01105318f,
     0.00998642f,  0.00867240f,  0.00717039f,  0.00553935f,  0.00383932f,  0.00212549f,  0.00045449f, -0.00112603f,
    -0.00257142f, -0.00384363f, -0.00491414f, -0.00575968f, -0.00636727f, -0.00673136f, -0.00685292f, -0.00674295f,
    -0.00641531f, -0.00589429f, -0.00520503f, -0.00437964f, -0.00345042f, -0.00245412f, -0.00142513f, -0.00040026f,
     0.00058896f,  0.00151132f,  0.00234105f,  0.00305345f,  0.00363191f,  0.00406265f,  0.00433928f,  0.00445786f,
     0.00442290f,  0.00424190f,  0.00392881f,  0.00349782f,  0.00296923f,  0.00236430f,  0.00170755f,  0.00102072f,
     0.00032939f, -0.00034322f, -0.00097536f, -0.00155007f, -0.00204759f, -0.00245649f, -0.00276761f, -0.00297137f,
    -0.00306830f, -0.00305704f, -0.00294354f, -0.00273657f, -0.00244541f, -0.00208507f, -0.00166794f, -0.00121240f,
    -0.00073375f, -0.00025085f,  0.00022115f,  0.00066574f,  0.00107118f,  0.00142398f,  0.00171542f,  0.00193645f,
     0.00208363f,  0.00215391f,  0.00214969f,  0.00207220f,  0.00192738f,  0.00172173f,  0.00146710f,  0.00117333f,
     0.00085220f,  0.00051372f,  0.00017216f, -0.00016074f, -0.00047418f, -0.00076114f, -0.00101046f, -0.00121415f,
    -0.00136820f, -0.00147167f, -0.00151771f, -0.00150880f, -0.00145269f, -0.00134443f, -0.00119876f, -0.00101705f,
    -0.00080836f, -0.00058131f, -0.00034187f, -0.00010118f,  0.00013467f,  0.00035501f,  0.00055435f,  0.00072470f,
     0.00086322f,  0.00096555f,  0.00103204f,  0.00106040f,  0.00105219f,  0.00100683f,  0.00092803f,  0.00081869f,
     0.00068651f,  0.00053688f,  0.00037704f,  0.00021043f,  0.00004259f, -0.00012230f, -0.00027567f, -0.00041128f,
    -0.00052415f, -0.00061566f, -0.00068551f, -0.00073043f, -0.00074352f, -0.00072720f, -0.00069138f, -0.00063867f,
    -0.00055752f, -0.00045462f, -0.00036035f, -0.00023528f, -0.00011969f,  0.00000041f,  0.00011684f,  0.00022396f,
     0.00032220f,  0.00040486f,  0.00047310f,  0.00052215f,  0.00055370f,  0.00056456f,  0.00055715f,  0.00052948f,
     0.00048485f,  0.00042207f,  0.00034510f,  0.00025372f,  0.00015223f,  0.00004086f, -0.00007544f, -0.00019579f,
    -0.00031466f, -0.00043000f, -0.00053506f, -0.00062648f, -0.00069558f, -0.00073641f, -0.00073739f, -0.00068889f,
    -0.00057490f, -0.00038047f, -0.00008332f,  0.00033810f,  0.00091408f,  0.00167537f,  0.00266218f,  0.00391676f,
    -0.00610107f};

static const float lp_audio_taps[LP_AUDIO_FILT_TAPS] = {
     0.00246253f,  0.00653798f,  0.00120876f, -0.00287389f,  0.00201971f, -0.00020231f, -0.00156764f,  0.00240512f,
    -0.00176244f, -0.00011997f,  0.00217598f, -0.00304036f,  0.00192898f,  0.00069908f, -0.00325112f,  0.00393558f,
    -0.00197824f, -0.00166413f,  0.00470662f, -0.00494003f,  0.00177669f,  0.00310013f, -0.00655946f,  0.00599165f,
    -0.00119629f, -0.00513856f,  0.00886932f, -0.00703653f,  0.00007751f,  0.00798494f, -0.01177575f,  0.00803340f,
     0.00184449f, -0.01203789f,  0.01558157f, -0.00892507f, -0.00509551f,  0.01811789f, -0.02096620f,  0.00967502f,
     0.01086701f, -0.02840958f,  0.02996278f, -0.01023451f, -0.02310694f,  0.05094574f, -0.05120893f,  0.01058206f,
     0.06674462f, -0.15844736f,  0.23238885f,  0.73929005f,  0.23238885f, -0.15844736f,  0.06674462f,  0.01058206f,
    -0.05120893f,  0.05094574f, -0.02310694f, -0.01023451f,  0.02996278f, -0.02840958f,  0.01086701f,  0.00967502f,
    -0.02096620f,  0.01811789f, -0.00509551f, -0.00892507f,  0.01558157f, -0.01203789f,  0.00184449f,  0.00803340f,
    -0.01177575f,  0.00798494f,  0.00007751f, -0.00703653f,  0.00886932f, -0.00513856f, -0.00119629f,  0.00599165f,
    -0.00655946f,  0.00310013f,  0.00177669f, -0.00494003f,  0.00470662f, -0.00166413f, -0.00197824f,  0.00393558f,
    -0.00325112f,  0.00069908f,  0.00192898f, -0.00304036f,  0.00217598f, -0.00011997f, -0.00176244f,  0.00240512f,
    -0.00156764f, -0.00020231f,  0.00201971f, -0.00287389f,  0.00120876f,  0.00653798f,  0.00246253f};

#ifdef APP_FIR_DEEMPH
static const float deemph_taps[DEEMPH_FILT_TAPS] = {
    -0.00051465f, -0.00099186f, -0.00159733f, -0.00227851f, -0.00310069f, -0.00400507f, -0.00506042f, -0.00620048f,
    -0.00749764f, -0.00887676f, -0.01041414f, -0.01202419f, -0.01378760f, -0.01560654f, -0.01756678f, -0.01955631f,
    -0.02166702f, -0.02377038f, -0.02596587f, -0.02810578f, -0.03029920f, -0.03237594f, -0.03445746f, -0.03634694f,
    -0.03818191f, -0.03973361f, -0.04116047f, -0.04219463f, -0.04302200f, -0.04332560f, -0.04332773f, -0.04264781f,
    -0.04155682f, -0.03958890f, -0.03708117f, -0.03344847f, -0.02911980f, -0.02333449f, -0.01665294f, -0.00803967f,
     0.00174917f,  0.01421623f,  0.02829790f,  0.04638760f,  0.06690555f,  0.09427435f,  0.12599297f,  0.17284975f,
     0.23109142f,  0.35660140f,  0.62006398f,  0.35660140f,  0.23109142f,  0.17284975f,  0.12599297f,  0.09427435f,
     0.06690555f,  0.04638760f,  0.02829790f,  0.01421623f,  0.00174917f, -0.00803967f, -0.01665294f, -0.02333449f,
    -0.02911980f, -0.03344847f, -0.03708117f, -0.03958890f, -0.04155682f, -0.04264781f, -0.04332773f, -0.04332560f,
    -0.04302200f, -0.04219463f, -0.04116047f, -0.03973361f, -0.03818191f, -0.03634694f, -0.03445746f, -0.03237594f,
    -0.03029920f, -0.02810578f, -0.02596587f, -0.02377038f, -0.02166702f, -0.01955631f, -0.01756678f, -0.01560654f,
    -0.01378760f, -0.01202419f, -0.01041414f, -0.00887676f, -0.00749764f, -0.00620048f, -0.00506042f, -0.00400507f,
    -0.00310069f, -0.00227851f, -0.00159733f, -0.00099186f, -0.00051465f};
#endif

static const float ctcss_freqs[38] = {
    67.0f, 71.9f, 74.4f, 77.0f, 79.7f, 82.5f, 85.4f, 88.5f, 91.5f, 94.8f, 97.4f, 100.0f, 103.5f, 107.2f,
    110.9f, 114.8f, 118.8f, 123.0f, 127.3f, 131.8f, 136.5f, 141.3f, 146.2f, 151.4f, 156.7f, 162.2f,
    167.9f, 173.8f, 179.9f, 186.2f, 192.8f, 203.5f, 210.7f, 218.1f, 225.7f, 233.6f, 241.8f, 250.3f};
// clang-format on

static proc_chain_t g_chain = {
    .state = proc_scanning,
    .active_chan = -1,
    .ctcss_freq = -1.0,
    .args = {
        .frequency = SDR_FREQUENCY,
        .gain = SDR_DEFAULT_GAIN,
        .audio_gain = SDR_DEFAULT_AUDIO_GAIN,
        .squelch_level = SDR_DEFAULT_SQUELCH_LEVEL,
        .waterfall = 0,
        .lowpass = false,
        .channel_mask = 0xFFFF,
        .lock_mode = lock_mode_start}};

static pthread_mutex_t lock;
static bool exit_via_sig;

static char doc[] =
    "rtl_pmr446 -- a PMR446 band scanner/receiver";

static char args_doc[] = "";

static struct argp_option options[] = {
    {"gain", 'g', "G", 0, "The gain to set in the SDR receiver in [dB] (default: " xstr(SDR_DEFAULT_GAIN) ")"},
    {"squelch", 's', "SQ", 0, "The squelch level in [dB] (default: " xstr(SDR_DEFAULT_SQUELCH_LEVEL) "dB)"},
    {"waterfall", 'w', "WT", 0, "If specified an ASCII waterfall is printed on the screen"},
    {"lowpass", 'l', 0, 0, "Turn on 4.5kHz lowpass audio filter (might reduce noise)"},
    {"mask", 'm', "CM", 0, "Channel mask e.g. 1,2,8-16 (only listen to channels 1,2 and 8 to 16)"},
    {"audio-gain", 'a', "AG", 0, "The gain to set in the SDR receiver in (default: " xstr(SDR_DEFAULT_AUDIO_GAIN) ")"},
    {"lock-mode", 'p', "LM", 0, "Channel lock mode, 'start', or 'max' (default: 'start')"},
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
                break;
            }

            if ((r < 1) || (r > 16))
            {
                LOG(ERROR, "The channels specified in channel mask must be in the range 1-16");
                argp_usage(state);
                break;
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

    case 'p':
        if (strncmp(arg, "max", sizeof("max")) == 0)
        {
            arguments->lock_mode = lock_mode_max;
        }
        else if (strncmp(arg, "start", sizeof("start")) == 0)
        {
            arguments->lock_mode = lock_mode_start;
        }
        else
        {
            LOG(ERROR, "Failed to parse the channel lock mode (should be 'start', or 'max')");
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

static void pll_init(pll_t *pll)
{
    pll->out_filt = iirfilt_rrrf_create((float[]){0.00000158f, 0.00000315f, 0.00000158f}, 3,
                                        (float[]){1.00000000f, -1.99644570f, 0.99645200f}, 3);
    log_assert(pll->out_filt);
    pll->lock_filt = iirfilt_rrrf_create((float[]){0.00000158f, 0.00000315f, 0.00000158f}, 3,
                                         (float[]){1.00000000f, -1.99644570f, 0.99645200f}, 3);
    log_assert(pll->lock_filt);

    pll->agc = agc_rrrf_create();
    log_assert(pll->agc);
    agc_rrrf_set_bandwidth(pll->agc, 0.03);
    agc_rrrf_set_scale(pll->agc, 1.0);

    pll->ref_sig = 0.0;
    pll->integral = 0.0;
    pll->phase = 0;
}

static void pll_destroy(pll_t *pll)
{
    liquid_error_code err;

    err = agc_rrrf_destroy(pll->agc);
    log_assert(err == LIQUID_OK);

    err = iirfilt_rrrf_destroy(pll->lock_filt);
    log_assert(err == LIQUID_OK);

    err = iirfilt_rrrf_destroy(pll->out_filt);
    log_assert(err == LIQUID_OK);
}

static void pll_execute(pll_t *pll, float const *xs, unsigned int nx)
{
    for (unsigned int i = 0; i < nx; i++)
    {
        float pll_loop_control = xs[i] * pll->ref_sig * 8.0;
        iirfilt_rrrf_execute(pll->out_filt, pll_loop_control, &pll->output);

        pll->integral += pll_loop_control / AUDIO_SAMPLERATE;
        pll->ref_sig = sinf(2 * M_PI * 150 * (pll->phase + pll->integral));
        float quad_ref = cosf(2 * M_PI * 150 * (pll->phase + pll->integral));
        float lock;
        iirfilt_rrrf_execute(pll->lock_filt, -quad_ref * xs[i], &lock);
        pll->locked = lock > 0.7f;
        pll->phase += 1.0 / AUDIO_SAMPLERATE;
    }
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

    chain->fm_demod = freqdem_create(0.5f);
    log_assert(chain->fm_demod);

    chain->ctcss_filt = firfilt_rrrf_create((float *)hp_audio_taps, HP_AUDIO_FILT_TAPS);
    log_assert(chain->ctcss_filt);

    chain->ctcss_lp_delay = wdelayf_create((HP_AUDIO_FILT_TAPS - 1) / 2);
    log_assert(chain->ctcss_filt);

    chain->ctcss_dcblock = iirfilt_rrrf_create_dc_blocker(0.0005);
    log_assert(chain->ctcss_dcblock);

    chain->audio_filt = firfilt_rrrf_create((float *)lp_audio_taps, LP_AUDIO_FILT_TAPS);
    log_assert(chain->audio_filt);

#ifdef APP_FIR_DEEMPH
    chain->deemph = firfilt_rrrf_create((float *)deemph_taps, DEEMPH_FILT_TAPS);
#else
    // 50us tau
    chain->deemph = iirfilt_rrrf_create((float[]){0.507301437230636, 0.507301437230636}, 2,
                                        (float[]){1.0, 0.014602874461272194}, 2);
#endif
    log_assert(chain->deemph);

    chain->resamp_buf = cbuffercf_create(resamp_buf_size);
    log_assert(chain->resamp_buf);

    chain->audio_buf = cbufferf_create(AUDIO_SAMPLERATE / 3);
    log_assert(chain->audio_buf);

    chain->ctcss_buf = cbufferf_create(2 * ctcss_buf_size);
    log_assert(chain->ctcss_buf);

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

    err = cbufferf_destroy(chain->ctcss_buf);
    log_assert(err == LIQUID_OK);
    err = cbufferf_destroy(chain->audio_buf);
    log_assert(err == LIQUID_OK);
    err = cbuffercf_destroy(chain->resamp_buf);
    log_assert(err == LIQUID_OK);
#ifdef APP_FIR_DEEMPH
    err = firfilt_rrrf_destroy(chain->deemph);
#else
    err = iirfilt_rrrf_destroy(chain->deemph);
#endif
    log_assert(err == LIQUID_OK);
    err = firfilt_rrrf_destroy(chain->audio_filt);
    log_assert(err == LIQUID_OK);
    err = iirfilt_rrrf_destroy(chain->ctcss_dcblock);
    log_assert(err == LIQUID_OK);
    err = wdelayf_destroy(chain->ctcss_lp_delay);
    log_assert(err == LIQUID_OK);
    err = firfilt_rrrf_destroy(chain->ctcss_filt);
    log_assert(err == LIQUID_OK);
    err = freqdem_destroy(chain->fm_demod);
    log_assert(err == LIQUID_OK);
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
    liquid_error_code err = cbufferf_read(inBuffer, nBufferFrames, &rp, &num_read);
    log_assert(err == LIQUID_OK);
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
    err = cbufferf_release(inBuffer, num_read);
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

void ctcss_execute(proc_chain_t *chain, float *x, unsigned int n)
{
    static int prev_code = -1;
    static uint16_t code_match_count = 0;

    iirfilt_rrrf_execute_block(chain->ctcss_dcblock, x, n, x);
    agc_rrrf_execute_block(chain->pll->agc, x, n, x);
    pll_execute(chain->pll, x, n);

    if (chain->pll->locked)
    {
        float ctcss_freq = CTCSS_MID_FREQ_HZ + (chain->pll->output * CTCSS_MID_FREQ_HZ);
        if (chain->args.waterfall == 0)
        {
            if (1)
            {
                int code = find_ctcss_code(ctcss_freq);
                if (code != prev_code)
                {
                    code_match_count = 0;
                }
                else if (code >= 1)
                {
                    if (code_match_count < 5)
                    {
                        code_match_count++;
                        if (code_match_count == 5)
                        {
                            LOG(INFO, "Acquired CTCSS code: %d (frequency: %3.2fHz)", code, ctcss_freq);
                        }
                    }
                }
                else
                {
                    LOG(DEBUG, "Acquired CTCSS frequency: %3.2fHz (unknown code)", ctcss_freq);
                }
                prev_code = code;
            }
        }
        chain->ctcss_freq = ctcss_freq;
    }
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
        if (chain->pll->locked)
        {
            if (ctcss_code > 0)
            {
                snprintf(&footer[w_len + 6], w_len + FOOTER_TAIL_LEN, "%8.3f MHz [%d]  [CTCSS:  %02d (%3.2fHz)]", SDR_FREQUENCY * 1e-6f,
                         chain->active_chan + 1, ctcss_code, chain->ctcss_freq);
            }
            else
            {
                if (chain->ctcss_freq > 0)
                {
                    snprintf(&footer[w_len + 6], w_len + FOOTER_TAIL_LEN, "%8.3f MHz [%d]  [CTCSS:  ?? (%3.2f)]",
                             SDR_FREQUENCY * 1e-6f,
                             chain->active_chan + 1, chain->ctcss_freq);
                }
                else
                {
                    snprintf(&footer[w_len + 6], w_len + FOOTER_TAIL_LEN, "%8.3f MHz [%d]",
                             SDR_FREQUENCY * 1e-6f,
                             chain->active_chan + 1);
                }
            }
        }
        else
        {
            snprintf(&footer[w_len + 6], w_len + FOOTER_TAIL_LEN, "%8.3f MHz [%d]",
                     SDR_FREQUENCY * 1e-6f,
                     chain->active_chan + 1);
        }
    }
    else
    {
        snprintf(&footer[w_len + 6], w_len + FOOTER_TAIL_LEN, "%8.3f MHz", SDR_FREQUENCY * 1e-6f);
    }
}

static size_t find_max_rssi_channel(proc_chain_t *chain, ch_buff_mat_t *chan_bufs, size_t ns, float *max_rssi)
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

    float rssi_max = average_power((*chan_bufs)[s_ch], ns);
    size_t max_i = s_ch;

    for (size_t i = s_ch; i < SDR_NUM_CHANNELS; i++)
    {
        // Only take into consideration the channels
        // enabled in mask
        if (chain->args.channel_mask & (1UL << i))
        {
            float rssi = average_power((*chan_bufs)[i], ns);
            if (rssi > rssi_max)
            {
                rssi_max = rssi;
                max_i = i;
            }
        }
    }

    *max_rssi = rssi_max;
    return max_i;
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
    size_t chan_size = (size_t)ceilf(res_size / SDR_NUM_CHANNELS);
    LOG(INFO, "SDR_RESAMP_BUF_SIZE=%ld, SDR_CHANNEL_BUF_SIZE=%ld", res_size, chan_size);
    log_assert(res_size == SDR_RESAMP_BUF_SIZE);
    log_assert(chan_size == SDR_CHANNEL_BUF_SIZE);

    complex float buffp[SDR_INPUT_CHUNK];
    complex float resamp_buf[SDR_RESAMP_BUF_SIZE];
    complex float tmp_chan_buf_out[SDR_NUM_CHANNELS];
    void *buffs[] = {buffp};

    ch_buff_mat_t chan_bufs;
    float tmp_buf1[SDR_CHANNEL_BUF_SIZE];
    float tmp_buf2[SDR_CHANNEL_BUF_SIZE];

    // assemble footer
    unsigned int footer_len = chain->args.waterfall + FOOTER_TAIL_LEN;
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

    pll_t pll;

    read = pthread_mutex_init(&lock, NULL);
    log_assert(read == 0);

    ret = init_liquid(chain, chain->args.waterfall, SDR_RESAMP_BUF_SIZE, SDR_CHANNEL_BUF_SIZE);
    log_assert(ret);

    ret = init_soapy(chain);
    if (!ret)
    {
        exit(EXIT_FAILURE);
    }

    ret = init_rtaudio(chain);
    log_assert(ret);

    pll_init(&pll);
    chain->pll = &pll;

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

        log_assert(ns <= SDR_CHANNEL_BUF_SIZE);

        // Update chain state
        switch (chain->state)
        {
        case proc_scanning:
        {
            float max_rssi;
            int max_ch = find_max_rssi_channel(chain, &chan_bufs, ns, &max_rssi);

            chain->rssi = max_rssi;
            if (chain->rssi > chain->args.squelch_level)
            {
                chain->active_chan = max_ch;
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
            if (chain->args.lock_mode == lock_mode_max)
            {
                float max_rssi;
                int max_ch = find_max_rssi_channel(chain, &chan_bufs, ns, &max_rssi);

                chain->rssi = max_rssi;
                if (chain->active_chan != max_ch)
                {
                    if (chain->args.waterfall == 0)
                    {
                        LOG(INFO, "Changed active channel from %d to %d", chain->active_chan + 1, max_ch + 1);
                    }
                    chain->active_chan = max_ch;
                }
            }
            else
            {
                chain->rssi = average_power(chan_bufs[chain->active_chan], ns);
            }

            if (chain->rssi < (chain->args.squelch_level - 5.0))
            {
                if (chain->args.waterfall == 0)
                {
                    LOG(INFO, "Detuned from channel %d", chain->active_chan + 1);
                }
                chain->active_chan = -1;
                chain->state = proc_scanning;
                chain->ctcss_freq = 0.0;
                freqdem_reset(chain->fm_demod);
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
                float tmp;
                liquid_error_code err;

                freqdem_demodulate_block(chain->fm_demod, chan_bufs[i], ns, tmp_buf1);
                firfilt_rrrf_execute_block(chain->ctcss_filt, tmp_buf1, ns, tmp_buf2);

                for (size_t k = 0; k < ns; k++)
                {
                    err = wdelayf_push(chain->ctcss_lp_delay, tmp_buf1[k]);
                    log_assert(err == LIQUID_OK);
                    err = wdelayf_read(chain->ctcss_lp_delay, &tmp);
                    log_assert(err == LIQUID_OK);
                    tmp_buf1[k] = tmp - tmp_buf2[k];
                    tmp_buf2[k] *= chain->args.audio_gain;
                }

                ctcss_execute(chain, tmp_buf1, ns);

#ifdef APP_FIR_DEEMPH
                firfilt_rrrf_execute_block(chain->deemph, tmp_buf2, ns, tmp_buf2);
#else
                iirfilt_rrrf_execute_block(chain->deemph, tmp_buf2, ns, tmp_buf2);
#endif
                if (chain->args.lowpass)
                {
                    firfilt_rrrf_execute_block(chain->audio_filt, tmp_buf2, ns, tmp_buf2);
                }
                pthread_mutex_lock(&lock);
                err = cbufferf_write(chain->audio_buf, tmp_buf2, ns);
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
#ifndef NDEBUG
        if (chain->args.waterfall == 0)
        {
            pthread_mutex_lock(&lock);
            unsigned int s = cbufferf_size(chain->audio_buf);
            pthread_mutex_unlock(&lock);
            if (s > 0)
            {
                LOG(DEBUG, "%d samples in audio buffer (%3.1f%% used)", s, 100 * (float)s / cbufferf_max_size(chain->audio_buf));
            }
        }
#endif
    }

    pll_destroy(chain->pll);
    destroy_rtaudio(chain);
    destroy_soapy(chain);
    destroy_liquid(chain);

    pthread_mutex_destroy(&lock);

    LOG(INFO, "Exiting");
    exit(EXIT_SUCCESS);
}
