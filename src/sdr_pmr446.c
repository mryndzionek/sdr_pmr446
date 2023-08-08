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

#define AUDIO_SAMPLERATE (12500UL)

#define SDR_INPUT_CHUNK (100000UL)
#define SDR_RESAMPLERATE (200000UL)
#define SDR_FREQUENCY (446.1e6)
#define SDR_NUM_CHANNELS (16)
#define SDR_DEFAULT_GAIN (42.0)
#define SDR_DEFAULT_AUDIO_GAIN (4.0)
#define SDR_DEFAULT_SQUELCH_LEVEL (-5.0)

#define HP_AUDIO_FILT_TAPS (297)
#define LP_AUDIO_FILT_TAPS (103)
#define LP_CTCSS_FILT_TAPS (315)
#ifdef APP_FIR_DEEMPH
#define DEEMPH_FILT_TAPS (101)
#endif

#define CTCSS_MID_FREQ_HZ (150.0f)

#define xstr(s) str(s)
#define str(s) #s

static error_t parse_opt(int key, char *arg, struct argp_state *state);

// clang-format off
static const float hp_audio_taps[HP_AUDIO_FILT_TAPS] = {
     0.00538494f, -0.00593683f, -0.00334367f, -0.00176582f, -0.00080616f, -0.00022083f,  0.00013836f,  0.00036230f,
     0.00050517f,  0.00060070f,  0.00066745f,  0.00071739f,  0.00075554f,  0.00078620f,  0.00080927f,  0.00082539f,
     0.00083344f,  0.00083325f,  0.00082293f,  0.00080191f,  0.00076911f,  0.00072408f,  0.00066598f,  0.00059516f,
     0.00051150f,  0.00041531f,  0.00030711f,  0.00018858f,  0.00006096f, -0.00007413f, -0.00021457f, -0.00035795f,
    -0.00050187f, -0.00064429f, -0.00078131f, -0.00091174f, -0.00103088f, -0.00113849f, -0.00122731f, -0.00130346f,
    -0.00134903f, -0.00138837f, -0.00138274f, -0.00136808f, -0.00132470f, -0.00124193f, -0.00114589f, -0.00102274f,
    -0.00086788f, -0.00068775f, -0.00049469f, -0.00028093f, -0.00004838f,  0.00019942f,  0.00045010f,  0.00070334f,
     0.00095601f,  0.00120740f,  0.00144816f,  0.00167289f,  0.00187411f,  0.00205116f,  0.00219992f,  0.00231808f,
     0.00239806f,  0.00243718f,  0.00243127f,  0.00238125f,  0.00228508f,  0.00214476f,  0.00195844f,  0.00172768f,
     0.00145244f,  0.00113754f,  0.00078619f,  0.00040442f, -0.00000199f, -0.00042674f, -0.00086377f, -0.00130764f,
    -0.00174794f, -0.00218100f, -0.00259257f, -0.00297876f, -0.00332922f, -0.00363319f, -0.00389111f, -0.00408666f,
    -0.00422011f, -0.00428483f, -0.00427608f, -0.00418645f, -0.00402034f, -0.00377102f, -0.00344036f, -0.00302894f,
    -0.00254360f, -0.00198512f, -0.00136124f, -0.00067891f,  0.00005052f,  0.00082054f,  0.00161886f,  0.00243451f,
     0.00325165f,  0.00405871f,  0.00483951f,  0.00558127f,  0.00626644f,  0.00688142f,  0.00740917f,  0.00783816f,
     0.00815328f,  0.00834287f,  0.00839296f,  0.00829556f,  0.00804124f,  0.00762362f,  0.00703606f,  0.00627617f,
     0.00534188f,  0.00423400f,  0.00295668f,  0.00151518f, -0.00008564f, -0.00183262f, -0.00371890f, -0.00572534f,
    -0.00784113f, -0.01004522f, -0.01232156f, -0.01464732f, -0.01700577f, -0.01936973f, -0.02172146f, -0.02403581f,
    -0.02629047f, -0.02846175f, -0.03053057f, -0.03247380f, -0.03427217f, -0.03590558f, -0.03735903f, -0.03861503f,
    -0.03966227f, -0.04048817f, -0.04108574f, -0.04144585f,  0.95843307f, -0.04144585f, -0.04108574f, -0.04048817f,
    -0.03966227f, -0.03861503f, -0.03735903f, -0.03590558f, -0.03427217f, -0.03247380f, -0.03053057f, -0.02846175f,
    -0.02629047f, -0.02403581f, -0.02172146f, -0.01936973f, -0.01700577f, -0.01464732f, -0.01232156f, -0.01004522f,
    -0.00784113f, -0.00572534f, -0.00371890f, -0.00183262f, -0.00008564f,  0.00151518f,  0.00295668f,  0.00423400f,
     0.00534188f,  0.00627617f,  0.00703606f,  0.00762362f,  0.00804124f,  0.00829556f,  0.00839296f,  0.00834287f,
     0.00815328f,  0.00783816f,  0.00740917f,  0.00688142f,  0.00626644f,  0.00558127f,  0.00483951f,  0.00405871f,
     0.00325165f,  0.00243451f,  0.00161886f,  0.00082054f,  0.00005052f, -0.00067891f, -0.00136124f, -0.00198512f,
    -0.00254360f, -0.00302894f, -0.00344036f, -0.00377102f, -0.00402034f, -0.00418645f, -0.00427608f, -0.00428483f,
    -0.00422011f, -0.00408666f, -0.00389111f, -0.00363319f, -0.00332922f, -0.00297876f, -0.00259257f, -0.00218100f,
    -0.00174794f, -0.00130764f, -0.00086377f, -0.00042674f, -0.00000199f,  0.00040442f,  0.00078619f,  0.00113754f,
     0.00145244f,  0.00172768f,  0.00195844f,  0.00214476f,  0.00228508f,  0.00238125f,  0.00243127f,  0.00243718f,
     0.00239806f,  0.00231808f,  0.00219992f,  0.00205116f,  0.00187411f,  0.00167289f,  0.00144816f,  0.00120740f,
     0.00095601f,  0.00070334f,  0.00045010f,  0.00019942f, -0.00004838f, -0.00028093f, -0.00049469f, -0.00068775f,
    -0.00086788f, -0.00102274f, -0.00114589f, -0.00124193f, -0.00132470f, -0.00136808f, -0.00138274f, -0.00138837f,
    -0.00134903f, -0.00130346f, -0.00122731f, -0.00113849f, -0.00103088f, -0.00091174f, -0.00078131f, -0.00064429f,
    -0.00050187f, -0.00035795f, -0.00021457f, -0.00007413f,  0.00006096f,  0.00018858f,  0.00030711f,  0.00041531f,
     0.00051150f,  0.00059516f,  0.00066598f,  0.00072408f,  0.00076911f,  0.00080191f,  0.00082293f,  0.00083325f,
     0.00083344f,  0.00082539f,  0.00080927f,  0.00078620f,  0.00075554f,  0.00071739f,  0.00066745f,  0.00060070f,
     0.00050517f,  0.00036230f,  0.00013836f, -0.00022083f, -0.00080616f, -0.00176582f, -0.00334367f, -0.00593683f,
     0.00538494f};

static const float lp_ctcss_taps[LP_CTCSS_FILT_TAPS] = {
     0.00038320f, -0.00011305f, -0.00012338f, -0.00014737f, -0.00018267f, -0.00022715f, -0.00027926f, -0.00033722f,
    -0.00039956f, -0.00046445f, -0.00053020f, -0.00059479f, -0.00065635f, -0.00071269f, -0.00076186f, -0.00080168f,
    -0.00083033f, -0.00084585f, -0.00084684f, -0.00083182f, -0.00080005f, -0.00075082f, -0.00068425f, -0.00060057f,
    -0.00050098f, -0.00038672f, -0.00026006f, -0.00012323f,  0.00002049f,  0.00016809f,  0.00031548f,  0.00045926f,
     0.00059472f,  0.00071877f,  0.00082592f,  0.00091411f,  0.00097930f,  0.00101722f,  0.00102722f,  0.00100705f,
     0.00095615f,  0.00087406f,  0.00076194f,  0.00062141f,  0.00045548f,  0.00026762f,  0.00006252f, -0.00015475f,
    -0.00037816f, -0.00060149f, -0.00081796f, -0.00102086f, -0.00120334f, -0.00135902f, -0.00148182f, -0.00156652f,
    -0.00160860f, -0.00160478f, -0.00155282f, -0.00145197f, -0.00130278f, -0.00110744f, -0.00086947f, -0.00059399f,
    -0.00028736f,  0.00004263f,  0.00038735f,  0.00073703f,  0.00108150f,  0.00141031f,  0.00171235f,  0.00197792f,
     0.00219690f,  0.00236041f,  0.00246099f,  0.00249268f,  0.00245122f,  0.00233426f,  0.00214169f,  0.00187568f,
     0.00154069f,  0.00114347f,  0.00069296f,  0.00020012f, -0.00032232f, -0.00086011f, -0.00139787f, -0.00191947f,
    -0.00240850f, -0.00284873f, -0.00322458f, -0.00352166f, -0.00372722f, -0.00383063f, -0.00382374f, -0.00370130f,
    -0.00346122f, -0.00310482f, -0.00263693f, -0.00206588f, -0.00140347f, -0.00066488f,  0.00013189f,  0.00096590f,
     0.00181446f,  0.00265292f,  0.00345569f,  0.00419719f,  0.00485188f,  0.00539560f,  0.00580603f,  0.00606361f,
     0.00615207f,  0.00605907f,  0.00577681f,  0.00530250f,  0.00463867f,  0.00379341f,  0.00278041f,  0.00161893f,
     0.00033355f, -0.00104613f, -0.00248609f, -0.00394840f, -0.00539211f, -0.00677404f, -0.00804980f, -0.00917473f,
    -0.01010511f, -0.01079912f, -0.01121798f, -0.01132697f, -0.01109648f, -0.01050279f, -0.00952902f, -0.00816550f,
    -0.00641057f, -0.00427066f, -0.00176054f,  0.00109668f,  0.00427014f,  0.00772106f,  0.01140390f,  0.01526701f,
     0.01925338f,  0.02330199f,  0.02734877f,  0.03132814f,  0.03517413f,  0.03882184f,  0.04220876f,  0.04527613f,
     0.04797019f,  0.05024329f,  0.05205499f,  0.05337284f,  0.05417318f,  0.05444157f,  0.05417318f,  0.05337284f,
     0.05205499f,  0.05024329f,  0.04797019f,  0.04527613f,  0.04220876f,  0.03882184f,  0.03517413f,  0.03132814f,
     0.02734877f,  0.02330199f,  0.01925338f,  0.01526701f,  0.01140390f,  0.00772106f,  0.00427014f,  0.00109668f,
    -0.00176054f, -0.00427066f, -0.00641057f, -0.00816550f, -0.00952902f, -0.01050279f, -0.01109648f, -0.01132697f,
    -0.01121798f, -0.01079912f, -0.01010511f, -0.00917473f, -0.00804980f, -0.00677404f, -0.00539211f, -0.00394840f,
    -0.00248609f, -0.00104613f,  0.00033355f,  0.00161893f,  0.00278041f,  0.00379341f,  0.00463867f,  0.00530250f,
     0.00577681f,  0.00605907f,  0.00615207f,  0.00606361f,  0.00580603f,  0.00539560f,  0.00485188f,  0.00419719f,
     0.00345569f,  0.00265292f,  0.00181446f,  0.00096590f,  0.00013189f, -0.00066488f, -0.00140347f, -0.00206588f,
    -0.00263693f, -0.00310482f, -0.00346122f, -0.00370130f, -0.00382374f, -0.00383063f, -0.00372722f, -0.00352166f,
    -0.00322458f, -0.00284873f, -0.00240850f, -0.00191947f, -0.00139787f, -0.00086011f, -0.00032232f,  0.00020012f,
     0.00069296f,  0.00114347f,  0.00154069f,  0.00187568f,  0.00214169f,  0.00233426f,  0.00245122f,  0.00249268f,
     0.00246099f,  0.00236041f,  0.00219690f,  0.00197792f,  0.00171235f,  0.00141031f,  0.00108150f,  0.00073703f,
     0.00038735f,  0.00004263f, -0.00028736f, -0.00059399f, -0.00086947f, -0.00110744f, -0.00130278f, -0.00145197f,
    -0.00155282f, -0.00160478f, -0.00160860f, -0.00156652f, -0.00148182f, -0.00135902f, -0.00120334f, -0.00102086f,
    -0.00081796f, -0.00060149f, -0.00037816f, -0.00015475f,  0.00006252f,  0.00026762f,  0.00045548f,  0.00062141f,
     0.00076194f,  0.00087406f,  0.00095615f,  0.00100705f,  0.00102722f,  0.00101722f,  0.00097930f,  0.00091411f,
     0.00082592f,  0.00071877f,  0.00059472f,  0.00045926f,  0.00031548f,  0.00016809f,  0.00002049f, -0.00012323f,
    -0.00026006f, -0.00038672f, -0.00050098f, -0.00060057f, -0.00068425f, -0.00075082f, -0.00080005f, -0.00083182f,
    -0.00084684f, -0.00084585f, -0.00083033f, -0.00080168f, -0.00076186f, -0.00071269f, -0.00065635f, -0.00059479f,
    -0.00053020f, -0.00046445f, -0.00039956f, -0.00033722f, -0.00027926f, -0.00022715f, -0.00018267f, -0.00014737f,
    -0.00012338f, -0.00011305f,  0.00038320f};

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
        .channel_mask = 0xFFFF}};

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
    agc_rrrf_set_bandwidth(pll->agc, 0.005);
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

    chain->ctcss_lp_filt = firfilt_rrrf_create((float *)lp_ctcss_taps, LP_CTCSS_FILT_TAPS);
    log_assert(chain->ctcss_lp_filt);

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
    err = firfilt_rrrf_destroy(chain->ctcss_lp_filt);
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

    firfilt_rrrf_execute_block(chain->ctcss_lp_filt, x, n, x);
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
            sprintf(&footer[w_len + 6], "%8.3f MHz [%d]",
                    SDR_FREQUENCY * 1e-6f,
                    chain->active_chan + 1);
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
    float ctcss_buf[chan_size];

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

    pll_t pll;

    read = pthread_mutex_init(&lock, NULL);
    log_assert(read == 0);

    ret = init_liquid(chain, chain->args.waterfall, res_size, chan_size);
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
                freqdem_demodulate_block(chain->fm_demod, chan_bufs[i], ns, fm_out_buf);
                memcpy(ctcss_buf, fm_out_buf, ns * sizeof(float));
                ctcss_execute(chain, ctcss_buf, ns);
                firfilt_rrrf_execute_block(chain->ctcss_filt, fm_out_buf, ns, fm_out_buf);
                for (size_t k = 0; k < ns; k++)
                {
                    fm_out_buf[k] *= chain->args.audio_gain;
                }
#ifdef APP_FIR_DEEMPH
                firfilt_rrrf_execute_block(chain->deemph, fm_out_buf, ns, fm_out_buf);
#else
                iirfilt_rrrf_execute_block(chain->deemph, fm_out_buf, ns, fm_out_buf);
#endif
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
