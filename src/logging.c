#include "logging.h"

#include <dlg/output.h>

static dlg_handler old_handler;
static void *old_data;

static void custom_handler(const struct dlg_origin *origin, const char *string, void *data)
{
    (void)data;
    dlg_generic_outputf_stream(stdout, "[%h:%m {%t} %f] %s%c\n", origin, string, dlg_default_output_styles, false);
}

void logging_init(void)
{
    old_handler = dlg_get_handler(&old_data);
    dlg_set_handler(custom_handler, NULL);
}
