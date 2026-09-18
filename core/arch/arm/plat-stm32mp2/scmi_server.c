#include <assert.h>
#include <compiler.h>
#include <stdint.h>
#include <drivers/scmi-msg.h>
#include <drivers/scmi.h>

const uint8_t *plat_scmi_protocol_list(unsigned int channel_id __unused)
{
	return NULL;
}

struct scmi_msg_channel *plat_scmi_get_channel(unsigned int channel_id)
{
	(void) channel_id;
	return NULL;
}

size_t plat_scmi_protocol_count(void)
{
	return 0;
}

const char *plat_scmi_vendor_name(void)
{
	return NULL;
}

const char *plat_scmi_sub_vendor_name(void)
{
	return NULL;
}
