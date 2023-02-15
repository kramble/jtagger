/* devices.c	- configuration information for programming RBF files

The RBF programming parameters may be obtained from the Altera/Intel Jrunner source code. Google for
jrunner intel, and look for the "Reference Solutions - Intel" link. You want "JRunner: JTAG Configuration"
and the source archive is wpjrunner.zip. The device details are in source/jb_device.h
This does not seem to have been updated since December 2012, so parameters from newer devices will need to be
obtained from elsewhere.

I have not copied the information verbatim to avoid any risk of copyright violation. The data format here is
my own, but can be derived from the parameters listed in jb_device.h and elsewhere in the Jrunner source.

The parameters are:
chip_id			as reported by the IDCODE instruction
ir_length		instruction register length (always 10 bits for Altera/Intel devices)
data_preamble	bit count of 1's prefixed to the rbf data (MAX_JTAG_INIT_CLOCK)
data_postamble	bit count of 1's appended to the rbf data (jrunner.c uses 128 for all devices)
check_bit		bit count of shifted bits before the single bit test for configuration success, this is derived from
				the Jrunner "Conf_done JTAG Sequence" value, muliplied by 3 (also see boundary scan documentation)
startup			minimum run/test cycle count for startup

Only the chip_id and ir_length are critical. The data_preamble and data_postamble are applied in order to
conform with the Jrunner configuration scheme, but the EP4CE22 seems quite tolerant, YMMV for other devices.
The startup value is a bit weird as Jrunner uses 200 (INIT_COUNT), but the CYCLONE_V handbook has 1222. It turns out
that I had a bug in finish_programming() which required a large value there and svf had a huge runtest(120000).
I've dropped it back to 200 for the DE0-Nano (acutally anything works now, even 0) but YMMV for other devices.

Add any additional devices you wish to support to the array below. If duplicate chip_ids are present, only
the first one is used (see find_device() for the array search algorithm). There is a default configuration in the
very first line that is applied to any unrecognised devices. I don't have much confidence in those values, so
you may need to experiment to find what works.

TODO add a command line option to read parameters from a configuration file (this is why device_params is a pointer to
the parameter array static_device_params in devices.c, so it can be overriden).

*/


#include "common.h"

static unsigned int static_device_params[][DEVICE_PARAMS_MAXINDEX+1] = {

//	chip_id		ir_length	preamble	postamble	check_bit	startup
{			0,	10,			8192,		8192,		0,			12000 },	// Defaults, this MUST be the FIRST entry
																			// NB the default chip_id is not searched so
																			// it will not override an actual chip id of 0
																			// The values here are complete guesswork. YMMV.

{	0x020f30DD,	10,			3192,		128,		447,		200 	},	// EP4CE22 as used in the DE0-Nano

/* It does no harm to leave these test items in place as only the first match is used, but commenting out for the git version

																			// startup tests are without the final
																			// IRSHIFT BYPASS or alternative tap_reset

{	0x020f30DD,	10,			3192,		128,		447,		0 	},		// Try startup 0, this works OK
																			// with IRSHIFT BYPASS / tap_reset

{	0x020f30DD,	10,			3192,		3192,		447,		3168	},	// Test (3168 is insufficient, increasing
																			// postambe does not help)

{	0x020f30DD,	10,			3192,		409,		447,		3176	},	// Test (minimum startup)

{	0x020f30DD,	10,			3192,		128,		447,		200 	},	// EP4CE22 as used in the DE0-Nano [DO NOT EDIT]

END COMMENT */

{	0, 0, 0, 0, 0, 0	}	// This MUST be the last entry. New devices should be added above. ALL six zeros are
							// required to detect the end of the array, bad things will happen if they are missing.
};

int unbaddifier[16] = { 1, 1, 0 };	// Two wrongs don't make a right, and this is misguided in so many ways. Don't rely on it!

unsigned int (*device_params)[DEVICE_PARAMS_MAXINDEX+1] = static_device_params;

