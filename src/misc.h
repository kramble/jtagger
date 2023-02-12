/* misc.h	- miscellaneous defines from various openocd headers, hence...

 SPDX-License-Identifier: GPL-2.0-or-later

 ***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#define ERROR_OK						(0)
#define ERROR_FAIL						(-4)
#define ERROR_WAIT						(-5)

/*
 * The JTAG subsystem defines a number of error codes,
 * using codes between -100 and -199.
 */
#define ERROR_JTAG_INIT_FAILED       (-100)
#define ERROR_JTAG_INVALID_INTERFACE (-101)
#define ERROR_JTAG_NOT_IMPLEMENTED   (-102)
#define ERROR_JTAG_TRST_ASSERTED     (-103)
#define ERROR_JTAG_QUEUE_FAILED      (-104)
#define ERROR_JTAG_NOT_STABLE_STATE  (-105)
#define ERROR_JTAG_DEVICE_ERROR      (-107)
#define ERROR_JTAG_STATE_INVALID     (-108)
#define ERROR_JTAG_TRANSITION_INVALID (-109)
#define ERROR_JTAG_INIT_SOFT_FAIL    (-110)

// The following is specific to jtagger.c (invalid chip ID)
#define ERROR_BADCHIP				(-666)
#define ERROR_IRVALIDATE			(-667)

// Simplified these cf openocd originals ...

// Using stdout here (originally was stderr), and appending printf("\n") using comma operator since
// gcc barfs on a semicolon "warning: macro expands to multiple statements", and gives a subsequent ERROR
// due to broken if/else flow control

#define LOG_INFO(expr ...) \
	fprintf(stdout, expr), fprintf(stdout, "\n")

#define LOG_ERROR(expr ...) \
	fprintf(stdout, expr), fprintf(stdout, "\n")

#define LOG_DEBUG(expr ...) \
	fprintf(stdout, expr), fprintf(stdout, "\n")

#define LOG_DEBUG_IO(expr ...) \
	fprintf(stdout, expr), fprintf(stdout, "\n")

// from jtag.h

/**
 * Defines JTAG Test Access Port states.
 *
 * These definitions were gleaned from the ARM7TDMI-S Technical
 * Reference Manual and validated against several other ARM core
 * technical manuals.
 *
 * FIXME some interfaces require specific numbers be used, as they
 * are handed-off directly to their hardware implementations.
 * Fix those drivers to map as appropriate ... then pick some
 * sane set of numbers here (where 0/uninitialized == INVALID).
 */
typedef enum tap_state {
	TAP_INVALID = -1,

	/* Proper ARM recommended numbers */
	TAP_DREXIT2 = 0x0,
	TAP_DREXIT1 = 0x1,
	TAP_DRSHIFT = 0x2,
	TAP_DRPAUSE = 0x3,
	TAP_IRSELECT = 0x4,
	TAP_DRUPDATE = 0x5,
	TAP_DRCAPTURE = 0x6,
	TAP_DRSELECT = 0x7,
	TAP_IREXIT2 = 0x8,
	TAP_IREXIT1 = 0x9,
	TAP_IRSHIFT = 0xa,
	TAP_IRPAUSE = 0xb,
	TAP_IDLE = 0xc,
	TAP_IRUPDATE = 0xd,
	TAP_IRCAPTURE = 0xe,
	TAP_RESET = 0x0f,
} tap_state_t;

// from commands.h

/**
 * The inferred type of a scan_command_s structure, indicating whether
 * the command has the host scan in from the device, the host scan out
 * to the device, or both.
 */
enum scan_type {
	/** From device to host, */
	SCAN_IN = 1,
	/** From host to device, */
	SCAN_OUT = 2,
	/** Full-duplex scan. */
	SCAN_IO = 3
};

/*
 * Size of data buffer that holds bytes in byte-shift mode.
 * This buffer can hold multiple USB packets aligned to
 * MAX_PACKET_SIZE bytes boundaries.
 * BUF_LEN must be greater than or equal MAX_PACKET_SIZE.
 */
#define BUF_LEN 4096


// ---------------------------------------------
// Structs etc copied from openocd usb_blaster.c
// ---------------------------------------------

enum gpio_steer {
	FIXED_0 = 0,
	FIXED_1,
	SRST,
	TRST,
};

struct ublast_info {
	enum gpio_steer pin6;
	enum gpio_steer pin8;
	int tms;
	int tdi;
	bool trst_asserted;
	bool srst_asserted;
	uint8_t buf[BUF_LEN];
	int bufidx;

	char *lowlevel_name;
	struct ublast_lowlevel *drv;
	uint16_t ublast_vid, ublast_pid;
	uint16_t ublast_vid_uninit, ublast_pid_uninit;
	int flags;
	char *firmware_path;
};

