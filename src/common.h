// common.h

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <alloca.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "misc.h"
#include "ublast_access.h"

// #define SOCKFILE "/tmp/jtag.socket"		// Just used as filename for ftok()
#define SOCKFILE "~/.jtag.socket"			// Just used as filename for ftok() (~ expands to $HOME)
#define PROGRAMFILE_S "system.svf"
#define PROGRAMFILE_R "system.rbf"
#define PROGRAMFILE_X "analyze.out"	// see analyze.c (processes openFPGALoader custom logfile)

// Status bit flags (pack into top 4 bits for access via clientflushrx)
#define JSTA_ERROR 0x10
#define JSTA_OPEN 0x20
#define JSTA_WERROR 0x40
#define JSTA_RERROR 0x80

// Client/Server message protocol. I'm using ASCII strings as it's simple and easy to debug.
// Single letter commands, some of which are a little contrived as I reserved A-F for hex digits.

// NB these are single chars not strings, 0-9, A-F are reserved for hex
// BEWARE these are for documentation, the program code mostly hard-codes the values directly in strings

// Client transmits (command letter, optional hex digits then Z)
#define JMSG_QUIT 'Q'	// Q for quit
#define JMSG_STATUS 'S'	// S for status
#define JMSG_MODE 'M'	// M for mode
#define JMSG_FTDIOPEN 'J'	// J for ftdi open (j since jtag)
#define JMSG_FTDICLOSE 'U'	// U for ftdi close (u like unjtag)
#define JMSG_FTDISTATUS 'H'	// H for hardware status
#define JMSG_READ 'R'	// R for read from ftdi
#define JMSG_WRITE 'W'	// W for write to ftdi
#define JMSG_HEX 'X'	// X introduces a hex string (it is NOT an initial command letter)
#define JMSG_END 'Z'	// Z for end of message

// Server transmits (response letter, recieved command letter, optional hex digits then Z)
#define JMSG_OK 'K'		// K for OK
#define JMSG_ERROR 'N'	// N for no (since E is hex)
#define JMSG_INVALID 'I'	// I for invalid message

#define MSGBUFLEN (16 + 2 * BUF_LEN)	// Allow space for command string plus hex-encoded write buffer

struct jtag_msgbuf {
    long mtype;
    char mtext[MSGBUFLEN];
};

#define DOABORT(s) doabort(__func__, s)

#define TOHEX(n) (((n) & 15) > 9 ? (((n)&15) -10 + 'a') : (((n)&15) + '0'))		// Convert low nibble to hex char

extern int g_isserver;		// Set to 1 for server, 0 for client (allows common init routine)

extern int g_msqtx, g_msqrx;// tx, rx from viewpoint of current program, so client transmits on tx, recieves on rx
							// and server uses same, the actual queues are initialised as complementary depending
							// on the server variable above.

extern int g_server_status;	// Current status see JSTA_* above

extern int g_mode;			// Server control (debugging)

extern int g_strictrx;		// Wait for exact sequence in rx (not used in server mode)

extern int g_response;		// Save response in g_strictrx mode

extern int g_sequence;		// Message sequence ('0'..'9' or null at start since .bss)

extern int g_silent;		// Don't print rx/tx (g_mode | 0x04 for server)

extern int g_standalone;	// Run standalone (no server)
extern int g_server_responds;	// Flag steers standalone response

extern int g_flushrx_timeout;		// logging
extern int g_flushrx_maxdelay;

extern int g_spoofprog;		// DEBUG inhibits respond()

extern int g_debug_log;		// Write responds to file

// NB using global g_clientmsg so caller can access last message
extern struct jtag_msgbuf g_clientmsg;		// See clientflushrx()

// Prototypes
void doabort(const char *func, char *s);
char *hexdump(uint8_t *buf, unsigned int size);
int char2hex(unsigned char c);
void reverse(char *s);
int parse_hex(char *s, uint8_t *buf, int *bufidx);
int initmessage();
int respond(char *s);
int clientflushrx(void);
int io_check(void);
int tap_reset(void);
int runtest5(void);
int runtest(int n);
int scan_dr_int(unsigned int val, int bits);
int program_fpga(char *fname, int filetype);
int serve_alone(char *msg);
int client(void);
int server(void);
int usercode(void);

