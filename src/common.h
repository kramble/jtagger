// common.h

// The Quartus 10.1 version of cygwin is VERY old, so this may not be needed for modern cygwin
// clock_gettime() is only needed for debug, so no great loss if cygwin does not have it
// Also affects older MinGW, so if it's a problem just comment out the #define WANT_CLOCK_GETTIME
#ifndef __CYGWIN__
#define WANT_CLOCK_GETTIME
#endif

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

// **** MUST GO AFTER #include <inttypes.h>
// This is for 32 bit gcc 3.4.4 (cygming special, gdc 0.12, using dmd 0.125)
// supplied with Quartus 10.1 - likely need to remove for 64 bit version
#ifdef __CYGWIN__
#undef PRIuPTR
#define PRIuPTR "u"
#endif

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "misc.h"
#include "ublast_access.h"

#ifndef __MINGW32__
#include <alloca.h>
#include <pwd.h>
#include <byteswap.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#define JTAGGER_SLEEP usleep
// #define JTAGGER_SLEEP(n) printf("\nsleep %d\n", (n)), usleep(n)	// DEBUG
#else
#define IPC_NOWAIT 04000
#define ENOMSG 42
int msgsnd(int msqid, const void *msgp, size_t msgsz, int msgflg);
ssize_t msgrcv(int msqid, void *msgp, size_t msgsz, long msgtyp, int msgflg);
#define JTAGGER_SLEEP windows_sleep
#endif

// #define SOCKFILE "/tmp/jtag.socket"		// Just used as filename for ftok()
#define SOCKFILE "~/.jtag.socket"			// Just used as filename for ftok() (~ expands to $HOME)
#define PROGRAMFILE_S "system.svf"
#define PROGRAMFILE_R "system.rbf"

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
#define JMSG_FLUSH 'P'	// P for purge (since F is hex)
#define JMSG_READ 'R'	// R for read from ftdi
#define JMSG_WRITE 'W'	// W for write to ftdi
#define JMSG_HEX 'X'	// X introduces a hex string (it is NOT an initial command letter)
#define JMSG_END 'Z'	// Z for end of message

// Server transmits (response letter, recieved command letter, optional hex digits then Z)
#define JMSG_OK 'K'		// K for OK
#define JMSG_ERROR 'N'	// N for no (since E is hex)
#define JMSG_INVALID 'I'	// I for invalid message

#define FILETYPE_NONE 0
#define FILETYPE_SVF 1
#define FILETYPE_RBF 2

#define MSGBUFLEN (16 + 2 * BUF_LEN)	// Allow space for command string plus hex-encoded write buffer

struct jtag_msgbuf {
    long mtype;
    char mtext[MSGBUFLEN];
};

#define DOABORT(s) doabort(__func__, s)

#define TOHEX(n) (((n)&15) > 9 ? (((n)&15) - 10 + 'a') : (((n)&15) + '0'))		// Convert low nibble to hex char

// UNHEX() is a bit inefficient due to the duplicated toupper() calls, but it's only used for debugging so no matter
#define UNHEX(c) ((c) < '0' ? 0 : (c) <= '9' ? (c) - '0' : toupper(c) < 'A' ? 0 : toupper(c) > 'F' ? 0 : toupper(c) - 'A' + 10)

extern int g_isserver;		// Set to 1 for server, 0 for client (allows common init routine)

extern int g_msqtx, g_msqrx;// tx, rx from viewpoint of current program, so client transmits on tx, recieves on rx
							// and server uses same, the actual queues are initialised as complementary depending
							// on the server variable above.

extern int g_server_status;	// Current status see JSTA_* above

extern int g_ftdi_ok;		// FTDI open succeeded

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

extern int g_respond_len, g_wop, g_wbyte, g_rop, g_rbyte;	// DEBUG counters

#ifdef WANT_CLOCK_GETTIME
extern struct timespec g_cumulative_read_time;
enum { NS_PER_SECOND = 1000000000 };
#endif

// NB using global g_clientmsg so caller can access last message
extern struct jtag_msgbuf g_clientmsg;		// See clientflushrx()

extern struct timespec g_cumulative_read_time;	// DEBUG

#define DEVICE_PARAMS_CHIP_ID	0
#define DEVICE_PARAMS_IR_LENGTH 1
#define DEVICE_PARAMS_PREAMBLE 2
#define DEVICE_PARAMS_POSTAMBLE 3
#define DEVICE_PARAMS_CHECK_BITS 4
#define DEVICE_PARAMS_STARTUP 5
#define DEVICE_PARAMS_MAXINDEX 5

// TODO add a command line option to read parameters from a configuration file (this is why device_params
// is a pointer to the parameter array static_device_params in devices.c, so it can be overriden).
extern unsigned int (*device_params)[DEVICE_PARAMS_MAXINDEX+1];		// See devices.c

#define NOREADMODE 0		// scan_dr_int() read parameter
#define READMODE 1

#define MILLISECONDS 1000	// for JTAGGER_SLEEP (calls usleep)

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
void jflush(void);
int tap_reset(void);
int runtest5(void);
void IRSHIFT_USER0(void);
void IRSHIFT_USER1(void);
int runtest(int n);
int scan_dr_int(unsigned int val, int bits, int read);
long long unsigned scan_vir_vdr(unsigned int irlen, unsigned int vrlen, unsigned int vir, unsigned int vdr, int read);
int print_nibble(void);
unsigned long long get_bitbang(unsigned int len, unsigned int shift);
void bulk_read(char *mem, unsigned int len, int vir);
void bulk_write(char *mem, unsigned int len, int vir);
int init_fpga(int *device_index);
int program_fpga(char *fname, int filetype, int device_index, int yes);
int serve_alone(char *msg);
int client(void);
int server(void);
int usercode(char *uparams);

#ifdef WANT_CLOCK_GETTIME
void add_timespec(struct timespec t1, struct timespec t2, struct timespec *td);
void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td);
#endif
#ifdef __MINGW32__
int windows_sleep(unsigned int useconds);
#endif
