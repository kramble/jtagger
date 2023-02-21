// common.c

#include "common.h"

int g_isserver;			// Set to 1 for server, 0 for client (allows common init routine)

int g_msqtx, g_msqrx;	// tx, rx from viewpoint of current program, so client transmits on tx, recieves on rx
						// and server uses same, the actual queues are initialised as complementary depending
						// on the server variable above.

int g_server_status;

int g_mode;			// Server control (debugging)

int g_strictrx;		// Wait for exact sequence in rx (not used in server mode)

int g_response;		// Save response in g_strictrx mode

int g_sequence;		// Message sequence ('0'..'9' or null at start since .bss)

int g_silent;		// Don't print rx/tx (g_mode | 0x04 for server)

int g_standalone = 1;	// Run standalone (no server) ... now the default
int g_server_responds;	// Flag steers standalone response

int g_flushrx_timeout;		// logging
int g_flushrx_maxdelay;

int g_spoofprog;		// DEBUG inhibits respond()

int g_debug_log;		// Write responds to file

// NB using global g_clientmsg so caller can access last message
struct jtag_msgbuf g_clientmsg;		// See clientflushrx()

// Prototypes
static int clientflushrx_internal(void);

void doabort(const char *func, char *s)	// named doabort() so as not to clash with library abort()
{
	// Generally used for internal errors to save on typing
	printf("ABORT: %s: %s\n", func, s);
	exit(1);
}

char *hexdump(uint8_t *buf, unsigned int size)	// From usb_blaster.c
{
	unsigned int i;
	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2*i, "%02X", buf[i]);
	return str;	// NB caller must call free(str)
}

int char2hex(unsigned char c)
{
	// NB assumes input is 0-9 A-F a-f, anything else will return garbage
	if (c > 'F')
		c -= ('f' - 'F');	// to uppercase
	if (c > '9')
		return c - 'A' + 10;
	else
		return c - '0';
}

void reverse(char *s)	// reverse a string in place 
{
	char *p;
	int c;
	p = s + strlen(s) - 1;
	while (s < p)
	{
		c = *s;
		*s++ = *p;
		*p-- = c;
	}
}

int parse_hex(char *s, uint8_t *buf, int *bufidx)
{
	// Parse command string of form eg "WX0123456789ABCDEFZ" or "RXnnnnZ"
	if (!s || !buf || !bufidx)
		return -1;

	size_t len = strlen(s);

	int byte = *bufidx = 0;

	// checking minium size, must have odd number of chars, max size and 'X', 'Z' are correct
	if (len < 5 || !(len & 1) || (len-3)/2 > BUF_LEN || s[1] != 'X' || s[len-1] != 'Z')
		return -1;

	for (int i=2; i<len-1; i++)
	{
		if (!isxdigit(s[i]))	// All chars must be hex
			return -1;
		if (i&1)				// at odd indices, convert current and previous char
			buf[byte++] = 16 * char2hex(s[i-1]) + char2hex(s[i]);
	}

	*bufidx = byte;

#if 0	// DEBUG
		int limit = 64;			// Limit output else ublast_initial_wipeout() is ridiculous
		int count = byte;
		if (count > limit)
			count = limit;
		char* hex = hexdump(buf, count);
		printf("parse_hex size %d hex %s%s\n", byte, hex, byte>count ? "..." : "");
		free (hex);
#endif
	return 0;
}

#ifndef __MINGW32__
const char *gethomedir(void)
{
	// https://stackoverflow.com/questions/2910377/get-home-directory-in-linux
	const char *homedir;

	if (!(homedir = getenv("HOME")))
	    homedir = getpwuid(getuid())->pw_dir;
	return (homedir);
}

int initmessage()
{
	// TODO use POSIX messages instead of SYSTEM V (write stubs for msgget() etc)
	const char *func = __func__;

	char *fsock = SOCKFILE;
	if (*fsock == '~')		// Parses ~/path/file as $HOME/path/file
	{
		if (fsock[1] != '/')
			doabort(func, "malformed SOCKFILE");
		const char *homedir = gethomedir();
		if (!homedir)
			doabort(func, "cannot get home directory");
		size_t want = strlen(homedir) + strlen(fsock);	// No need for +1 as skipping 1st char of fsock
		fsock = (char*) alloca(want);
		strcpy (fsock, homedir);
		strcat (fsock, SOCKFILE+1);		
	}

	// Check for SOCKFILE (just used as filename for ftok(), nothing is read/written from it)
	int mode = g_isserver ? O_RDWR : O_RDONLY;
	int fd = open(fsock, mode);
	if (fd < 0)
	{
		if (g_isserver)
		{
			printf("init: could not open %s, attempting to create\n", fsock);
			fd = open(fsock, O_CREAT | mode, 0600);
			if (fd < 0)
			{
				perror("init: open");
				return 1;
			}
		}
		else
		{
			printf("init: could not open %s, you need to start the server\n", fsock);
			perror("init: open");
			return 1;
		}
	}

	// Attempt to read PID
	char pidstr[256];
	pid_t filepid = 0;
	int bytes = read (fd, pidstr, sizeof(pidstr));
	if (bytes == -1)
	{
		perror("init: read");
		close(fd);
		return 1;
	}

	if (bytes > 0)
	{
		if (bytes == sizeof(pidstr))
			bytes--;		// Ensure it fits (will still fail below as that's too long for a PID)
		pidstr[bytes] = 0;	// Terminate string
		if (sscanf(pidstr, "%d", &filepid) != 1 || filepid == 0)
		{
			printf("init: could not read PID\n");
			close(fd);
			return 1;
		}

		// Check if PID is still active
		printf("init: Checking if PID %d is still active\n", filepid);
		int res = kill(filepid, 0);
		if (g_isserver)
		{
			if (res == 0)	// This may only work for same user, if another user it may return EPERM so TODO test that
			{
				printf("init: server is already running as PID %d\n", filepid);
				close(fd);
				return 1;
			}
		}
		else
		{
			if (res)	// This may only work for same user, if another user it may return EPERM so TODO test that
			{
				printf("init: server is NOT running as PID %d, you should start it\n", filepid);
				close(fd);
				return 1;
			}
		}
	}

	printf("init: OK %s readable PID %d\n", fsock, filepid);

	if (g_isserver)
	{
		filepid = getpid();
		sprintf(pidstr, "%d", filepid);
		size_t len = strlen(pidstr)+1;
		lseek(fd, 0, SEEK_SET);
		if (write(fd, pidstr, len) != len)
		{
			perror("init: write");
			close(fd);
			return 1;
		}
	}

	close(fd);

	key_t key;

    if ((key = ftok(fsock, g_isserver ? 'R' : 'T')) == -1)
	{
        perror("init: ftok");
		return 1;
    }

    if ((g_msqrx = msgget(key, 0600 | IPC_CREAT)) == -1)
	{
        perror("init: msgget rx");
		return 1;
    }
    
    if ((key = ftok(fsock, g_isserver ? 'T' : 'R')) == -1)
	{
        perror("init: ftok");
		return 1;
    }

    if ((g_msqtx = msgget(key, 0600 | IPC_CREAT)) == -1)
	{
        perror("init: msgget tx");
		return 1;
    }
    
	printf("init: message queue inititalised OK\n");
	return 0;
}

#else

int msgsnd(int msqid, const void *msgp, size_t msgsz, int msgflg)
{
	return 0;
}

ssize_t msgrcv(int msqid, void *msgp, size_t msgsz, long msgtyp, int msgflg)
{
	return 0;
}

int initmessage()
{
	printf("Client/Server mode is not supported on Windows\n");
	exit(1);
	return 0;
}

#endif // __MINGW32__

int respond(char *s)
{
	// if (!g_silent) printf("rs: %s\n", s);

	struct jtag_msgbuf msg;
	msg.mtype = 1;

	if (g_debug_log && s[0] == 'W')
	{
		static FILE *flog;
		if (!flog)
			flog = fopen("debugout.hex", "wb");
		if (!flog)
		{
			perror("debug log");
			exit (1);
		}
		fprintf(flog,"%s\n",s);
		// Let it close automatically on exit
	}

	if (g_spoofprog)		// DEBUG programming - inhibits sending messages (NB after logging)
		return 0;

	if (g_server_responds)
	{
		*(g_clientmsg.mtext) = '0';	// Dummy sequence prefix
		strcpy(g_clientmsg.mtext+1, s);
		g_response = g_clientmsg.mtext[1] |
			 (g_clientmsg.mtext[2]<<8) |
			 (g_clientmsg.mtext[3]<<16) |
			 (g_clientmsg.mtext[4]<<24);
		return g_response;
	}

	if (g_standalone)
	{
		int response= serve_alone(s);	// NB this returns the response returned above = g_response
		return response;
	}

	if (!g_sequence)	// This should not happen for server since it extracts g_sequence from command message
	{					// but jtagger client WILL encounter this for first command message sent.
		printf("respond: null g_sequence, init to '0'\n");
		g_sequence = '0';
	}

	// Increment sequence (only relevant for jtagger client since server overwrites it on next rx)
	if (!g_isserver)
		if (++g_sequence > '9')
			g_sequence = '0';

	size_t len = strlen(s) + 2;					// NB +2 for since include terminator in length plus and
												//     extra one for g_sequence which we prxfix below

	// printf("respond len = %ld\n", len);		// DEBUG EINVAL (see below)

	if (len > sizeof(msg.mtext))
	{
		printf("respond: string too long");		// Send it anyway
		len = sizeof(msg.mtext);
	}

	msg.mtext[0] = g_sequence;
	strncpy(msg.mtext+1, s, sizeof(msg.mtext)-1);	// BEWARE max length string is NOT terminated

#if 0	// DEBUG
	if (!g_silent)
	{
		char printbuf[64];
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
		snprintf(printbuf, sizeof(printbuf)-1, "tx: %s", msg.mtext);
#pragma GCC diagnostic pop
		puts(printbuf);	// NB adds \n
	}
#endif

	if (msgsnd(g_msqtx, &msg, len, 0) == -1)
	{
		// NB we get EINVAL if len > MSGMAX which is 8192 ... oops it's 8196 for ublast_initial_wipeout()
		printf("respond: ERROR %d len %" PRIuPTR "\n", errno, len);
		perror("respond: msgsnd");
		exit (1);
	}		

	// ALWAYS wait for response in strict mode
	if (g_strictrx)
		g_response = clientflushrx_internal();

	return 0;
}

int clientflushrx(void)
{
	// In strict mode clientflushrx() is done by respond(), with exception for startup (g_sequence == NULL)
	if (g_strictrx && g_sequence)
		return g_response;

	return clientflushrx_internal();
}

static int clientflushrx_internal(void)
{
	if (g_standalone)
		return g_response;

	// NB using global g_clientmsg so caller can access last message
	g_clientmsg.mtype = 1;

	int retval = 0;
	int count = 0;	// messsages received
	int first = 1;	// flags entry

	// sequence prefix delay
	int delay = 5 * 1000;	// 5ms
	int maxiter = 100;		// 500ms total (only when NOT g_strictrx)
	int iter = 0;

	for(;;)
	{
		int rxlen = msgrcv(g_msqrx, &g_clientmsg, sizeof g_clientmsg.mtext, 0, IPC_NOWAIT);
		if (rxlen == -1)
		{
			if (errno == ENOMSG)
			{
				if (first)	// No messages were received, delay and retry (ONCE only unless g_strictrx)
				{
					int firstdelay = 20 * 1000;	// 20 ms (NB less than 10ms is insufficient)
					JTAGGER_SLEEP(firstdelay);
					if (g_strictrx)
					{
						// Wait FOREVER - needed for programming else message queue fills and we stall (need
						// to use ipcs / ipcrm to recover). TODO Could have very long timeout and abort.
						if (!g_silent && !iter)	// !iter ensures we print only once per respond() call
							printf("clientflushrx STRICT no messages, waiting FOREVER\n");
					}
					else
					{
						first = 0;
						printf("clientflushrx no messages, waiting ONCE ONLY\n");
					}

					// This is the for() loop from io_check(), but implemented over the outer loop
					iter++;

					if (iter > g_flushrx_maxdelay)	// log maximum iterations
						g_flushrx_maxdelay = iter;

					continue;
				}
				break;	// Normal exit route (all messages have been processed)
			}
			perror("clientflushrx: msgrcv");
			exit (1);
        }
		first = 0;
		count++;
		if (rxlen > 0)
		{
			// Check for unterminated string
			char ch = g_clientmsg.mtext[rxlen-1];
			if (ch != 0)
			{
				printf("rx: unterminated string\n");
				g_clientmsg.mtext[rxlen-1] = 0;	// terminate it, and print final char separately
		        printf("rx: %s%c\n", g_clientmsg.mtext, ch);
			}
			else
		        if (!g_silent) printf("rx: %s\n", g_clientmsg.mtext);

			// Strip sequence prefix (0..9)
			char prefix = g_clientmsg.mtext[0];
			if (isdigit(prefix) && g_clientmsg.mtext[1])
			{
				if (prefix == g_sequence)
				{
					// Return first four chars (after prefix) of final message packed into int value
					// NB for short messages the upper bits will be garbage (TODO restrict to rxlen)
					retval = g_clientmsg.mtext[1] |
						 (g_clientmsg.mtext[2]<<8) |
						 (g_clientmsg.mtext[3]<<16) |
						 (g_clientmsg.mtext[4]<<24);
					// NB this could be overwritten by a subsequent message, but that's fine as we
					// only want to return the final message
				}
				else
				{
					// This is normal when multiple commands are sent before calling clientflushrx()
					// We just add a delay and timeout to wait for the expected prefix
					// BEWARE a maximum of 9 enqueued messages are supported before prefix wraparound
					// so take care to call clientflushrx() or io_check() before sending more ... TODO
					// automate this in respond()
					if (g_sequence)
					{
						printf("rx: bad sequence got %c expected %c\n", prefix, g_sequence);
						if (g_strictrx)
						{
							printf("STRICT MODE ... ABORT\n");
							exit (1);
						}
					}
					else
					{
						// This is normal if queue was not empty on startup, so g_strictrx ignored
						printf("rx: bad sequence got %c expected NULL\n", prefix);
					}

					// NB g_strictrx only gets here on startup (when g_sequence == NULL)

					retval = 0;

					if (iter >= maxiter)
					{
						printf("clientflushrx TIMEOUT\n");
						g_flushrx_timeout++;
						break;	// returns retval=0 immediately
					}

					// This is the for() loop from io_check(), but implemented over the outer loop
					iter++;

					if (iter > g_flushrx_maxdelay)	// log maximum iterations
						g_flushrx_maxdelay = iter;

					JTAGGER_SLEEP(delay);
				}
			}
			else
			{
				printf("rx: malformed message ignored\n");
				retval = 0;
			}
		}
		else
	        printf("rx: NULL\n");	// Does not happen since empty string is sent as length 1 due
									// to null string terminator
	}
	return retval;
}

int io_check(void)
{
	// Return last message (waits for message), for use after FTDI read calls "RX..Z"
	// BEWARE call clientflushrx() before the "RX..Z" to purge previous messages. May also need
	// a delay in clientflushrx() to ensure all messages have been sent ... DONE, 20mS but perhaps
	// it would be better to track messages (use a sequence number), will be tedious to implement
	// as needs significant changes to protocol (eg an extra char after initial command, say 0..9
	// which could be stripped in the low level message handlers, so maybe not too hard)

	// NOTE delay is now moved to clientflushrx() so can be removed from here ...
	// And we're just left with a call to clientflushrx(), so wrap it in a timeout check.
	
	int ret = clientflushrx();	// NB in g_strictrx mode, this just returns g_response
	if (!ret)
	{
		// These can be made to happen by issuing a read command after an IRSHIFT which does not set the read
		// bit flag, see commented example in vjtag_test(). It does not trigger the second (million) timeout.
		printf("io_check TIMEOUT (was there an unexpected read command?)\n");
		g_flushrx_timeout += 1000;		// Count these in same variable, but units 1000
		JTAGGER_SLEEP(200 * 1000);	// Give it a (lenghty) bit longer
		ret = clientflushrx();
		if (!ret)
		{
			printf("... io_check STILL NO RESULT, give up\n");
			g_flushrx_timeout += 1000000;	// units of one meeeeelion 1,000,000
		}
	}
	return ret;
}

#ifdef __MINGW32__
#include <windows.h>
int windows_sleep(unsigned int useconds)
{
	// While MinGW supplies usleep(), it seems to be buggy and just returns without delay so use
	// the native Windows version (which takes milliseconds, not microseconds as a parameter)
	Sleep(useconds / 1000);
	return 0;
}
#endif
