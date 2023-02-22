/* server.c - JTAG server using System V message queues (somewhat depreciated cf POSIX MQs)

See https://beej.us/guide/bgipc/html//index.html (easy examples, which is why I'm using System V not POSIX)

NB to list from shell use "ipcs" and "ipcrm" to delete them

Uses ublast_access_ftdi from openocd for communication with Altera USB Blaster

*/

#include "common.h"
#include <ftdi.h>		// For driver_status()

// BEWARE the server process and jtagger process have INDEPENDENT versions of info (since they
// are separate processes). So data must be copied between them at the message level.
// NB info.drv is used ONLY by server (jtagger does not communicate directly with FTDI)

//
/*
 * Global device control
 */
static struct ublast_info info = {
	.ublast_vid = 0x09fb, /* Altera */
	.ublast_pid = 0x6001, /* USB-Blaster */
	.lowlevel_name = NULL,
	.srst_asserted = false,
	.trst_asserted = false,
	.pin6 = FIXED_1,
	.pin8 = FIXED_1,
	// MJ the remaining members initialize to zero since info is in .bss (see objdump -t server, though it
	// is also listed in .data). The above values are possibly stored in .data (see objdump -s -j.data server
	// which shows fb090160 at offset 8040(YMMV))
};

static int initftdi(void)
{
	// Simplified version of openocd ublast_init()
	info.drv = ublast_register_ftdi();

	if (!info.drv)
	{
		LOG_ERROR("Error registering lowlevel driver \"%s\"",
			  info.lowlevel_name);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/*
	 * Register the lowlevel driver
	 */
	info.drv->ublast_vid = info.ublast_vid;
	info.drv->ublast_pid = info.ublast_pid;
	info.drv->ublast_vid_uninit = info.ublast_vid_uninit;
	info.drv->ublast_pid_uninit = info.ublast_pid_uninit;
	info.drv->firmware_path = info.firmware_path;

	info.flags |= info.drv->flags;

	int ret = info.drv->open(info.drv);
	// printf("ret = %d\n", ret);
	return ret;
}

static int driver_status(void)
{
	if (!info.drv)
	{
		printf ("driver_status: info.drv is NULL\n");
		return -1;
	}

	printf ("info.drv->flags %08x\n", info.drv->flags);
	printf ("info.drv->priv %p\n", info.drv->priv);

	struct ftdi_context *ftdic = (struct ftdi_context *)info.drv->priv;
	if (!ftdic)
	{
		printf ("driver_status: ftdic is NULL\n");
		return -1;
	}

	printf ("ftdic->type %08x\n", ftdic->type);
	printf ("ftdic->bitbang_enabled %08x\n", ftdic->bitbang_enabled);
	printf ("ftdic->baudrate %08x\n", ftdic->baudrate);
	printf ("ftdic->max_packet_size %08x\n", ftdic->max_packet_size);
	printf ("ftdic->readbuffer_chunksize %08x\n", ftdic->readbuffer_chunksize);
	printf ("ftdic->writebuffer_chunksize %08x\n", ftdic->writebuffer_chunksize);
	printf ("ftdic->readbuffer %p\n", ftdic->readbuffer);
	printf ("ftdic->readbuffer_offset %08x\n", ftdic->readbuffer_offset);
	printf ("ftdic->readbuffer_remaining %08x\n", ftdic->readbuffer_remaining);

	unsigned int ret =	 ((ftdic->type & 0xff) << 24) |
						((ftdic->bitbang_enabled & 0xff) << 16) |
						(ftdic->readbuffer_remaining & 0x0000ffff);
	return ret;
}

static int flush_write()
{
	uint32_t bytes_written = 0;

	if (!(info.drv && info.drv->open))
		return -1;			// Not open

	if (info.bufidx == 0)	// Nothing to write
		return 0;

	g_wop++; g_wbyte += info.bufidx;

	int ret = info.drv->write(info.drv, info.buf, info.bufidx, &bytes_written);
	if (ret || bytes_written != info.bufidx)
	{
		printf("write: ERROR ret %d or bytes_written %u != info.bufidx %d\n",
#ifdef __CYGWIN__	// It's fussy
					ret, (unsigned int)bytes_written, info.bufidx);
#else
					ret, bytes_written, info.bufidx);
#endif
		g_server_status |= JSTA_WERROR;
		info.bufidx = 0;
		return -1;
	}
	else
	{
		g_server_status &= ~JSTA_WERROR;
		info.bufidx = 0;
		return 0;
	}
	return 0;
}

static int handle_message(char *s)
{
	char resp[256];
	size_t len = strlen(s);
	if (len < 1)
	{
		printf("handle: empty message\n");
		respond("IZ");
		return 1;
	}

	if (s[len-1] != JMSG_END)
	{
		printf("handle: missing JMSG_END\n");
		sprintf(resp, "I%cZ", s[0]);
		respond(resp);
		return 1;
	}

	switch (s[0])
	{

	case JMSG_STATUS :
		{
			unsigned int status = (g_mode & 2) ? JSTA_OPEN : g_server_status;	// Spoof connected state
			status |= info.bufidx << 16;	// So we can get buffer fill state (may be useful for packing reads)
			char *hex = hexdump((uint8_t*)&status, sizeof(status));
			if (hex)
			{
				sprintf(resp, "K%cX%sZ", s[0], hex);
				free (hex);
			}
			else
				sprintf(resp, "N%cZ", s[0]);	// Should not happen
			respond(resp);
			break;
		}

	case JMSG_MODE :
		{
			// Get mode byte from message "RXnnZ"
			if (s[1] != JMSG_HEX)
			{
				respond("NMZ");
				break;
			}

			// Use the info buffer as temporary storage for the size parameter
			if (parse_hex(s, info.buf, &info.bufidx))
			{
				printf("read: parse_hex ERROR\n");
				respond("NMZ");
				break;
			}

			// Just set lowest byte to g_mode (no check of validity)
			g_mode = info.buf[0];
			if (!g_silent)
				printf("setmode 0x%08x\n", g_mode);
			g_silent = g_mode & 0x04;	// TODO make modes #define's

			respond("KMZ");
			break;
		}

	case JMSG_FTDIOPEN :
		{
			// No checks, just attempt whatever client requests (may segfault!)
			info.bufidx = 0;	// In case we close and reopen
			if (initftdi())
			{
				respond("NJZ");
				g_server_status |= JSTA_ERROR;
			}
			else
			{
				respond("KJZ");
				g_server_status |= JSTA_OPEN;
				g_server_status &= ~JSTA_ERROR;
				g_server_status &= ~JSTA_WERROR;
				g_server_status &= ~JSTA_RERROR;
			}
			break;
		}

	case JMSG_FTDICLOSE :
		{
			// BEWARE this will segfault if not already open
			if (info.drv && info.drv->open)
			{
				int ret = info.drv->close(info.drv);
				if (ret)
				{
					respond("NUZ");
					g_server_status |= JSTA_ERROR;
				}
				else
				{
					respond("KUZ");
					g_server_status &= ~JSTA_OPEN;
					g_server_status &= ~JSTA_ERROR;
					g_server_status &= ~JSTA_WERROR;
					g_server_status &= ~JSTA_RERROR;
				}
			}
			else
			{
				respond("NUZ");
				g_server_status |= JSTA_ERROR;
			}
			break;
		}

	case JMSG_FTDISTATUS :
		{
			if (info.drv)
			{
				// Print driver status in server, return an int value to client
				int ret = driver_status();
				char *hex = hexdump((uint8_t*)&ret, sizeof(ret));
				if (hex)
				{
					sprintf(resp, "KHX%sZ", hex);
					free (hex);
				}
				else
					strcpy(resp, "NHZ");	// Should not happen
				respond(resp);
			}
			else
			{
				printf("ftdi_status: info.drv is NULL\n");
				respond("NHZ");
			}
			break;
		}


	case JMSG_WRITE :
		{
			int flushret = 0;	// NB default to success
			uint8_t *p = info.buf;
			if (info.bufidx)
			{
				// Check space (converted hex will always be less than half of string size due to WX..Z)
				if (strlen(s) > (sizeof(info.buf) - info.bufidx) / 2)
					flushret = flush_write();

				p += info.bufidx;
			}

			int len;
			if (parse_hex(s, p, &len))
			{
				printf("write: parse_hex ERROR\n");
				respond("NWZ");
				g_server_status &= ~JSTA_WERROR;
			}

			info.bufidx += len;
			if (flushret)
			{
				respond("NWZ");
				g_server_status |= JSTA_WERROR;
			}
			else
			{
				respond("KWZ");
				g_server_status &= ~JSTA_WERROR;
			}
			break;
		}

	case JMSG_READ :
		// NOTE in usb_blaster.c this is called from ublast_read_byteshifted_tdos() or ublast_read_bitbang_tdos()
		// where the meaning of the data depends on the mode (see notes there). This should be handled by
		// jtagger.c so nothing to do here.
		{
			flush_write();	// TODO handle error (status flags are already set)

			// Get size from message "RXnnnnZ"
			if (s[1] != JMSG_HEX)
			{
				respond("NRZ");
				break;
			}

			// Use the info buffer as temporary storage for the size parameter
			if (parse_hex(s, info.buf, &info.bufidx))
			{
				printf("read: parse_hex ERROR\n");
				respond("NRZ");
				g_server_status &= ~JSTA_RERROR;
				break;
			}
			
			if (info.bufidx < 1 || info.bufidx > 2)	// want size to be one or two bytes
			{
				printf("read: bad size %d\n", info.bufidx);
				respond("NRZ");
				g_server_status &= ~JSTA_RERROR;
				break;
			}

			if (info.bufidx == 1)
				info.bufidx = *((unsigned char*)info.buf);
			else if (info.bufidx == 2)
				info.bufidx = *((unsigned short*)info.buf);
				
			// printf("read: size %d\n", info.bufidx);
			g_rop++; g_rbyte += info.bufidx;

			int ret = -1;
			uint32_t bytes_read = 0;		// return value from info.drv->read()
			if (info.drv && info.drv->open)	// NB checking function pointer info.drv->open exists
			{

#ifdef WANT_CLOCK_GETTIME
			    struct timespec start, finish, delta;	// DEBUG long pauses

			    clock_gettime(CLOCK_MONOTONIC, &start);
#endif
				ret = info.drv->read(info.drv, info.buf, info.bufidx, &bytes_read);
				if (ret)
				{
#ifdef __CYGWIN__
					printf("read: ERROR ret %d bytes_read %u\n", ret, (unsigned int)bytes_read);
#else
					printf("read: ERROR ret %d bytes_read %u\n", ret, bytes_read);
#endif
					respond("NRZ");
					g_server_status |= JSTA_RERROR;
				}
				else
				{
#ifdef WANT_CLOCK_GETTIME
					double report = 0.1L;	// Reporting threshold
					report = 10.0L;			// Effectively disabled
				    clock_gettime(CLOCK_MONOTONIC, &finish);
    				sub_timespec(start, finish, &delta);
    				add_timespec(g_cumulative_read_time, delta, &g_cumulative_read_time);
					if (delta.tv_sec > 0 || delta.tv_nsec > (long)(report * NS_PER_SECOND))
					    printf("FTDI read %d.%.9" PRId64 " seconds cumulative %d.%.9" PRId64 " seconds\n",
								(int)delta.tv_sec, (int64_t)delta.tv_nsec,
								(int)g_cumulative_read_time.tv_sec, (int64_t)g_cumulative_read_time.tv_nsec);
#endif

					char str[MSGBUFLEN];
					char *hex = hexdump(info.buf, bytes_read);
					strcpy (str, "KRX");
					strcat (str, hex);
					strcat (str, "Z");
					respond(str);

					// DEBUG (see note above about the meaning of the data, vis byte vs bit)
					if (!g_silent)
#ifdef __CYGWIN__
						printf("read: %d bytes hex %s\n", (unsigned int)bytes_read, hex);
#else
						printf("read: %d bytes hex %s\n", bytes_read, hex);
#endif

					if (hex)
						free(hex);
					g_server_status &= ~JSTA_RERROR;
				}
			}
			else
			{
				printf("read: info not initialized\n");
				respond("NRZ");
				g_server_status &= ~JSTA_RERROR;
			}

			info.bufidx = 0;	// Since we used it for temporary storage above
			break;
		}

	case JMSG_FLUSH :
		{
			int ret = flush_write();
			if (ret)
				respond("NPZ");
			else
				respond("KPZ");
			break;
		}

	default :
		printf("handle: unknown command %c (TODO?)\n", s[0]);
		sprintf(resp, "I%cZ", s[0]);
	}

	return 0;
}

int server(void)
{
	printf("starting server\n");
	struct jtag_msgbuf msg;
	msg.mtype = 1;

	clientflushrx();	// Discard any pending messages (NB will give bad sequence messages, no harm done)

	for(;;)
	{
		int rxlen = msgrcv(g_msqrx, &msg, sizeof msg.mtext, 0, 0);
		if (rxlen == -1)
		{
			perror("server: msgrcv");
			exit (1);
        }
		if (rxlen > 0)
		{
			int limit = 64;		// Limit for printing, not max message length
			if (rxlen > limit && !g_silent)
			{
				char buf[256];
				if (limit >= sizeof(buf)-2)	// Belt'n'Braces in case I forget to match buf with limit
				{
					printf("rx: limit error\n");
					return 1;
				}
				strncpy(buf, msg.mtext, limit);
				buf[limit] = 0;	// since strncpy does not terminate
			    printf("rx: %s... %d\n", buf, rxlen-1);	// NB rxlen includes terminator, so -1 to avoid confusion
			}
			else
			{
				// Check for unterminated string
				char ch = msg.mtext[rxlen-1];
				if (ch != 0)
				{
					printf("rx: unterminated string\n");
					msg.mtext[rxlen-1] = 0;	// terminate it, and print final char separately
				    if (!g_silent) printf("rx: %s%c\n", msg.mtext, ch);
				}
				else
				    if (!g_silent) printf("rx: %s\n", msg.mtext);
			}

			// Strip sequence prefix (0..9)
			char *str = msg.mtext;
			if (isdigit(str[0]) && str[1])
			{
				g_sequence = *str++;	// Save for use by respond()
				// Handle JMSG_QUIT here, the rest in handle_message()
				if (str[0] == JMSG_QUIT)
				{
					// NB "nQZ" has rxlen == 4 since rxlen includes null terminate
					if (rxlen == 4 && str[1] == JMSG_END)
					{
						printf("recieved JMSG_QUIT, exiting\n");
						respond("KZ");
						return 0;
					}
					else
					{
						printf("invalid message begins JMSG_QUIT\n");
						respond("IHZ");
					}
				}
				else
				{
					if ((g_mode & 1) && str[0] != 'M' && str[0] != 'S')
					{
						// Just acknowledge (used for debug)
						// NB g_mode flag bit 2 also modifies status response, see handle_message()
						char reply[256];
						sprintf(reply, "K%c%sZ", str[0], str[0] == 'R' ? "X" : "");  
						respond(reply);
					}
					else
						handle_message(str);
				}
			}
			else
			{
				printf("rx: malformed message ignored\n");
				// Do NOT respond (allow client to timeout)
			}
		}
		else
	        printf("rx: NULL\n");	// Does not happen since empty string is sent as length 1 due
									// to null string terminator
		
	}
	return 0;
}

int serve_alone(char *msg)	// NOT static, called from jtagger
{
	// Called from respond(), msg does NOT contain squence prefix

	if (!g_standalone)
		doabort(__func__, "not g_standalone");

	if (msg[0] == JMSG_QUIT)
		doabort(__func__, "JMSG_QUIT");

	// if (!g_silent) printf("sr: %s\n", msg);

	// handle_message() will call respond() so set flag to ensure correct message steering
	if (g_server_responds)
		doabort(__func__, "bad g_server_responds");

	g_server_responds = 1;

	if ((g_mode & 1) && msg[0] != 'M' && msg[0] != 'S')
	{
		// Just acknowledge (used for debug)
		// NB g_mode flag bit 2 also modifies status response, see handle_message()
		char reply[256];
		sprintf(reply, "K%c%sZ", msg[0], msg[0] == 'R' ? "X" : "");  
	    if (!g_silent) printf("st: %s\n", msg);
		respond(reply);
	}
	else
		handle_message(msg);

	g_server_responds = 0;

	return g_response;	// returns to jtagger respond() with server response (HACK using global)
}

int client(void)
{
	printf("starting test client\n");
	struct jtag_msgbuf msg;
	msg.mtype = 1;

	clientflushrx();
	char line[256];
	while (fgets(line, sizeof(line), stdin))	// NB Ctrl-D exits
	{
		size_t len = strlen(line);
		if (len < 1)	// Should not happen as fgets() includes \n terminator
		{
			printf("client: zero length string");
			exit (1);
		}

		line[--len] = 0;	// Delete \n

		// Check for escapes (eg pressing up arrow) and discard if present
		for (int i=0; i<len; i++)
		{
			if (line[i] == 27)
			{
				len = 0;	// discard line
				printf("client: input contains ESC char, discarding entire line\n");
				break;
			}
		}

		if (len > 0)		// Do not send empty string ... not that that would cause a problem
		{					// but useful to use this to perform clientflushrx()

			for (int i=0; i<len; i++)
				line[i] = toupper(line[i]);		// Lazy, don't want to type CAPITALS

			len++;			// NB include terminator in length for msgsnd()

			if (len >= sizeof(msg.mtext))
				printf("client: string too long");			// Send it anyway

			strncpy(msg.mtext, line, sizeof(msg.mtext));	// BEWARE max length string is NOT terminated

			// NB client does NOT prefix message with g_sequence char. This is left for the user to do at
			// command input (this allows for testing).

			if (msgsnd(g_msqtx, &msg, len, 0) == -1)
			{
				perror("client: msgsnd");
				exit (1);
			}		
		}

		// Allow time for server to respond (BEWARE this may be inappropriate for FTDI R/W operations)
		JTAGGER_SLEEP(10 * 1000);	// 10 milliseconds
		clientflushrx();
	}
	printf("client: fgets EOF\n");
	return 0;
}


