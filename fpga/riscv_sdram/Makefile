CC = gcc
CFLAGS = -std=gnu99 -Wall

#ifeq ($(MSYSTEM), MINGW32)
ifdef WINDIR
LIBS = -lftdi1	# Windows MinGW may need this version instead
else
LIBS = -lftdi
endif

# Path to common source
PX = ../../src/

# Local source
USERCODE = usercode.o

HEADERS = $(PX)common.h $(PX)misc.h
COMMONOBJS = common.o devices.o server.o ublast_access_ftdi.o
SERVEROBJS = servermain.o $(COMMONOBJS)
JTAGGEROBJS = riscvmain.o jtagger.o program.o $(USERCODE) $(COMMONOBJS)
ALLOBJS = riscvmain.o servermain.o jtagger.o program.o $(USERCODE) $(COMMONOBJS)

SERVEREXE = jtagserver
JTAGGEREXE = jtagriscv

all: $(ALLOBJS) $(HEADERS)
	$(CC) -o $(JTAGGEREXE) $(JTAGGEROBJS) $(LIBS)
# Comment out the following to omit jtagserver from the build...
#	$(CC) -o $(SERVEREXE) $(SERVEROBJS) $(LIBS)
#	rm -f $(ALLOBJS)	# auto remove clutter

server: $(ALLOBJS) $(HEADERS)
	$(CC) -o $(SERVEREXE) $(SERVEROBJS) $(LIBS)

# Clunky, TODO better ...

jtagger.o: $(HEADERS) $(PX)jtagger.c
	$(CC) -c $(CFLAGS) -o jtagger.o $(PX)jtagger.c

program.o: $(HEADERS) $(PX)program.c
	$(CC) -c $(CFLAGS) -o program.o $(PX)program.c

servermain.o: $(HEADERS) $(PX)servermain.c
	$(CC) -c $(CFLAGS) -o servermain.o $(PX)servermain.c

common.o: $(HEADERS) $(PX)common.c
	$(CC) -c $(CFLAGS) -o common.o $(PX)common.c

devices.o: $(HEADERS) $(PX)devices.c
	$(CC) -c $(CFLAGS) -o devices.o $(PX)devices.c

server.o: $(HEADERS)
	$(CC) -c $(CFLAGS) -o server.o $(PX)server.c

ublast_access_ftdi.o: $(HEADERS) $(PX)ublast_access_ftdi.c
	$(CC) -c $(CFLAGS) -o ublast_access_ftdi.o $(PX)ublast_access_ftdi.c

# NB local source, so no $(PX)
riscvmain.o: $(HEADERS) riscvmain.c
	$(CC) -c $(CFLAGS) -o riscvmain.o riscvmain.c

usercode.o: $(HEADERS) usercode.c
	$(CC) -c $(CFLAGS) -o usercode.o usercode.c

#%.o: %.c $(HEADERS)
#	$(CC) -c $(CFLAGS) -o $@ $(PX)$<

clean:
	rm -f $(JTAGGEREXE) $(SERVEREXE) $(JTAGGEREXE).exe $(SERVEREXE).exe $(ALLOBJS)

cleanobj:
	rm -f $(ALLOBJS)
