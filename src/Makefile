CC = gcc
CFLAGS = -std=gnu99 -Wall

#ifeq ($(MSYSTEM), MINGW32)
ifdef WINDIR
LIBS = -lftdi1	# Windows MinGW may need this version instead
else
LIBS = -lftdi
endif

USERCODE = usercode.o

HEADERS = common.h misc.h
COMMONOBJS = common.o devices.o server.o ublast_access_ftdi.o
SERVEROBJS = servermain.o $(COMMONOBJS)
JTAGGEROBJS = jtagger.o program.o $(USERCODE) $(COMMONOBJS)
ALLOBJS = jtagger.o servermain.o program.o $(USERCODE) $(COMMONOBJS)

SERVEREXE = jtagserver
JTAGGEREXE = jtagger

all: $(ALLOBJS)
	$(CC) -o $(JTAGGEREXE) $(JTAGGEROBJS) $(LIBS)
# Comment out the following to omit jtagserver from the build...
#	$(CC) -o $(SERVEREXE) $(SERVEROBJS) $(LIBS)

%.o: %.c $(HEADERS)
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	rm -f $(JTAGGEREXE) $(SERVEREXE) $(JTAGGEREXE).exe $(SERVEREXE).exe $(ALLOBJS)

cleanobj:
	rm -f $(ALLOBJS)
