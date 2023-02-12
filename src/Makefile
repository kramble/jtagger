CC = gcc
CFLAGS = -Wall
LIBS = -lftdi

HEADERS = common.h misc.h
COMMONOBJS = ublast_access_ftdi.o common.o server.o
SERVEROBJS = servermain.o $(COMMONOBJS)
JTAGGEROBJS = jtagger.o program.o $(COMMONOBJS)
ALLOBJS = jtagger.o servermain.o program.o $(COMMONOBJS)

SERVEREXE = jtagserver
JTAGGEREXE = jtagger

all: $(ALLOBJS)
	$(CC) -o $(JTAGGEREXE) $(JTAGGEROBJS) $(LIBS)
# Comment out the following to omit jtagserver from the build...
	$(CC) -o $(SERVEREXE) $(SERVEROBJS) $(LIBS)

%.o: %.c $(HEADERS)
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	rm -f $(JTAGGEREXE) $(SERVEREXE) $(ALLOBJS)

cleanobj:
	rm -f $(ALLOBJS)
