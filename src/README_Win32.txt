Building on Windows using MinGW 32 bit. Tested using an oldish MinGW-32
with gcc 4.8.1 (2013) on 32 bit Windows 10. I'll try building on 64 bit
Windows at a later date (32 bit was exhausting enough!)

Requires libftdi for windows. The official website suggests building from
source but this is not at all straightforward due to a plethora of
dependancies. I spent some considerable time on this then gave up and just
downloaded a pre-built version from...
https://sourceforge.net/projects/picusb/files/

I used libftdi1-1.3git_devkit_mingw32_13Dec2015.zip (there is a more recent
one too, but I did not test that).

Unzip it, and copy the following files to your MinGW installation.
lib/* to MinGW/lib
include/ftdi.h to MinGW/include

Also copy bin/libftdi1.dll to jtagger-main/src

Edit jtagger-main/src/Makefile, uncomment the line "LIBS = -lftdi1", and
remove the previous version "LIBS = -lftdi" (I won't make this the default as
it breaks the rasperrypi build)

Then in an MSYS shell
cd jtagger-main/src
make

You can expect one pragma warning, but otherwise it should be OK.
If you move the executable, copy libftdi1.dll to the same folder, unless
you want to install libftdi1.dll into C:\Windows\system32

The Client/Server option (-s) does not work on Windows, but it was of
little use anyway.

You will also need zdiag to install the windows driver.

for testing. Jtagger source was fairly easy to port, but building libftdi
from source is not at all straightforward. So I cheated and downloaded a
prebuilt version. It works, see README_Win32.txt for details.

Previous comment about cygwin just for info, I have not progressed this...
Also a Windows port looks quite promising. It compiles on cygwin (which I was
surprised to find is included with Quartus 10.1, who knew?) That uses a very
old version of gcc but adding CFLAGS -std=c99 fixes most of the issues).
HOWEVER it compiles only after commenting out the FTDI calls. The SYSTEM V
messaging headers are present but that does not actually work on cygwin (this
is no great loss as it's not needed for standalone). So I just need to get
libftdi working on windows (it seems that it is supported according to some
quick googling). SIGH, I was getting quite used to Linux Mint, bah Windows :-(