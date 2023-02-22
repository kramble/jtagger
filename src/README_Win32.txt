Building on Windows using MinGW 32 bit. Tested using an oldish MinGW-32
with gcc 4.8.1 (2013) on 32 bit Windows 10. Also gcc 8.2.0 on Vista and
gcc 3.4.4 using Quartus 10.1 Cygwin 32 bit.

Requires libftdi for windows. The official website suggests building from
source but this is not at all straightforward due to a plethora of
dependancies. I spent some considerable time on this then gave up and just
downloaded a pre-built version from...
https://sourceforge.net/projects/picusb/files/

I used libftdi1-1.3git_devkit_mingw32_13Dec2015.zip (there is a more recent
one too, but I did not test that).

Unzip it, and copy the following files to your MinGW or Cygwin installation,
lib/libftdi1.a and libftdi1.dll.a to C:/MinGW/lib (or where gcc is installed)
Also include/libftdi1/ftdi.h to C:/MinGW/include

Also copy bin/libftdi1.dll and libusb-1.0.dll to jtagger-main/src as these
must be in the same folder as the jtagger executable in order for it to run.

The following step should no longer be needed as the Makefile now checks for
the WINDIR environment label in order to detect a Windows OS...
Edit jtagger-main/src/Makefile, uncomment the line "LIBS = -lftdi1", and
remove the previous version "LIBS = -lftdi" (I won't make this the default as
it breaks the rasperrypi build)

Then to build, start a MinGW MSYS shell (or Cygwin bash if building there)
cd jtagger-main/src
make

If you move the executable, copy libftdi1.dll and libusb-1.0.dll to the same
folder or alternatively install them into C:\Windows\system32

The Client/Server option (-s) does not work on Windows, but it was of
little use anyway.

You will also need zadig https://zadig.akeo.ie/ to install the WinUSB driver.
BEWARE This will replace the existing Altera driver (which can be reinstalled
from C:\altera\10.1\quartus\drivers\usb-blaster if needed)

A previous comment about cygwin just for info...
It compiles on cygwin (which I was surprised to find is included with Quartus
10.1, who knew?) That uses a very old version 3.4.4 of gcc. The SYSTEM V
messaging headers ARE present (unlike in MinGW) so there is no need to #ifdef
around them, but messaging does not actually work on cygwin (this is no great
loss as it's not needed for standalone).
