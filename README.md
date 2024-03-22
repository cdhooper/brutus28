# Brutus-28 PAL decoder

The goal of this project to build a brute-force decoder for traditional
PAL and GAL 22V10 programmable logic parts. The implementation is not
complete, but at this point, it can decode simple combinatorial gate
devices back to an output which looks similar to what WinCUPL might take
as input.

The rev1 directory contains board design files for the first version of
this board. Note that there is a bug in the design which causes 5V to
always be applied to the inserted PLD's VCC rail, although some protection
can be afforded as the PLD's GND rail is correctly switched by firmware.
This bug will be fixed in the Rev 2 design, once available.

The rev2 directory contains board design files for the second version of
this board. This is the current version of Brutus. Improvements over the
first version include bug fixes, the addition of a PLCC-20 socket, options
for both narrow and wide DIP sockets, and finally optional LEDs which can
show the state of each of the 28 pins for analysis progress.

The fw directory contains STM32 firmware for all board revisions.

The sw directory contains software which runs on your Linux or MacOS
host to do analysis of the output from the STM32 firmware. Note that
with current software, you must manually capture to a file the terminal
output from the firmware. The host software can then perform an analysis
on the captured file and provide decoded operations.

As this is still a work in progress, there is still much debug output
scattered throughout both the software and firmware.

Acquiring the Brutus board files, software, and firmware:
<PRE>
    git clone https://github.com/cdhooper/brutus28
</PRE>

Building Brutus host-side software from Linux:
<PRE>
    cd brutus28/sw
    make
</PRE>

Building Brutus firmware from Linux:
<PRE>
    git clone https://github.com/cdhooper/brutus28
    cd brutus28/fw
    make
</PRE>

Installing Brutus firmware on the Brutus-28 board from Linux (DFU method):
<UL>
<LI> Connect the board to your computer via the USB-C port
<LI> Hold down the Abort button and then press and release the RESET button. Continue holding the ABORT button for three seconds.
<LI> Wait about 10 seconds and a new device should appear on your Linux host. As seen by <B>lsusb</B>:
<PRE>Bus 001 Device 032: ID 0483:df11 STMicroelectronics STM Device in DFU Mode<</PRE>
<LI> In the brutus28/fw directory, type <B>make dfu</B>
<LI> After firmware programming is complete, you may need to press the reset button on your Brutus board. DFU mode exit does not always drop the USB link. You should notice the Green power LED illuminate on your Brutus board.
</UL>


Installing Brutus firmware on the board from Linux (ST-Link method):
<UL>
<LI> Connect the board to an ST-Link device, making sure that all pins are connected to their proper positions. The Brutus ST-Link connector pinout is identical to the STM32F4-Discovery board, so if you have one of those boards and a ribbon cable, it's an easy connection.
<LI> Connect the ST-Link to your PC by USB (on the STM32F4-Discovery board, this is the USB Mini-B port
<LI> In the brutus28/fw directory, type <B>make flash</B>
<LI> After firmware programming is complete, you should notice the Green power LED illuminate on your Brutus board.
</UL>


Capturing and analyzing
<UL>
<LI> Install the IC to analyze in a compatible socket with pin 1 of the chip facing pin 1 of the socket.
<LI> Connect the Brutus board by USB to your computer. It is not necessary to unplug Brutus while changing chips to analyze. Power is automatically removed from the chip when it's not in use.
<LI> There is not yet an easy way to capture from Brutus. Under Linux, I use a program I wrote called term for this. Example:
<PRE>
    echo pld walk dip18 -9 -18 raw | term /dev/ttyACM0 >> chip.cap
</PRE>
<LI> Once you have a capture file, you can then run the software to analyze it.
<PRE>
    brutus chip.cap -d dip18
</PRE>
The output from the brutus utility includes an analysis and logic statements in a format compatible with with WinCUPL language used for programming Lattice GAL22V10 parts.



Eventually further information will be made available here:
    http://eebugs.com/brutus28
