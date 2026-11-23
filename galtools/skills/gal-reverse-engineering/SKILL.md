---
name: gal-reverse-engineering
description: Reverse engineer GAL and PAL chips (GAL16V8, GAL20V8, GAL22V10 and their PALCE, ATF and PEEL equivalents) as black boxes with a Brutus-28 board and the galtools in the brutus28 repository. galre captures what the chip does, recovers CUPL equations, builds a JED and checks it against every capture. Use this skill whenever the user wants to read, dump, copy, clone, identify or reverse a GAL, PAL or PLD on a Brutus-28, says they put a chip in the socket, mentions galre, galtools, pld capture or a JED rebuilt from a chip, wants galre run on a JED or a recording (sim, replay), or asks why a galre run stopped, differs or looks odd. Also use it before changing galtools itself.
---

# Reverse engineering a GAL with Brutus-28 and galre

galre reads a GAL from the outside. It drives every input combination,
clocks every state, learns registers, latches and enables, writes a CUPL
source, builds the JED and checks the JED against everything it captured.

The tool does most of the work. What goes wrong is around it: the wrong
device type, a chip that was never really reset, a result nobody checked,
a conclusion drawn from the wrong evidence. This skill is the list of
those, with the reasons, so that you can judge cases it does not name.

The reference is in the repository, next to the tool. Read the part you
need instead of guessing:

- `galtools/HOWTO_REVERSE.md`: the procedure, step by step. Section 3
  (sockets and jumpers), section 6 (reading the result) and section 10
  (when galre stops) are the ones you will want most.
- `galtools/README.md`: every option, and "How it works".

Everything here assumes the board is plugged into this computer and that
`galtools/` and `fw/` sit side by side in a checkout of the repository.

## Before you touch the chip

**Ask what is in the socket and what its label says.** You cannot see
the board. Do not drive a chip you have not been told about, and after
the user swaps chips, do not assume which one is in. One read of the
outputs after power-up, with no clock, tells two known chips apart if
you need to make sure.

**An unknown 24-pin part is a GAL22V10 until something shows otherwise.**
A GAL20V8 read as a 22V10 comes out complete, with pins 14 and 23 as
inputs. A 22V10 read as a GAL20V8 does not. Nothing visible from the
outside shows a GAL20V8; only the label does. Whenever the device type
is your assumption, say so in plain words. Do not let it pass as a fact.

**Ask which package and socket before you name jumpers.** The table is
in HOWTO section 3. galre checks the jumpers before it switches the
supply on and prints what it expected if they are wrong, so let that
check run; `--no-jumper-check` is for a detection that is known to be
wrong, not for impatience. Read the PLCC20 warning in that section
before anyone puts the VCC jumper on socket pin 20 with a 5 V supply.

**Use the board's own serial device.** Other USB serial devices may be
connected to the same computer and in use. Pick the port by its name
(under `/dev/serial/by-id/` on Linux, `/dev/cu.usbmodem*` on macOS) and
make sure it is the Brutus-28.
Only one program can talk to the board at a time.

**Firmware and galtools come from the same commit.** Older firmware
rejects newer keys, and newer galtools falls back and says so in its
report. To install firmware without a hand on the board: type
`reset dfu` on its console, then run `dfu-util` as in HOWTO section 2;
on Linux `usbreset 0483:df11` afterwards if the host keeps listing the
boot loader. Flash only when the user has said to.

**galre needs a CUPL compiler and, for some checks, a C compiler.** It
refuses to start a board run without a compiler that builds one gate
for the device, because a result that was not built was not checked.
The final check simulates some designs (latches, a reset term, a walk
of pin 1 in a chip with registers) through the firmware built for this
computer, which takes a C compiler and `fw/` beside `galtools/`.

## Running it

    galtools/galre.py board -p PORT -d GAL22V10 -s plcc28 -t NAME -o OUTDIR

- A run takes from two minutes to more than an hour. Start it in the
  background with its output in a log file, and look at the log; do not
  sit on a foreground command.
- `--fix 6-9=0` holds pins and halves the work for each one. Hold a pin
  only if it is known to be unused or tied on the board the chip came
  from. A first run without `--fix` is slower and assumes nothing.
- Leave pin 1 free: it is the clock.
- `galre.py sim FILE.jed -o OUT` runs the same analysis on a simulated
  chip, and `galre.py replay OUT/capture.jsonl -o OUT2` runs it again
  on a recording, without the board. Use replay after any failure that
  happened after the captures: the recording is complete.

galre takes care of several things by itself and says so in its report.
Read those lines; they tell you what kind of chip you have:

- "A power cycle of 20 ms does not reset this chip": it lengthened the
  power cycles.
- "clock edges change what the chip drives: pin 13 is held low and read
  alone": a GAL16V8 or GAL20V8 with registers, see below.
- "the chip does not answer the same every time": a sweep was repeated
  and decided by majority. Run again and compare.

## When is it done

A run is finished when all of this is true:

1. Exit code 0.
2. The report has `NAME.jed against the captures (...): identical`.
3. The source has no comment "Not seen on the chip", or you have told
   the user what that comment lists.
4. You have said which device type the result is for and whether that
   is known or assumed.

Exit code 1 means the JED or the equations disagree with the captures,
or the check could not run; 2 means nothing was captured (jumpers,
compiler, power cycle); 3 means galre does not model what the chip
does. HOWTO section 10 has every message.

"Identical" compares the JED with what was captured. It says nothing
about cases the walks never reached. For a clocked design every input
combination is clocked in from every state, so that is everything. For
a design with latches it is not: galre lists what it wrote without
having seen it, and anything in that list is a guess that fits.

Where it matters, add a check that does not come from the same run:
walk the chip in orders the recording does not have and compare with
the JED in the simulator, or take the state graph from power-up again
and compare. Two runs from scratch that agree are worth more than one.

## What went wrong before, and why

**A chip that is not reset gives a consistent, wrong result.** A
PALCE20V8 kept its charge through the default 20 ms power cycle and
came up, every time, in the state one clock leads to. Every capture
agreed with every other, the JED matched them all, and the design
lacked four states that are reached from the real power-up state only.
galre now compares a short power cycle with a long one. If a register
is low right after power-up on a GAL16V8 or GAL20V8, either it is
another device or the chip was not reset: try `--off 500` before you
change the device.

**Many outputs switched on at once can clock the chip.** The supply at
the socket is not stiff (the chip's ground reaches board ground through
a small FET). When the pin that enables eight registered outputs fell
while the board held six or more of their pins low, that part took it
for a clock edge, every time. galre saw registers change without a clock and tried to explain
them as latches ("the chip holds state without a clock", then "not
latches alone"). For a GAL16V8 or GAL20V8 with registers galre now
holds the enable pin (11 or 13) low and reads it by itself. For outputs
that a product term enables, as on a GAL22V10, hold the input that
enables them with `--fix`, add that enable to the source by hand, and
check it with a walk of that pin alone. Suspect this whenever a chip
that plainly has registers is reported as changing state without a
clock.

**"Excessive number of product terms" from CUPL is about galre's
equations, not about the chip.** galre's covers are not the shortest.
A PALCE20V8 with four latches came out with sixteen product terms on
one pin and needs six. Do not conclude from this message that the part
is a 22V10. Look at the captures: with few reachable states a much
smaller cover usually exists, and you can write it by hand from the
table of what the outputs do and check it against the recording.

**A quick probe can mislead you about tri-state outputs.** A pin reads
as "not driven" when it follows the level the board pulls it to. If
your probe pulls all I/O pins together, an output that depends on an
I/O pin used as an input (pins 14 and 23 of a GAL20V8, any I/O pin of a
22V10) looks the same. Set every input you are not walking to a known
level, and report a pin as switched off only after galre's own walk
says so.

**A result that was not checked is not a result.** A run once captured
for an hour, wrote equations, said "not compiled" and exited with 0,
and the equations were wrong. Another ended in a traceback at the final
check because `galtools/` had been copied without `fw/`. In both cases
the recording was fine: replay it where the compiler and the firmware
sources are, and report the outcome of that.

**A chip that passes by luck stops passing.** In a design with latches
several equations fit the captures. Which one galre picks can change
with any change to galtools. One design passed until a change for other
designs made galre pick another, and nobody noticed for two days
because that slow design had been left out of the test batch. When you
change galtools, run the unit tests and every design you have, the slow
ones too, and compare each result with the one before.

## Finishing by hand

Sometimes galre gets most of the way: the equations are right but an
enable is missing because a pin was held, or the cover does not fit the
device. Then:

1. Write the `.pld` yourself, starting from galre's. Say in a comment
   what came from the captures and what you added.
2. Compile it with the same CUPL (`cupl -jaxm4 NAME.pld`; `-jaxm0`
   does not minimise, so your terms stay as written).
3. Check the JED against the recording, not against your expectations:
   run the recorded jobs on the simulated chip and compare every
   answer. `galtools/verify.py` and `SimRig` in `galtools/rig.py` are
   what galre itself uses.
4. Check on the chip what the recording does not cover: for a pin that
   was held, walk it alone; for a registered part, clock every input
   combination from every state, starting from power-up.

Keep an earlier result that you replace, under a name that says it is
superseded, and tell the user why it was wrong.

## Telling the user

Lead with the outcome: solved or not, and where the files are. Then,
briefly:

- the device type, and whether it is known from the label or assumed;
- what the chip does, in words, before the equations;
- how it was checked (how many states, input combinations, walks) and
  what was not;
- anything you said earlier that turned out wrong, corrected plainly.

If the run failed, say which exit code and message, what was written
anyway, and the next step you propose.
