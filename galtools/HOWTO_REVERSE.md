# How to reverse engineer a GAL with Brutus-28

This guide takes you from a GAL16V8, GAL20V8 or GAL22V10 pulled from a board to CUPL
source and a JED that behaves the same. `galre.py` treats the chip as a
black box: it drives the pins, clocks the chip from state to state and
reads what comes back. It never reads the fuses, so a read-protected chip
works as well as an open one. It recovers:

- combinational logic and output enables;
- registered designs (state machines, counters), for every state reachable
  from power-up;
- the GAL22V10's asynchronous reset term;
- latches built from combinational feedback (set/reset and transparent
  latches, bits that set once), alone or in a registered design, where
  they can also read the clock pin.

[README.md](README.md) explains how each part works and lists every limit.
This guide covers what to do and how to read the result.

> **Status:** first board run on 2026-11-18: an ATF22V10 in the PLCC
> socket, a ROM-board state machine (19 pins enumerated, 6 states), came
> out the same as the analysis of its original JED, equation for equation.
> The asynchronous reset and latch paths have run only on the host, against
> the GAL simulator (the test suite builds `fw/capture.c` for the host).
> Keep the recording (`capture.jsonl`) of every board run: with it, the
> analysis can be redone and problems can be found without the chip.

## 1. What you need

- A Brutus-28 board and a USB-C cable.
- Firmware and galtools from the same commit. The host and the firmware
  share the `pld capture` command set, and new galtools versions use keys
  that older firmware rejects.
- Python 3.8 or newer. numpy is optional; with it, walk data decodes
  faster.
- A CUPL compiler: galre builds the JED with it and checks it against
  every capture. WinCUPL's `cupl` or a native build of it will do. galre
  finds it through `--cupl`, `$CUPL`, or `cupl` on the PATH, and runs it as
  `cupl -jaxm4 NAME.pld`. The `.pld` says so in a comment, and carries
  what else CUPL needs (the device mode, and a `MIN` statement for
  latches). A result that was not built is not checked, so `galre board`
  does not start without a compiler that builds a design for the device.
  With `--no-cupl` it records anyway; replay the recording where the
  compiler is (section 8).
- To build and install the firmware: `arm-none-eabi-gcc`, `make` and
  `dfu-util`.

## 2. Build and install the firmware

    git submodule update --init fw/libopencm3
    cd fw
    make

This builds `fw/objs/fw.bin`. To install it over USB:

1. Hold down the ABORT button, press and release RESET, and keep holding
   ABORT for three seconds.
2. After about 10 seconds the board shows up as USB device `0483:df11`
   (STM Device in DFU Mode).
3. On Linux, run `make dfu` in `fw/`. It first installs udev rules into
   `/etc/udev/rules.d`, with sudo. On macOS (`brew install dfu-util`), or
   to skip the udev rules, run dfu-util yourself:

       dfu-util --device 0483:df11 --alt 0 --download objs/fw.bin --dfuse-address 0x08000000:leave

4. Press RESET if the green power LED does not come on.

The board's serial port is `/dev/ttyACM0` or similar on Linux, and
`/dev/cu.usbmodemXXXX` on macOS. To check the firmware, type
`pld capture ?` on the board's console: the keys listed under `set` must
include `gray rot start skm skv avm avv quiet chk follow dclk ser`.

## 3. Put the chip in

Put the chip at the top of the 28-pin socket, pin 1 to socket pin 1. A
chip with fewer than 28 pins then has its highest pin on socket pin 28.
Set the jumpers:

| Chip | VCC jumper | GND jumper | galre option |
|---|---|---|---|
| GAL16V8 (DIP20) | socket pin 28 | socket pin 10 | `-d GAL16V8` |
| GAL20V8 (DIP24) | socket pin 28 | socket pin 12 | `-d GAL20V8` |
| GAL22V10 (DIP24) | socket pin 28 | socket pin 12 | `-d GAL22V10` |
| GAL20V8 or GAL22V10 (PLCC28, in the PLCC socket) | socket pin 28 | socket pin 14 | `-d GAL20V8 -s plcc28`, `-d GAL22V10 -s plcc28` |
| GAL16V8 (PLCC20, in the small PLCC socket) | socket pin 20 | socket pin 10 | `-d GAL16V8 -s plcc20` |

Set the supply jumper to 5 V.

**A PLCC20 part at 5 V: read this first.** The PLCC20 socket is wired
pin for pin to socket pins 1 to 20, so the part's VCC is socket pin 20,
not 28. On the rev 2 board every socket pin also goes straight to an
STM32 input, and those of socket pins 17 to 22 are PC0 to PC5, which
ST's pin table does not mark as 5 V tolerant (the others are). With the
VCC jumper on pin 20 and the supply jumper on 5 V, PC3 is tied to the
5 V rail through R13 and R14 (22 ohms each, in parallel) only. This
combination has not been run. A DIP part, or a PLCC20 part in a DIP
adapter, has its VCC on socket pin 28 (PC11, 5 V tolerant).

Before the first capture, galre checks the jumpers without switching the
supply on. With only the GND rail switched on, it drives each socket pin
high through its 1K resistor, finds the pins jumpered to VCC and GND, and
only then measures the supply. If the
jumpers do not suit the chip, it stops with exit code 2 and prints what it
expected and what it found. The chip only ever sees 1K-limited drives
until the jumpers are right.

Some parts tie socket pins that are none of their signal pins to the
rails inside. An ATF22V10C in a PLCC28 has VCC on pin 1 as well and GND
on pins 8, 15 and 22, where a GAL22V10 has no connection, and the check
finds them as jumpered. galre takes rails on such pins as they are, if
the VCC and GND pins themselves are found, and says so ("jumper check:
socket pin 1 (VCC) and socket pins 8, 15, 22 (GND): no pin of a
GAL22V10 in the plcc28 socket ..."). A rail on a pin of the device is
still refused.

## 4. Decide which pins to hold

galre walks every input combination of the pins it enumerates, so each
extra pin doubles the work. Some figures from the simulator's operation
count:

| Design | Pins enumerated | Board time |
|---|---|---|
| `designs/cnt16v8` (counter) | 14 | about 12 s |
| `designs/fsm22v10` (state machine) | 17 | about 110 s |
| A 22V10 ROM-board controller from the field | 19 | about 6 min |
| `designs/latch22v10` (three latches), 7 pins held | 11 | about 2 s |

With `--fix`, a pin is held at a fixed level and not enumerated:

    --fix 6-9=0,13=1

Hold a pin only if you know its level on the board the chip came from:
not connected there (hold it at 0), or tied high or low (hold it at that
level). The equations then take the held levels as given, so they match
the chip only with those pins at those levels. galre stops with "pin N is
held at V, but the GAL drives it" if a held pin turns out to be an output.

Leave pin 1 free. In a registered design it is the clock, and galre needs
it to move from state to state.

Without `--fix`, galre enumerates every signal pin. That works too, it
just takes longer. For latch designs, the learning on the host also grows
with the number of input combinations.

## 5. Run galre

    galtools/galre.py board -p /dev/ttyACM0 -d GAL22V10 -o out -t mychip --fix 6-9=0

`-t` names the design and its files (default `galre`). galre prints its
progress and keeps the same text in `out/report.txt`. A run goes like
this:

1. **Jumper check**, as above.
2. **Power-up walk.** galre reads every pin for every input combination.
   Brutus drives every pin weakly, through 1K, so a pin the GAL drives
   reads the GAL's value instead of Brutus's. This finds the
   combinational outputs and the candidates for registers. The power-up
   state is then walked in six more orders, three of them with one pin
   changing at a time. A clocked design reads the same in all seven.
   Every other state is also walked a second time, one pin at a time.
3. **Clocked designs.** galre clocks every input combination into every
   state it finds, walks each new state, and returns to the state under
   test by a clocked path or by power-cycling. If a state falls back to
   the power-up state without a clock, it learns the GAL22V10's
   asynchronous reset term and explores again around it. Every sweep of
   clock edges is run twice, and must read the same (see "the chip does
   not answer the same every time" in section 6).
4. **Latches.** If the power-up walk reads differently in another order,
   the chip holds state without a clock. galre then walks the inputs in
   Gray code order (one pin changes per write) in many bit orders, and
   learns which outputs are latches and what sets and clears them.
5. **Latches in a clocked design.** If a pin changes without a clock in a
   clocked design, and it is not the reset term, galre takes it for a
   latch. It explores the registers again without that pin, then walks
   pin 1 together with the inputs, from every state, and learns the
   latches with pin 1 and the registers as their inputs. It also learns
   register inputs that read a latch, and latches on bidirectional pins.
6. **Outputs that do not settle.** If the same walk reads differently
   twice, a pin may be oscillating: an output that feeds back inverted.
   galre reads every vector many times, takes a pin for its own
   complement where it does not settle, and writes that. The source
   says in a comment which pin it is, and which outputs it took to
   follow it. A copy oscillates at its own frequency.

It writes:

- `out/mychip.pld`: the recovered design as CUPL source;
- `out/report.txt`: setup, pin roles, states, equations, checks and board
  work;
- `out/capture.jsonl`: the setup and every capture the board returned;
- with a CUPL compiler, `out/mychip.jed`, already checked against the
  captures.

## 6. Read the result

galre exits with:

| Code | Meaning |
|---|---|
| 0 | Design written; with a CUPL compiler, the JED matches every capture |
| 1 | The equations or the JED disagree with the captures |
| 2 | The jumper check failed; nothing was captured |
| 3 | The chip changes state without a clock in a way galre does not model; no equations written |

### A registered design

From `galre sim designs/fsm22v10.jed` (shortened):

    power-up walk: combinational [19, 20] (tri-state [20]), state candidates [14, 15, 16, 17, 18, 21, 22, 23] (never driven [14, 15, 16, 17, 18])
    3 states reachable from power-up; inputs [14, 15, 16, 17, 18], registered [21, 22, 23], combinational [19, 20]

    state ZZZZZ010  W=(0, 1, 0)  path []
    state ZZZZZ111  W=(1, 1, 1)  path [2083]
    state ZZZZZ000  W=(0, 0, 0)  path [2083, 2083]

    P19 = !P4 & P5 & P14
        # P4 & !P5;
    !P20 = P4 & !P22;
    P20.oe = P5;
    P21.d = P2 & P22 & !P23
        # !P3 & P23;
    !P22.d = P2 & !P22
        # P3 & P23;
    P22.oe = !P13;
    ...
    fsm.jed against the captures (3 states x 131072 input vectors): identical

- The **power-up walk** line gives the pin roles. Pins that are never
  driven are inputs; pins whose value changes with the inputs are
  combinational outputs; tri-state ones have an output enable.
- Each **state** is listed with its register values `W` and the input
  vectors clocked in to reach it from power-up (`path`; bit b of each
  number drives the b-th enumerated pin, counted from the lowest pin).
- **Equations** use pin names `P<n>`. `.d` is a register input and `.oe`
  an output enable. On a GAL22V10, a negated `.d` (`!P22.d`) is a register
  whose output is active low: it is high at power-up and after a reset,
  and CUPL builds it that way.
- The last line is the check of the rebuilt JED. It must say `identical`.

**"the chip does not answer the same every time: clocked in from state S,
N of M input vectors read differently in 5 sweeps of the same vectors".**
galre runs every sweep of clock edges twice. Here the two differed, so it
ran the sweep five times and took each input vector to lead where most of
them said. The source says so in a comment and the run ends with a NOTE.
The JED is then checked against what most sweeps said, not against a chip
that answers the same every time: run galre again and compare the two
results, and look at the equations of the registers that the named
vectors change. Causes seen so far:

- several pins changing in one write, with firmware that has no `ser=`
  (section 10): a GAL in a PLCC28 loaded its registers, as on a clock
  edge, in one of 20 writes in which the pin that enables its outputs fell
  together with four inputs;
- a bad contact in the socket: reseat the chip.

`--once` runs every sweep once, as galre did before; a wrong reading then
passes for the chip's logic, or for a latch.

### An asynchronous reset term

From `galre sim designs/arlow22v10.jed`, whose reset input on pin 3 is
active low:

    asynchronous reset: the state 1 clocks from power-up (the last with P2=1 P3=1 P5=0) returns to the power-up state when !P3; exploring again around it
    4 states reachable from power-up; inputs [], registered [22, 23], combinational [21]
    the asynchronous reset term fires in 3 of them
    ...
    P22.ar = !P3;
    P23.ar = !P3;

The term is written as `.ar` on every register, since the GAL22V10 has one
reset term shared by all of them.

### Latches

From `galre sim designs/srnor16v8.jed`, a set/reset latch of two
cross-coupled NOR gates (pins 16 and 17), an exclusive OR of one of them
(pin 15), and a bit that sets once (pin 18):

    asynchronous behaviour: pin 18 read differently when the inputs are walked in descending order (...): the chip holds state without a clock, like a latch built from combinational feedback
    trying latches built from combinational feedback
    latch model: inputs [1, 2, 3, 4, 5], outputs [15, 16, 17, 18]
    latch model: latch pins [17, 18], combinational [15, 16], from 12 walks

    P15 = P4 & !P17
        # !P4 & P17;
    P16 = !P2 & !P17;
    P17 = P2 & !P3
        # !P3 & P17;
    P18 = P4 & P5
        # P18;

    srnor.jed against the captures (12 walks x 32 input vectors): identical
      11 walks start differently (galre does not model the latches' state at power-up), the first ...: the rebuilt chip powers up with pin 15 at 0, pin 16 at 1, pin 17 at 0, agrees from read 2

- The first line is not an error: it is how galre notices the latches.
- **Latch pins** are the outputs that hold state. Their equations read
  their own pin (`!P3 & P17` holds pin 17), as the original's do.
- **Combinational** outputs are functions of the inputs and the latch
  pins. Here pin 16, the second NOR gate, reads as `!P2 & !P17`, so one
  latch pin is enough for the pair.
- **"walks start differently"** is a note, not a failure. The state a
  latch takes at power-up is not modelled; in a cross-coupled pair it is a
  race. The rebuilt chip is checked from the first read where it agrees.

The result holds for inputs that change one at a time. Logic that reacts
only when two inputs change at the same moment is not captured.

### Latches in a registered design

From `galre sim designs/reglatch22v10.jed --fix 7-11=0,13-19=0`, two
registers (pins 22 and 23) and a latch (pin 21) that is set while the
clock is high, with an output of the latch on pin 20:

    asynchronous behaviour: reading the state 2 clocks from power-up (the last with ...) changed it: the chip changes state without a clock
    pin 21 changed: taking it for latches built from combinational feedback, or outputs of latches, and exploring the registers again without
    ...
    4 states reachable from power-up; inputs [], registered [22, 23], combinational []
    registers [22, 23]; walking pin 1 with the inputs for pins 20, 21
    latch model: latch pins [21], other outputs [20], from 20 walks with pin 1 enumerated

    P20 = P6 & P21;
    P20.oe = P4;
    P21 = CLK & !P3 & P4
        # CLK & P3 & P22
        # P3 & P22 & P21
        # CLK & P4 & P22;
    P22.d = P23;
    P23.d = P2 & !P22;
    P22.ar = !P5;
    P23.ar = !P5;

    rl22.jed against the captures (4 states x 32 input vectors, 20 walks with pin 1): identical

- The first two lines are not an error: this is how galre notices the
  latch. It may happen once per pin.
- **Latch pins** hold state; **other outputs** are functions of the
  inputs, the registers and the latch pins. Their equations may read
  `CLK`, pin 1.
- If a line says that a pin "is not always driven: enumerating it too,
  and starting again", the latch is on a bidirectional pin, and galre
  runs once more with that pin as an input too. Its `.oe` is the shortest
  product term that fits what was seen.
- "N clock edges went another way than in the sweeps" means that a
  register input reads a latch pin; the `.d` equations then name it.
- "pin N has an enable, and other logic reads it where the GAL lets go"
  is an open-drain style output that is also read as an input. galre
  runs once more with it enumerated.
- The latch's equation may have more product terms than the original.
  The extra ones (here `CLK & P4 & P22`) keep it from glitching when an
  input changes; they do not change what it does. CUPL's minimiser would
  remove them, so the `.pld` has a `MIN` statement for the latch pins;
  compile it with the command in its first comment.
- "exploring again by the registers and the levels of ... together":
  where a clock edge leads depends on a latch. galre explores once more
  with the latch levels as part of each state. This takes the firmware
  from the same commit, and more jobs.
- "N more walks, chosen to show product terms deciding a latch where no
  walk had": galre checked what its equations only guessed, and learned
  from the walks that came out otherwise.
- "not seen, for no walk tried gets there (N): !P15 by ... with P8
  high; ...": the named product term decides the latch with that pin at
  that level, by the equations, but the chip was never seen there. The
  term may lack a literal (`!P8` in the example). If the chip's
  behaviour in such a case matters, look at it by hand.
- "P17 has a product term that is true only in passing: ...": the term
  reads an output together with inputs at levels the output does not
  have once it has followed them, so it is true only for that output's
  gate delay: a pulse, as in an edge detector. The equations reproduce
  it, and the source says so in a comment. Check a copy on the board;
  its delays decide whether the pulse is long enough. A real GAL16V8
  with `designs/edgelatch16v8` in it never took the edge; with the two
  delays of `designs/edge2latch16v8` it does. Settled pins do not show
  how late a copy is, so if other outputs read the same as the one in
  the term, galre writes them as a chain ("P16 is written as following
  P14, not the inputs"), for the longest pulse the original can have
  had. If the copy then misbehaves elsewhere, write that output with
  its inputs again.
- "pins ... taken to move after the other latches of a step": several
  latches change in one step, and with these following the others the
  equations fit the macrocells; they read the others' pins.

## 7. Check the result on a real chip

The check against the captures shows that the JED reproduces everything
galre saw. To check the chip itself, program a blank GAL with
`out/mychip.jed`, run galre on the copy with the same options, and
compare:

    galtools/galre.py board -p /dev/ttyACM0 -d GAL22V10 -o copy -t mychip --fix 6-9=0
    diff out/mychip.pld copy/mychip.pld

The two `.pld` files should be the same. A difference points at behaviour
the copy does not reproduce.

## 8. Redo the analysis without the board

    galtools/galre.py replay out/capture.jsonl -o out2

The device, held pins and title come from the recording. A recording
replays only with a galtools version that asks the board the same
questions; after an update that changes the captures, run on the board
again.

## 9. Practise without hardware

`sim` runs a simulated chip from a JED, then also compares the rebuilt
JED with the original under random stimulus:

    galtools/galre.py sim galtools/designs/fsm22v10.jed -o /tmp/fsm

A design with an asynchronous reset term or latches is simulated through
the firmware built for the host (needs a C compiler), one port write at a
time, with the chip simulated in C too; that takes seconds, also with
every pin enumerated. The test
designs in `designs/`, with the pins the tests hold:

| Design | What it shows | `--fix` |
|---|---|---|
| `cnt16v8` | 2-bit counter | |
| `fsm22v10` | 3-state machine, output enables | |
| `comb16v8` | combinational only | |
| `cnt20v8` | GAL20V8, registered mode: counter, pin 13 as the registers' enable | |
| `lowreg22v10` | a 22V10 that looks like a registered GAL20V8, but for a register that is low after power-up | `4-11=0,14-19=0,23=0` |
| `comb20v8` | GAL20V8, complex mode: an output that reads another output's pin | |
| `simple20v8` | GAL20V8, simple mode | |
| `ar22v10` | asynchronous reset term `P3 & !P4` | `6-11=0,13-20=0` |
| `arlow22v10` | active-low reset input and register outputs | `4=0,6-11=0,13-20=0` |
| `arw22v10` | reset term that reads a register | `4=0,6-11=0,13-20=0` |
| `aropen22v10` | reset term that reads an I/O pin, an open-drain copy of a register that a register input reads back | `5-11=0,13-15=0,17-18=0,20-21=0` |
| `latch16v8` | set/reset and transparent latch | `7-9=0,11-15=0,19=0` |
| `latch22v10` | three latches in a chain, an output of two of them | `7-11=0,13-19=0` |
| `dlat22v10` | two transparent latches with one enable | `5-11=0,13=0,17-23=0` |
| `srnor16v8` | cross-coupled NOR latch, a bit that sets once | `6-9=0,11-14=0,19=0` |
| `reglatch16v8` | a register and a latch | `5-9=0,11=0,12-16=0,19=0` |
| `reglatch22v10` | registers, and a latch that reads the clock pin | `7-11=0,13-19=0` |
| `latchd16v8` | a register input that reads a latch | `5-9=0,11=0,12-16=0,19=0` |
| `trilatch22v10` | a latch on a bidirectional pin, read by an output and a register | `7-11=0,13-20=0` |
| `oeread22v10` | an enable that reads another output's pin | `4-11=0,13-20=0` |
| `quietlatch16v8` | a latch that only shows when one pin changes at a time | `2=0,6-9=0,11-16=0,19=0` |
| `racelatch16v8` | a latch held by another output's pin, with a race | `6-9=0,11-14=0,18-19=0` |
| `chain16v8` | three latches that set each other off in one step, an output with an enable | `8-9=0,11-14=0,19=0` |
| `clklatch22v10` | no registers: a latch that is open while pin 1 is high and looks like a register when clocked | `7-11=0,13-20=0` |
| `oelatch16v8` | registered GAL16V8 with a latch, pin 11 not held: a register with a constant input, an output that shows a register | `6-9=0,12=0,14=0` |
| `routelatch22v10` | a register that loads only with a latch set on the way to the state: explored by registers and latch level together | `5-11=0,13-20=0` |
| `latchfollow16v8` | two latches that set together, and a third that follows them through one term | `12-14=0,19=0` |
| `edgelatch16v8` | an edge detector made from an output's gate delay, an output with an enable | `7-9=0,11-14=0,19=0` |
| `edge2latch16v8` | the same with the delay of two outputs: a real GAL16V8 takes the edge with it, and never did with `edgelatch16v8` | `7-9=0,11-13=0,19=0` |
| `osc16v8` | an output that feeds back inverted and oscillates, and one that follows it | `5-9=0,11-16=0,19=0` |
| `oscstate16v8` | an oscillating output and a latch in one chip: galre stops | `6-9=0,11-16=0,19=0` |
| `buried22v10` | a register that never reaches its pin: galre stops | `5-11=0,13-21=0` |

## 10. When galre stops

**Exit code 2, "no CUPL compiler" or "CUPL does not build a design".**
Nothing was run. galre found no compiler, or the one it found gave no JED
for one gate on this device (the report has what it printed; a missing
device library is the usual reason). Install it or name it with `--cupl`.
`--no-cupl` records without it: the equations are then written, but not
built and not checked, until the recording is replayed with a compiler.

**Exit code 2, jumper check.** Set the jumpers as printed under
"expected". The detection has been checked on one board only; the report
lists its readings for every pin. If you are sure the jumpers are right
and the detection is wrong, `--no-jumper-check` turns the check off, both
in galre and in the firmware.

**A traceback ending in `CaptureError: pld capture set ...: Invalid ...`.**
The firmware is older than galtools. Install the firmware from the same
commit (section 2).

**"pin N is held at V, but the GAL drives it".** Remove that pin from
`--fix`.

**"This firmware changes several pins in one write (it has no ser=)".**
The run goes on. With that firmware, the pins that change in one write
reach the chip in no certain order, and a chip may now and then take that
for a clock edge (see "the chip does not answer the same every time" in
section 6). Install the firmware from the same commit (section 2).

**"two power cycles and the same clocks did not lead back to the state
the job started from ...; running the job again".** A wrong reading, or a
bad contact. The job is run again, twice at most; if that does not help,
galre stops with "could not get back to the path state".

**Exit code 3.** The message before "no equations written" says why:

| Message | Meaning |
|---|---|
| "pin N read differently when the same walk is repeated (...), but the same every time when one input changes at a time" | Where a step changes several inputs, they reach the chip in another order each time, and a latch takes that differently. Not a fault: galre goes on with the latch analyses, which change one input at a time. If no circuit of latches explains the chip, it reads every vector many times, in case an output oscillates at few inputs. |
| "the same walk read differently twice on ..." | A pin reads differently on the same inputs, also when one input changes at a time. galre goes on to see whether it is an output that oscillates ("trying an output that feeds back inverted"); this is the reason only if one of the next two follows. |
| "pin N reads unsteadily at ... input vectors, and settles at ... of them in another walk order" | Not an oscillator, which fails to settle at the same inputs every time: a bad contact in the socket or a floating pin. Reseat the chip. |
| "pin N settles at ..., but high in one walk order and low in another: the chip holds state as well as oscillating" | An oscillating output together with latches or registers, which galre does not model. |
| "... is 1 once and 0 once: not a circuit of latches on those pins" | Not latches alone. galre goes on to try registers and latches together; this is the reason only if that fails too. |
| "the registers do not follow the transitions the sweeps and the walks showed" | The registers go another way than learned, and no latch explains it: state that no pin shows, or readings that do not repeat. |
| "... led to state ..., but the same clocks from power-up lead to ..." | The same: a register that never reaches a pin, or readings that do not repeat (reseat the chip). In a design with latches galre first explores again by the registers and the latch levels together ("exploring again by the registers and the levels of ..."); this is the reason only if it shows again after that. |
| "... leads to a state the sweeps did not find (...): no sequence of clocked vectors leads there" | A state that is reached only with a latch at a level that it has after inputs changed without a clock. galre explores by registers and latch levels, but every state by clocked vectors. |
| "... is 1 once and 0 once: the pin oscillates there ..., or it is not a circuit of registers and latches on those pins" | The same point was seen with two outcomes. galre first tries again with every output pin as a signal of its own ("trying again with every output pin ..."); this is the reason only if that fails too. Then: an output that oscillates, or state that no pin shows. |
| "N steps are not explained by one circuit of latches (the first: pins ... change at ...)", "latch pins ... change together at ..., in no order that agrees with the other walks" or "checking the steps against the learned circuit keeps adding to it" | A step whose outcome depends on more than the levels galre sees, such as a race that the chip's delays decide. |
| "the model of registers and latches did not predict ..." | Latches in a clocked design that behave in a way galre's model does not cover. |
| "latch pin ... is not driven" | A latch in a clocked design with a tri-state output. |
| "walks that change one input at a time do not" | The chip reacts only to several inputs changing at once. |
| "latch pins ... change together ..., in no order that agrees with the other walks" | Latches that galre cannot separate. |
| "outputs ... are not always driven" | A latch design with tri-state outputs. |
| "the latch model did not predict ..." or "the latch model does not predict the walks" | Latches that behave in a way galre's model does not cover. |
| "... returns to the power-up state without a clock, but not for the inputs of one product term" | Something other than the GAL22V10's reset term clears the registers. |
| "walking the inputs in state ... changed it" or "reading the state ... changed it" | If "taking it for latches" follows, galre goes on. Otherwise: a registered design whose state changes without a clock in a way that is neither the reset term nor a latch pin. |

**Exit code 1.** The equations or the JED disagree with some capture; the
report lists the first differences. A marginal contact or slow edges can
cause this. Reseat the chip and try a longer settle time after each write,
for example `--settle 5` (default 1 µs). If it persists, keep
`capture.jsonl`: replaying it shows the same differences without the chip.

**It takes too long.** Hold more pins (section 4).

## What galre cannot see

- States that cannot be reached from power-up. The rebuilt chip matches
  the original only on reachable states.
- Registers that never reach a pin (buried registers).
- A combinational use of pin 1 in a registered GAL22V10 without latches:
  pin 1 stays low except for clock edges. (With latches, galre walks pin 1
  too.) A latch that is open while the clock is high and holds while it
  is low looks like a register.
- A combinational output that depends only on registers looks like a
  register and is written as one. It behaves the same on the pins.
- For latches: inputs changing at the same moment, and the state at
  power-up. In a clocked design also: a state that is reached only with
  a latch at a level that no sequence of clocked vectors leaves it at.

See the Limits section of [README.md](README.md) for the full list.
