# How to reverse engineer a GAL with Brutus-28

This guide takes you from a GAL16V8 or GAL22V10 pulled from a board to CUPL
source and a JED that behaves the same. `galre.py` treats the chip as a
black box: it drives the pins, clocks the chip from state to state and
reads what comes back. It never reads the fuses, so a read-protected chip
works as well as an open one. It recovers:

- combinational logic and output enables;
- registered designs (state machines, counters), for every state reachable
  from power-up;
- the GAL22V10's asynchronous reset term;
- latches built from combinational feedback (set/reset and transparent
  latches, bits that set once).

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
- A CUPL compiler, optional but recommended: galre then builds the JED and
  checks it against every capture. WinCUPL's `cupl` or a native build of it
  will do. galre finds it through `--cupl`, `$CUPL`, or `cupl` on the PATH,
  and runs it as `cupl -jaxm4 NAME.pld`.
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
include `gray rot start skm skv avm avv quiet chk follow dclk`.

## 3. Put the chip in

Put the chip at the top of the 28-pin socket, pin 1 to socket pin 1. A
chip with fewer than 28 pins then has its highest pin on socket pin 28.
Set the jumpers:

| Chip | VCC jumper | GND jumper | galre option |
|---|---|---|---|
| GAL16V8 (DIP20) | socket pin 28 | socket pin 10 | `-d GAL16V8` |
| GAL22V10 (DIP24) | socket pin 28 | socket pin 12 | `-d GAL22V10` |
| GAL22V10 (PLCC28, in the PLCC socket) | socket pin 28 | socket pin 14 | `-d GAL22V10 -s plcc28` |

Set the supply jumper to 5 V.

Before the first capture, galre checks the jumpers without switching the
supply on. With only the GND rail switched on, it drives each socket pin
high through its 1K resistor, finds the pins jumpered to VCC and GND, and
only then measures the supply. If the
jumpers do not suit the chip, it stops with exit code 2 and prints what it
expected and what it found. The chip only ever sees 1K-limited drives
until the jumpers are right.

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
   state is then walked in three more orders. A clocked design reads the
   same in all four.
3. **Clocked designs.** galre clocks every input combination into every
   state it finds, walks each new state, and returns to the state under
   test by a clocked path or by power-cycling. If a state falls back to
   the power-up state without a clock, it learns the GAL22V10's
   asynchronous reset term and explores again around it.
4. **Latches.** If the power-up walk reads differently in another order,
   the chip holds state without a clock. galre then walks the inputs in
   Gray code order (one pin changes per write) in many bit orders, and
   learns which outputs are latches and what sets and clears them.

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
time. That is slow with many pins, so hold the unused ones. The test
designs in `designs/`, with the pins the tests hold:

| Design | What it shows | `--fix` |
|---|---|---|
| `cnt16v8` | 2-bit counter | |
| `fsm22v10` | 3-state machine, output enables | |
| `comb16v8` | combinational only | |
| `ar22v10` | asynchronous reset term `P3 & !P4` | `6-11=0,13-20=0` |
| `arlow22v10` | active-low reset input and register outputs | `4=0,6-11=0,13-20=0` |
| `arw22v10` | reset term that reads a register | `4=0,6-11=0,13-20=0` |
| `latch16v8` | set/reset and transparent latch | `7-9=0,11-15=0,19=0` |
| `latch22v10` | three latches in a chain, an output of two of them | `7-11=0,13-19=0` |
| `dlat22v10` | two transparent latches with one enable | `5-11=0,13=0,17-23=0` |
| `srnor16v8` | cross-coupled NOR latch, a bit that sets once | `6-9=0,11-14=0,19=0` |
| `reglatch16v8` | a register and a latch together: galre stops | `5-9=0,11=0,12-16=0,19=0` |

## 10. When galre stops

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

**Exit code 3.** The message before "no equations written" says why:

| Message | Meaning |
|---|---|
| "the same walk read differently twice on ..." | A pin reads differently on the same inputs: a bad contact in the socket, or an output that oscillates. Reseat the chip. |
| "... is 1 once and 0 once: not a circuit of latches on those pins" | Registers and latches in one design, which galre does not model. |
| "walks that change one input at a time do not" | The chip reacts only to several inputs changing at once. |
| "latch pins ... change together ..., in no order that agrees with the other walks" | Latches that galre cannot separate. |
| "outputs ... are not always driven" | A latch design with tri-state outputs. |
| "the latch model did not predict ..." or "the latch model does not predict the walks" | Latches that behave in a way galre's model does not cover. |
| "... returns to the power-up state without a clock, but not for the inputs of one product term" | Something other than the GAL22V10's reset term clears the registers. |
| "walking the inputs in state ... changed it" or "reading the state ... changed it" | A registered design whose state changes without a clock in another way. |

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
- A combinational use of pin 1 in a registered GAL22V10: pin 1 stays low
  except for clock edges.
- A combinational output that depends only on registers looks like a
  register and is written as one. It behaves the same on the pins.
- For latches: inputs changing at the same moment, and the state at
  power-up.

See the Limits section of [README.md](README.md) for the full list.
