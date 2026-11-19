# galtools: black-box GAL reverse engineering with Brutus-28

`galre.py` recovers the logic of a GAL16V8 or GAL22V10 from what Brutus-28
can observe at the socket: combinational and registered designs, the
GAL22V10's asynchronous reset, and latches built from combinational
feedback. It writes CUPL source and, when a CUPL compiler is available,
builds a JED and checks it against every captured state and input vector.

For a step-by-step guide, see [HOWTO_REVERSE.md](HOWTO_REVERSE.md).

`pld walk` sees only the power-up state, so it cannot recover registered
logic. galre also clocks the chip from state to state and walks each
reachable state. The firmware side is the `pld capture` command
(`fw/capture.c`).

> **Status:** the firmware commands have run only on the host, against the
> GAL simulator (`fwsim.py` compiles `fw/capture.c` for the test
> suite). They have not yet been run on a Brutus-28 board.

## Requirements

- Python 3.8 or newer, standard library only. With numpy installed, walk
  data decodes faster.
- Brutus-28 firmware with `pld capture`.
- Optional: a CUPL compiler, used to turn the result into a JED and check it
  (`--cupl`, `$CUPL`, or `cupl` on the PATH). It is run as
  `cupl -jaxm4 NAME.pld`, so WinCUPL's `cupl` or a native build of it
  will do.

## Use

Put the chip at the top of the 28-pin socket, pin 1 to pin 1, as described
in the main README:

- GAL16V8: VCC on socket pin 28, GND on socket pin 10.
- GAL22V10: VCC on socket pin 28, GND on socket pin 12.
- PLCC28 GAL22V10: use `-s plcc28`.

Set the VCC and GND jumpers to match, and the supply jumper to 5 V, then
run:

    galtools/galre.py board -p /dev/ttyACM0 -d GAL22V10 -o out -t mychip

galre first checks the jumpers (`pld capture jumpers`):

1. The board finds the socket pins jumpered to VCC and GND without
   connecting the PLD supply. It switches on only the GND rail, which
   connects no supply, and drives one pin at a time high through its 1K
   resistor for 30 ms, after draining the VCC rail. A pin on the GND
   jumper is held low, and the VCC rail stays near 0 V. A pin on the VCC
   jumper charges the VCC rail, even when the chip draws so much current
   that the pin itself reads low (an A4091 U304 pulled it to 713 mV). The
   chip's other pins charge it too, through the chip's input protection,
   but lower (on the first board, an ATF22V10's pins reached 22 mV at
   most, the VCC pin 1691 mV), so the VCC pins are those within 250 mV of
   the highest reading. The chip only ever sees the 1K-limited drives.
2. Only if those pins are right does it switch the supply on to measure
   it: both rails on for 10 ms, as for a capture, so the reading is what
   the chip gets. It must be within 16% of 5 V.

If the jumpers do not suit the device and socket, galre stops before any
capture and says what it expected, what it found, and the board's
readings:

    jumper check: the jumpers do not suit a GAL22V10 in the dip24 socket
      expected: VCC on socket pin 28, GND on socket pin 12, supply 5 V
      detected: VCC on socket pin 24, GND on socket pin 12, supply not measured (only once VCC and GND are right)
      board: jumpers: GND rail on, Pin12 high: VCC rail ... mV, pin reads 0: GND
      board: ...

The firmware checks the same way before every run it is sent (about 2 s
with a chip in the socket), and drives the VCC pin high and the GND pin
low in every vector. The detection has been checked on one board, with an
ATF22V10 in the PLCC socket; its readings go into the report. If it gets
the jumpers wrong, `--no-jumper-check` turns off both checks.

This writes:

- `out/mychip.pld`: the recovered design.
- `out/report.txt`: the setup, pin roles, states, equations and board
  work.
- `out/capture.jsonl`: the setup, then every capture the board returned.
  The setup is the device, socket, timing, detected jumpers and the
  detection's readings, firmware version, held pins, survey size, title
  and date.
- With a CUPL compiler, also `out/mychip.jed`.

To redo the analysis without the board:

    galtools/galre.py replay out/capture.jsonl -o out2

The device, held pins, survey size and title come from the recording.
Recordings made before the setup was recorded need `-d`.

To try it without hardware, `sim` runs a simulated chip from a JED. It then
also compares the rebuilt JED with the original under random stimulus:

    galtools/galre.py sim galtools/designs/fsm22v10.jed -o /tmp/fsm

A design with an asynchronous reset term or combinational feedback loops
behaves differently depending on the exact order of the firmware's writes,
so `sim` runs it through the firmware built for the host (`fwsim.py`,
needs a C compiler). That model includes the three port writes per vector
(see Limits). It simulates the chip one write at a time, so it is slow
with many enumerated pins; hold the unused ones with `--fix`.

galre exits with:

- 0 when it wrote the design (and, with a CUPL compiler, the JED matches
  every capture);
- 1 when the equations or the JED disagree with the captures;
- 2 when the jumper check failed;
- 3 when the chip changes state without a clock in a way galre does not
  model, such as registers and latches in one design (see Limits).

Options:

- `--fix 6-11=0,13=1` holds pins at fixed values and does not enumerate
  them, for pins known to be unused or tied. Each held pin halves the
  enumeration. The recovered equations take the held values as given, so
  they match the chip only with those pins at those values. galre stops if
  the GAL drives a held pin.
- `--survey N` sets how many sample inputs per state are clocked in before
  the full sweeps (default 64).
- `--settle`, `--off` and `--on` set the firmware timing: µs after each
  write, ms with power off during a power cycle, and ms after power on.
- `--no-jumper-check` powers the socket without checking the jumpers.

## How it works

1. **Power-up walk.** galre walks every signal pin except pin 1 (the clock,
   held low) and the held pins. Every pin is driven weakly through Brutus's
   1K resistors, so a pin the GAL drives reads the same whatever its drive:
   that tells galre which pins the GAL drives.
   - An I/O pin whose driven value changes with the inputs is
     combinational.
   - Every other I/O pin is a state candidate: an input, a register or a
     constant.

   It then walks the power-up state three more times: in descending order,
   with the order of the pins reversed (the pin that changed fastest
   changes slowest), and both. A clocked chip reads the same in every
   order. One that does not holds state without a clock, and galre models
   it as latches (see Latches).
2. **States.** A state is the value or Z of each candidate. It is read
   through probe vectors: for each candidate, one input vector that enables
   it, taken from the walks. Each probe is read once as driven, then once
   per candidate with only that candidate's drive inverted.

   Starting from power-up, each new state gets:
   - an exhaustive walk. Each vector is read twice, the second time with
     the drive of the pins outside the enumeration inverted. The probes
     are read before and after the walk, and must show the same state;
   - a survey: a spread sample of inputs, each clocked in once;
   - then a full sweep: every input vector clocked in once.

   After every transition the firmware returns to the state under test:
   - by a clocked return path the host computed from the transitions it
     already knows (checked by reading the state's signature again);
   - or, failing that, by power-cycling and replaying the path from
     power-up.

   Some designs need special handling:
   - If a new state drives a candidate that no probe enables, probes are
     added and the exploration restarts.
   - If a candidate turns out to depend on the inputs, it is
     reclassified as combinational.
   - If clocking never changes anything, the design is taken as
     combinational and pin 1 is enumerated as an input.
3. **Register values.** galre assigns each register its value in each
   state. A register that is never enabled in a state takes the complement
   of the only value it is seen with.
4. **Equations.** For every output, output enable and register input,
   galre finds the support from single-variable flips and minimises it with
   a greedy two-level minimiser, on both polarities. The minimiser is the
   one from the MACH111 tester's mtest. galre then writes CUPL.

   A GAL22V10 register shows its output polarity on the pin whenever it is
   cleared, at power-up and by the asynchronous reset. galre writes a
   register that is high at power-up with a negated `.d`, which CUPL makes
   active low, so the rebuilt chip powers up the same way.

### Asynchronous reset (GAL22V10)

The GAL22V10's asynchronous reset is one product term, shared by every
register, that clears them all while it is true, without a clock. The
power-up state is all registers cleared, so the term never shows there. In
other states it shows in two ways:

- The walk of a state ends in the power-up state.
- In a transition sweep, reading the new state resets it. The firmware
  reads the clocked vector right after the clock edge and again after the
  probes, and reports the vectors where the two reads differ (`chk`). A
  reset input that is active low is low in most probe vectors, so this is
  usually the first sign.

galre then finds the term's inputs from that state:

1. It walks the state in Gray code order from the vector clocked in last,
   so one pin changes per write and no mix of old and new pin values (see
   Limits) can fire the term. The reads differ from the power-up walk up
   to the reset and agree after it. The last vector that reads differently
   on an output, d, tells the state from the power-up state.
2. For every enumerated pin and value, it walks the state again, skipping
   the vectors with that pin at that value and writing around them, then
   reads d. The state survives exactly when the term needs the pin at that
   value. These walks read back only the probe of d.

Then it explores again, around the term:

- Every write goes around the term's inputs: the pins that leave it are
  written first, then the others, then those that enter it, each port
  group on its own (`avm`, `avv`).
- Probe vectors lie outside it.
- A state whose walk still ends in the power-up state is one in which the
  term fires. It is walked again without the term's inputs. Its outputs
  and register inputs there are not seen, and are don't-cares: the
  rebuilt chip resets there too.

The term's register literals come from the states in which it fires and
those in which it does not. galre writes the term as `.ar` on every
register.

### Latches

A combinational output fed back into the array can hold a value: a
set/reset or transparent latch, or a bit that sets and stays set until
power-down. Its level then depends on the order in which the inputs
changed, not only on their present values. When the power-up walk reads
differently in another order, galre models the design as latches without
registers (`latch.py`):

- The pin roles come from the power-up walk. The dedicated inputs (pin 1
  too) and the I/O pins that read their own drive in every vector are
  inputs, the other I/O pins outputs.
- The inputs are walked in Gray code order from power-up, so one pin
  changes per write and no mix of old and new pin values (see Limits)
  reaches the chip. The walks go in every rotation of the bit order
  (`rot`), both ways (`rev`), so that every pin changes often and in many
  latch states.
- The latch pins are the outputs that read differently at the same input
  vector in two walks. Of those, an output that reads as a function of the
  inputs and the other latch pins is a plain output of the latches, such
  as `Y & !Z`, or the second output of a cross-coupled pair.
- Every output is a function of the inputs and the latch pins. For a
  latch pin it is the excitation: the chip rests where it equals the latch
  pins' levels and moves otherwise. Each step of a walk says that the
  vector before rested, and where the latches settled from the new one.
- When several latch pins change in one step, galre fires them in rounds:
  those of a round together (latches with a shared enable), the rounds one
  after the other (a latch that reads another flips after it). Of the
  splits that contradict nothing seen, it takes the one that best agrees
  with the steps that change a single latch.
- Each output is minimised, on both polarities, over the variables the
  steps show it depending on; where the walks did not go, it does not
  depend on the others either. A latch also gets, for every step that
  holds it, a product term that holds through the step, so the rebuilt
  latch cannot glitch there.
- galre then walks from other start vectors until the learned circuit
  predicts two walks in a row, learning again from those it gets wrong.
  Last, it checks that every output is always driven.

The equations read the latch pins, as the original's do:

    P17 = P4 & P5
        # !P5 & P17
        # P4 & P17;

The rebuilt JED is checked against the walks from the first read where it
agrees, and in `sim` against the original under random stimulus that
changes one input at a time. galre does not model the latches' state at
power-up, which in a cross-coupled pair is a race the rebuilt chip may
settle differently. It reports the walks that start differently.

Estimated board time is based on the simulator's count of operations
(about 2.5 µs per write and read, 22 ms per power cycle, and USB transfer):

| Design | Inputs enumerated | States | Board time |
|---|---|---|---|
| `designs/cnt16v8` (counter) | 14 | 4 | about 12 s |
| `designs/fsm22v10` (state machine) | 17 | 3 | about 110 s |
| A 22V10 ROM-board controller from the field | 19 | 6 | about 6 min, 377 power cycles |
| `designs/latch22v10` (three latches), pins 13-19 held | 11 | 24 walks | about 2 s |

The three extra power-up walks take up to a minute of that on a GAL22V10,
mostly USB transfer. Reading the clocked vector twice in the transition
sweeps adds about 5 µs per vector. An asynchronous reset term takes up to
two walks per enumerated pin to learn (about a second each with 19 pins),
and the states in which it fires are walked twice.

A latch design takes one Gray code walk per bit order and a few more to
check, each of every input vector, and learning grows with the number of
vectors: half a second for latch22v10 with 11 inputs.

Return paths replace almost all power cycles. Without them, the last design
needed about 3 million power cycles.

## Limits

- Only states reachable from power-up are seen, and the result matches the
  chip only on those states.
- A combinational output that depends only on registers looks the same as a
  register and is written as one. It behaves the same on the pins.
- Pin 1 stays low except for clock edges, so a combinational use of pin 1
  in a registered 22V10 is not captured.
- Registers that never reach a pin are not seen.
- Of state that changes without a clock, galre models the GAL22V10's
  asynchronous reset term and latches built from combinational feedback.
  It takes a design for latches when the power-up walk reads differently
  in another order. Tried in simulation on set/reset and transparent
  latches with their inputs on every pair of pins, and on set-only bits,
  the three extra orders together show every latch that can leave its
  power-up value. No two of them do. A latch that acts only in states
  other than power-up, in a registered design, is not seen.

  galre stops with exit code 3 before writing equations when:
  - a design has both registers and latches (the same inputs and latch
    levels then lead to different places);
  - a latch reacts only to several inputs changing at once;
  - a latch output is not always driven;
  - in a registered design, a state's walk does not leave it as it found
    it (other than by returning to the power-up state where the
    asynchronous reset term fires), or reading a new state in a
    transition sweep changes it.
- For latches, the result holds for inputs that change one at a time, and
  the latches' state at power-up is not modelled.
- The asynchronous reset term must be one product term of the enumerated
  inputs and the registers. A term that reads a combinational output
  stops galre.
- The firmware drives the socket with three port writes: socket pins 1-16,
  then 17-24, then 25-28. For a few tens of ns between them, the chip sees
  the new values on the pins written so far and the old ones on the rest.
  Clocked logic does not care, because a clock edge is a write of its own,
  but a latch or an asynchronous reset term can react to the mix. Once
  galre knows the reset term, every write goes around it. The walks that
  learn latches change one pin per write.

## Firmware commands

`pld capture ?` lists the commands. `pld capture jumpers` prints the
detection's readings, then `jumpers vcc=<pins> gnd=<pins> supply=<mV>`.
The supply is measured only when the pins match the job's `vcc=` and
`gnd=`, and is `-` otherwise.

The host first configures a job:

- `set pins= base= inv= clk= cand= mask= count= mul= rev= gray= rot=
  start= skm= skv= avm= avv= quiet= chk= settle= off= on= vcc= gnd=
  supply=`
- `path`
- `probe`
- `ret <crc> <vectors>`

When `vcc=` and `gnd=` are set, each run first checks the jumpers against
them, and against `supply=` if set, in the order above. It is refused if
they differ, or if the job enumerates, inverts, clocks or probes a VCC or
GND pin.

It then runs one of:

- `walk`: every vector of the enumerated pins. The i-th vector is
  x = (i * `mul`) mod 2^n, or the i-th Gray code with `gray=1`, rotated
  left by `rot` bits within its n bits. Bit b of x drives the b-th
  enumerated pin from the lowest socket pin up, or from the highest down
  with `rev=1`. Every vector is XORed with `start`.
  Vectors v with v & `skm` = `skv` are skipped (read as 0). With probe
  vectors set, the signature of the state is reported before and after the
  walk; with `quiet=1`, only the signatures are.
- `trans`: the clock transitions from the path state. With `chk=1`, the
  clocked vector is also read right after the edge and again after the
  probes, and the first 16 vectors where the reads differ are reported.
- `state`: the signature of the path state.

With `avm` set, no write passes through a vector v with v & `avm` = `avv`
on its way to the next vector, in any of the three port writes; a target
vector inside is still written.

Pin masks and vectors are 28-bit socket values in hex, bit n-1 for socket
pin n. A run answers with `---- CAPTURE ----` followed by CRC-checked binary
chunks. The header comment of `fw/capture.c` describes the format.
`capture.py` decodes it.

## Files

| File | Contents |
|---|---|
| `galre.py` | Command line |
| `analysis.py` | Discovery, exploration, equations, CUPL |
| `latch.py` | Latches built from combinational feedback |
| `rig.py` | `SimRig` (simulated chip), `BrutusRig` (board, jumper check), recording and replay, socket pin maps |
| `fwsim.py`, `fwstub/` | `fw/capture.c` built for the host, with a simulated GAL in the socket |
| `capture.py` | Serial transport and capture format |
| `galsim.py` | Fuse-level GAL16V8/22V10 simulator |
| `bitsim.py` | The same, evaluated for all input vectors at once |
| `verify.py` | Rebuilt JED against the captures, and JED against JED |
| `designs/` | Test designs (CUPL source and compiled JEDs) |
| `tests/` | Test suite |

Run the tests from the repository root:

    python3 -m unittest discover -s galtools/tests

The CUPL round trip runs only when a CUPL compiler is found. The firmware
tests need a C compiler.
