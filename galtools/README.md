# galtools: black-box GAL reverse engineering with Brutus-28

`galre.py` recovers the logic of a GAL16V8 or GAL22V10, registered designs
included, from what Brutus-28 can observe at the socket. It writes CUPL
source and, when a CUPL compiler is available, builds a JED and checks it
against every captured state and input vector.

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
   connecting the PLD supply. With both PLD rails off, it drives each pin
   high through its 1K resistor and keeps the pins that lift the rails.
   It then switches on only the GND rail, which connects no supply, and
   drives those pins again: a pin on the VCC jumper charges the VCC rail,
   and a pin on the GND jumper stays low. The chip only ever sees the
   1K-limited drives.
2. Only if those pins are right does it switch the supply on to measure
   it: the VCC rail on and the GND rail open for 10 ms, as `pld check`
   does. It must be within 16% of 5 V.

If the jumpers do not suit the device and socket, galre stops before any
capture and says what it expected, what it found, and the board's
readings:

    jumper check: the jumpers do not suit a GAL22V10 in the dip24 socket
      expected: VCC on socket pin 28, GND on socket pin 12, supply 5 V
      detected: VCC on socket pin 24, GND on socket pin 12, supply not measured (only once VCC and GND are right)
      board: jumpers: rails off, Pin12 high: VCC rail ... mV, GND rail ... mV
      board: ...

The firmware checks the same way before every run it is sent, and drives
the VCC pin high and the GND pin low in every vector. The detection's
thresholds and timing have not yet been checked on a board, with or
without a GAL in the socket; its readings go into the report. If it gets
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
(see Limits).

galre exits with:

- 0 when it wrote the design (and, with a CUPL compiler, the JED matches
  every capture);
- 1 when the equations or the JED disagree with the captures;
- 2 when the jumper check failed;
- 3 when the chip changes state without a clock (see Limits).

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
   order; see Limits for what galre does when it does not.
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

Estimated board time is based on the simulator's count of operations
(about 2.5 µs per write and read, 22 ms per power cycle, and USB transfer):

| Design | Inputs enumerated | States | Board time |
|---|---|---|---|
| `designs/cnt16v8` (counter) | 14 | 4 | about 12 s |
| `designs/fsm22v10` (state machine) | 17 | 3 | about 110 s |
| A 22V10 ROM-board controller from the field | 19 | 6 | about 6 min, 377 power cycles |

The three extra power-up walks take up to a minute of that on a GAL22V10,
mostly USB transfer.

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
- State that changes without a clock is not modelled: the GAL22V10's
  asynchronous reset term, and latches built from combinational feedback.
  galre detects it and stops with exit code 3 before writing equations:
  - The power-up walk must read the same in all four orders. Tried in
    simulation on set/reset and transparent latches with their inputs on
    every pair of pins, and on set-only bits, the three extra orders
    together show every latch that can leave its power-up value. No two of
    them do.
  - Every other state's walk must leave the state as it found it. The
    asynchronous reset shows here, as a return to the power-up state.

  A latch that acts only in states other than power-up, on pins that are
  not state candidates, is not seen.
- The firmware drives the socket with three port writes: socket pins 1-16,
  then 17-24, then 25-28. For a few tens of ns between them, the chip sees
  the new values on the pins written so far and the old ones on the rest.
  Clocked logic does not care, because a clock edge is a write of its own,
  but a latch or an asynchronous reset term can react to the mix.

## Firmware commands

`pld capture ?` lists the commands. `pld capture jumpers` prints the
detection's readings, then `jumpers vcc=<pins> gnd=<pins> supply=<mV>`.
The supply is measured only when the pins match the job's `vcc=` and
`gnd=`, and is `-` otherwise.

The host first configures a job:

- `set pins= base= inv= clk= cand= mask= count= mul= rev= settle= off=
  on= vcc= gnd= supply=`
- `path`
- `probe`
- `ret <crc> <vectors>`

When `vcc=` and `gnd=` are set, each run first checks the jumpers against
them, and against `supply=` if set, in the order above. It is refused if
they differ, or if the job enumerates, inverts, clocks or probes a VCC or
GND pin.

It then runs one of:

- `walk`: every vector of the enumerated pins. The i-th vector is
  x = (i * `mul`) mod 2^n, with bit b of x driving the b-th enumerated pin
  from the lowest socket pin up, or from the highest down with `rev=1`.
  With probe vectors set, the signature of the state is reported before
  and after the walk.
- `trans`: the clock transitions from the path state.
- `state`: the signature of the path state.

Pin masks and vectors are 28-bit socket values in hex, bit n-1 for socket
pin n. A run answers with `---- CAPTURE ----` followed by CRC-checked binary
chunks. The header comment of `fw/capture.c` describes the format.
`capture.py` decodes it.

## Files

| File | Contents |
|---|---|
| `galre.py` | Command line |
| `analysis.py` | Discovery, exploration, equations, CUPL |
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
