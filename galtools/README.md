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
> GAL simulator (`tests/fwharness.py` compiles `fw/capture.c` for the test
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

Set the VCC and GND jumpers to match, then run:

    galtools/galre.py board -p /dev/ttyACM0 -d GAL22V10 -o out -t mychip

This writes:

- `out/mychip.pld`: the recovered design.
- `out/report.txt`: pin roles, states, equations and board work.
- `out/capture.jsonl`: every capture the board returned.
- With a CUPL compiler, also `out/mychip.jed`.

To redo the analysis without the board:

    galtools/galre.py replay out/capture.jsonl -d GAL22V10 -o out2

To try it without hardware, `sim` runs a simulated chip from a JED. It then
also compares the rebuilt JED with the original under random stimulus:

    galtools/galre.py sim galtools/designs/fsm22v10.jed -o /tmp/fsm

Options:

- `--fix 6-11=0,13=1` holds pins at fixed values and does not enumerate
  them, for pins known to be unused or tied. Each held pin halves the
  enumeration.
- `--survey N` sets how many sample inputs per state are clocked in before
  the full sweeps (default 64).
- `--settle`, `--off` and `--on` set the firmware timing: µs after each
  write, ms with power off during a power cycle, and ms after power on.

## How it works

1. **Power-up walk.** galre walks every signal pin except pin 1 (the clock,
   held low). Every pin is driven weakly through Brutus's 1K resistors.
   Each vector is read twice, once as driven and once with the pins outside
   the enumeration inverted, so galre can tell which pins the GAL drives.
   - An I/O pin whose driven value changes with the inputs is
     combinational.
   - Every other I/O pin is a state candidate: an input, a register or a
     constant.
2. **States.** A state is the value or Z of each candidate. It is read
   through probe vectors: for each candidate, one input vector that enables
   it, taken from the walks. Each probe is read once as driven, then once
   per candidate with only that candidate's drive inverted.

   Starting from power-up, each new state gets:
   - an exhaustive walk;
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
| `designs/cnt16v8` (counter) | 14 | 4 | about 10 s |
| `designs/fsm22v10` (state machine) | 17 | 3 | about 70 s |
| A 22V10 ROM-board controller from the field | 19 | 6 | about 5 min, 374 power cycles |

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
- Asynchronous reset (22V10 AR) and latches built from feedback change state
  during a walk. The firmware can report signatures before and after a walk
  to detect this, but galre does not handle such designs.

## Firmware commands

`pld capture ?` lists the commands. The host first configures a job:

- `set pins= base= inv= clk= cand= mask= count= mul= settle= off= on=`
- `path`
- `probe`
- `ret <crc> <vectors>`

It then runs one of:

- `walk`: every vector of the enumerated pins.
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
| `rig.py` | `SimRig` (simulated chip), `BrutusRig` (board), recording and replay, socket pin maps |
| `capture.py` | Serial transport and capture format |
| `galsim.py` | Fuse-level GAL16V8/22V10 simulator |
| `bitsim.py` | The same, evaluated for all input vectors at once |
| `verify.py` | Rebuilt JED against the captures, and JED against JED |
| `designs/` | Test designs (CUPL source and compiled JEDs) |
| `tests/` | Test suite, including the host build of `fw/capture.c` |

Run the tests from the repository root:

    python3 -m unittest discover -s galtools/tests

The CUPL round trip runs only when a CUPL compiler is found. The firmware
tests need a C compiler.
