# galtools: black-box GAL reverse engineering with Brutus-28

`galre.py` recovers the logic of a GAL16V8 or GAL22V10 from what Brutus-28
can observe at the socket: combinational and registered designs, the
GAL22V10's asynchronous reset, and latches built from combinational
feedback, also inside a registered design. It writes CUPL source and, when a CUPL compiler is available,
builds a JED and checks it against every captured state and input vector.

For a step-by-step guide, see [HOWTO_REVERSE.md](HOWTO_REVERSE.md).

`pld walk` sees only the power-up state, so it cannot recover registered
logic. galre also clocks the chip from state to state and walks each
reachable state. The firmware side is the `pld capture` command
(`fw/capture.c`).

> **Status:** first board run on 2026-11-18: an ATF22V10 in the PLCC
> socket, a ROM-board state machine (19 pins enumerated, 6 states), came
> out the same as the analysis of its original JED, equation for equation.
> The asynchronous reset and latch paths have run only on the host, against
> the GAL simulator (the test suite builds `fw/capture.c` for the host).

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

With `csim`, CUPL's simulator, on the PATH (or `--csim`, `$CSIM`), galre
also asks it for a second opinion (`csimcheck.py`): it writes test vectors
(`out/mychip.si`) from what its own simulator says the JED does under
random stimulus, and runs csim on the recovered source. csim works from
the compiled source and the device library, not from the fuse map, so an
agreement checks galre's simulator and the compiler against each other.
csim does not know the level of a latch that nothing has set since
power-up; galre expects such levels as unknown.

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
(see Limits). The chip in the socket is then simulated in C as well
(`fwstub/galmodel.c`, the same model as `galsim.py`; a test compares the
two read for read), at a few million writes per second: a design with
latches, three registers and 15 enumerated pins takes about 15 s.

galre exits with:

- 0 when it wrote the design (and, with a CUPL compiler, the JED matches
  every capture);
- 1 when the equations or the JED disagree with the captures;
- 2 when the jumper check failed;
- 3 when galre stops before writing equations: the chip changes state
  without a clock in a way galre does not model, has state that no pin
  shows, or is beyond a limit such as `--max-states` (see Limits). The
  last lines of the report say which.

Options:

- `--fix 6-11=0,13=1` holds pins at fixed values and does not enumerate
  them, for pins known to be unused or tied. Each held pin halves the
  enumeration. The recovered equations take the held values as given, so
  they match the chip only with those pins at those values. galre stops if
  the GAL drives a held pin.
- `--survey N` sets how many sample inputs per state are clocked in before
  the full sweeps (default 64).
- `--max-states N` is where galre gives up on a state machine (default
  1024). The firmware takes paths of up to 512 clocks and 2048 different
  states per sweep.
- `--settle`, `--off` and `--on` set the firmware timing: µs after each
  write, ms with power off during a power cycle, and ms after power on.
- `--no-jumper-check` powers the socket without checking the jumpers.

## How it works

Brutus drives every pin through a 1K resistor, except the clock: through
the 1K, the clock edge is slow, and a GAL22V10 then takes some transitions
wrongly, because its registers do not all switch at the same moment and
one can catch another's new value through the feedback. On the first
board run, a few transitions in a thousand from one state came out wrong
that way, and not always the same ones. So the
firmware drives pin 1 directly from the CPU (`dclk`), which is safe because
pin 1 of a GAL is always an input. Also, just before each clock edge, it
drives every pin at the level it reads (`follow`), so no pin the GAL
drives is fought through its 1K at the edge. `--no-dclk` and `--no-follow`
turn these off.

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
  (`rot`), both ways (`rev`), and in mixed Gray code orders (`mix`, see
  Limits), so that every pin changes often and in many
  latch states.
- The latch pins are the outputs that read differently at the same input
  vector in two walks. Of those, an output that reads as a function of the
  inputs and the other latch pins is a plain output of the latches, such
  as `Y & !Z`, or the second output of a cross-coupled pair.
- Every output is a function of the inputs and the latch pins. For a
  latch pin it is the excitation: the chip rests where it equals the latch
  pins' levels and moves otherwise. Each step of a walk says that the
  vector before rested, and where the latches settled from the new one.
- The chip is seen at rest only, so a step proves little about what the
  latches do on the way:
  - nothing changed: every latch is excited to stay, and may not glitch
    between the two points;
  - one latch changed: that latch is excited at the step's first point.
    What the others do there is not known: one may be excited for a gate
    delay and fall back once the first has moved (in a GAL from an A4000,
    pin 14 pulses low while pin 18 rises);
  - several changed: in which order, and through which levels, is not
    seen.

  galre learns the circuit from the first two. It then runs the learned
  circuit through every step, every excited latch moving at once (one
  gate delay each, as in the chip's simulator), and it must come to rest
  where the chip did. Where it does not, the step says more: a latch that
  the circuit moves and the chip left alone holds there; a step that
  changed several latches is split into rounds, those of a round moving
  together, of the splits that contradict nothing seen the one that best
  agrees with the rest. Then every step is checked again.
- Each output is minimised, on both polarities, over the variables the
  steps show it depending on; where the walks did not go, it does not
  depend on the others either. A latch also gets, for every step that
  holds it, a product term that holds through the step, so the rebuilt
  latch cannot glitch there.
- galre then walks from other start vectors until the learned circuit
  predicts as many walks in a row as make 16384 steps (at least two, at
  most 64; mixed Gray code walks), learning again from those it gets
  wrong.
  Last, it checks that every output is always driven.

The equations read the latch pins, as the original's do:

    P17 = P4 & P5
        # !P5 & P17
        # P4 & P17;

The last term changes nothing at rest: it holds the latch while P5
changes. CUPL's minimiser removes such terms, so the source says not to
minimise the latches (`MIN [P17, P18] = 0;`). What CUPL needs is all in
the source: that statement, the device (for a GAL16V8 sometimes its mode,
`g16v8ma` or `g16v8ms`), and a comment with the command galre used,
`cupl -jaxm4 NAME.pld`. If the design does not fit with the extra terms,
galre leaves the statement out and says so.

The rebuilt JED is checked against the walks from the first read where it
agrees, and in `sim` against the original under random stimulus that
changes one input at a time. galre does not model the latches' state at
power-up, which in a cross-coupled pair is a race the rebuilt chip may
settle differently. It reports the walks that start differently.

### Registers and latches together

A registered design can hold state in a latch too, and a GAL22V10's
product terms can read pin 1, the clock, like any other input. An example
from an Amiga SCSI controller: a chip select that is set while the clock
is high and a register is set, holds itself while that register and an
input allow, and is let go when the reset term clears the register.
Such a latch does not show in the power-up walk. It shows when, after a
clock edge, reading the new state changes a pin without a clock, and it is
not the reset term. galre then goes on (`reglatch.py`):

1. The pins that change without a clock are no state candidates any more,
   and the transition sweeps leave them out of their reads (`mask`). What
   is left are the registers. galre explores them as usual: states,
   transitions, the reset term. Each sweep also records the level of every
   pin just before each clock edge (`pre`), because a register input may
   read a latch.
2. It walks pin 1 together with the enumerated pins, in Gray code order,
   so the walk clocks the chip whenever it raises pin 1. galre follows the
   register state through such a walk. A clock edge with the outputs at
   the levels the sweep had goes where the sweep went. With a latch at
   another level, it may go elsewhere: galre takes it on trust, and
   checks every registered pin that is driven in the reads that follow.
   A register that reads otherwise loaded the other value; galre keeps
   that edge, and so learns the register inputs with the latch pins as
   inputs. The walks are:
   - from every state, two with pin 1 changing slowest: every input
     vector with pin 1 high, then every one with pin 1 low, around the
     reset term where it would fire;
   - from power-up, one for every other position of pin 1 in the bit
     order, both ways, so that the chip is clocked and reset in many
     places;
   - then walks from other states and start vectors, until the learned
     circuit predicts as many in a row as make 16384 steps (at least
     two, at most 64), in mixed Gray code order, so pin 1 rises often
     and in many places.
3. The outputs that read differently at the same inputs, pin 1 level and
   register state are the latch pins, less those that are a function of
   these and the other latch pins. Every output, enable and register
   input is learned per context: a register state with the levels of the
   latch pins. A latch pin's excitation is learned as for latches alone,
   with pin 1 as an input: a step that changes one pin, or clocks the
   chip, says where the latches settle from the levels they had.

   A latch can sit on a bidirectional pin: the GAL drives it only while
   its enable is true, and reads it as an input otherwise (a bus signal
   such as DTACK). galre enumerates such a pin like an input. The GAL
   drives it for sure where it is not at the level Brutus drives it to,
   and does not where it follows when the walk changes its drive. The
   enable is one product term; galre takes the shortest one that holds
   all the first reads and none of the second. The other equations read
   the pin's level, whoever drives it.
4. The equations are written over pin 1 (as `CLK`), the enumerated pins,
   the registers and the latch pins. A latch gets a product term for
   every step that holds it. From `designs/reglatch22v10`:

       P21 = CLK & !P3 & P4
           # CLK & P3 & P22
           # P3 & P22 & P21
           # CLK & P4 & P22;

   The last term is not in the original: it holds the latch while P3
   changes with the clock high.

Every output's pin is a signal with a gate delay of its own, and another
output may read it. That matters when a latch is held through another
output: in `designs/racelatch16v8`, L is held by C or by NC, the
complement of C on another pin. By its equation it always holds. In the
chip it drops when C falls, because the term with C lets go a gate delay
before the one with NC holds. No circuit in which only the latches have
a delay does that, and galre finds contradicting samples. It then learns
again with every output pin the array can read as a variable, so that the
equations read other outputs' pins as the original's do
(`!P16 = !P3 & !P15 # !P2 & !P16`, with pin 15 for NC). It does this only
as a second attempt, and only up to 22 bits of inputs, register states
and outputs together.

If the power-up walk does read differently in another order, galre first
tries latches alone, and then registers and latches together
(`designs/reglatch16v8`). A design without registers goes the same way if
its outputs have enables, which the analysis of latches alone does not
take; pin 1 is then one more input.

An output with a constant value and an enable (`X = 'b'0; X.oe = ...`, an
open-drain bus signal) is not enumerated at first: undriven, it sits at
the level Brutus drives it to, 0, or 1 in every second read. If other
logic reads that pin, it is an input as well. After the exploration,
galre therefore walks (and sweeps) every state with all such pins held at
the level they do not drive, and once more for each with that pin at the
other level. A pin whose level makes a difference is enumerated, and
handled like a latch on a bidirectional pin, so that the equations read
its level (`designs/oeread22v10`: `P22.oe = !P23`).

Tried in simulation on the eight GALs of a Zorro III SCSI controller,
which have up to six registers, three latches and 52 states: latches
that read the clock pin, latches on bidirectional bus pins, and register
inputs that read latches.

The rebuilt JED is also run through every walk with pin 1, and compared
read for read.

Estimated board time is based on the simulator's count of operations
(about 2.5 µs per write and read, 22 ms per power cycle, and USB transfer):

| Design | Inputs enumerated | States | Board time |
|---|---|---|---|
| `designs/cnt16v8` (counter) | 14 | 4 | about 12 s |
| `designs/fsm22v10` (state machine) | 17 | 3 | about 110 s |
| A 22V10 ROM-board controller from the field | 19 | 6 | about 6 min, 377 power cycles |
| `designs/latch22v10` (three latches), pins 13-19 held | 11 | 24 walks | about 2 s |

The six extra power-up walks take up to two minutes of that on a GAL22V10,
mostly USB transfer, and every other state is walked twice. Reading the clocked vector twice in the transition
sweeps adds about 5 µs per vector. An asynchronous reset term takes up to
two walks per enumerated pin to learn (about a second each with 19 pins),
and the states in which it fires are walked twice.

A latch design takes one Gray code walk per bit order and a few more to
check, each of every input vector, and learning grows with the number of
vectors: half a second for latch22v10 with 11 inputs.

Latches in a registered design take, on top of the registers, two walks
per state and two per enumerated pin, each of every input vector with
pin 1 high and low. With 17 pins enumerated and 8 states that is about 50
walks of 262144 vectors, about 5 s each on the board.

Return paths replace almost all power cycles. Without them, the last design
needed about 3 million power cycles.

## Limits

- Only states reachable from power-up are seen, and the result matches the
  chip only on those states.
- A combinational output that depends only on registers looks the same as a
  register and is written as one. It behaves the same on the pins.
- In a registered design, pin 1 stays low except for clock edges, unless
  galre finds latches in it and walks pin 1 with the inputs. So a
  combinational use of pin 1 in a registered GAL22V10 is captured only in
  a design with latches. A latch that is open while the clock is high and
  holds whatever the inputs do while it is low looks like a register, and
  is written as one.
- Registers that never reach a pin are not seen.
- Of state that changes without a clock, galre models the GAL22V10's
  asynchronous reset term and latches built from combinational feedback.
  It takes a design for latches when the power-up walk reads differently
  in another order. Tried in simulation on set/reset and transparent
  latches with their inputs on every pair of pins, and on set-only bits,
  the three counting orders (descending, bit-reversed, both) together
  show every latch that can leave its power-up value. No two of them do.
  In a counting order, though, a pin never rises alone: every faster pin
  falls with it. A latch that holds only while another signal keeps
  still (`!P14 = !P8 # !P14 & !P13`, from a graphics card's GAL) never
  holds then, and the chip reads as combinational. So the power-up state
  is also walked in Gray code order and in two mixed Gray code orders,
  and every other state once in a mixed one, from the vector the path
  left it at; these must read the same as the counting walk. A latch in
  a registered design that acts only in other states also shows when
  reading a new state changes it.

  A mixed Gray code changes one pin per vector like the usual one, but
  is walked as one half, one pin's change, the other half, and each half
  again so, with the pin that splits each part chosen by a hash (`mix`).
  The usual Gray code changes its lowest pin at every second vector and
  its highest once. In a mixed one, which pins change often differs from
  part to part, so most pins change about equally often, and a few
  values of `mix` cover them all. What a latch does when a pin changes
  depends on where the other pins and latches stand, so this shows much
  more of it per walk.

  galre stops with exit code 3 before writing equations when:
  - the chip has state that no pin shows (a register whose output is
    never enabled): the same clocks then lead to different states;
  - a state is reached only with a latch at a level that the sweeps never
    had at that clock edge, or a clock edge goes another way because of a
    latch and no registered pin shows it before the next edge;
  - a latch reacts only to several inputs changing at once;
  - an output oscillates (it feeds back inverted, as in a ring
    oscillator): its level when read is then arbitrary, and the samples
    contradict each other;
  - in a design of latches alone, a latch output is not always driven and
    the registers-and-latches analysis cannot take it either;
  - in a registered design, something other than the reset term or a
    latch pin changes without a clock.
- For latches, the result holds for inputs that change one at a time, and
  the latches' state at power-up is not modelled. In a registered design,
  a clock edge is taken to change pin 1 and the registers at the same
  moment; a latch that catches the few ns between the two is not
  modelled. The equations hold where the walks went: every input vector
  in every register state at both levels of pin 1, but not every latch
  level in each of them. Where a latch and a condition always go
  together, an equation or enable may read the one where the original
  reads the other, or both.
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
  mix= start= skm= skv= avm= avv= quiet= chk= pre= follow= dclk= settle=
  off= on= vcc= gnd= supply=`
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
  with `rev=1`. With `mix` not 0 (and `gray=1`) the order is the mixed
  Gray code that `mix` selects. Every vector is XORed with `start`.
  Vectors v with v & `skm` = `skv` are skipped (read as 0). The
  enumerated pins may include the clock pins: after the path, the walk
  then drives them like any other pin, and clocks the chip when it raises
  them. With probe
  vectors set, the signature of the state is reported before and after the
  walk; with `quiet=1`, only the signatures are.
- `trans`: the clock transitions from the path state. With `chk=1`, the
  clocked vector is also read right after the edge and again after the
  probes, and the first 16 vectors where the reads differ are reported,
  each with the pins that differ. Pins left out of `mask` do not count.
  With `pre=1`, every vector's record is followed by the read of all
  pins just before its clock edge.
- `state`: the signature of the path state.

Every clock step drives its vector with the clock pins low, then raises
them. With `follow=1`, the pins are read after the first write and every
read pin is driven at the level it read before the clock rises. With
`dclk=1`, the clock pins are driven directly by the CPU's PLD_* pins
instead of through the 1K, and only while the chip is powered; a clock
pin may not be a VCC or GND pin.

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
| `reglatch.py` | Latches in a registered design, and pin 1 as an input |
| `rig.py` | `SimRig` (simulated chip), `BrutusRig` (board, jumper check), recording and replay, socket pin maps |
| `fwsim.py`, `fwstub/` | `fw/capture.c` built for the host, with a simulated GAL in the socket (`galmodel.c`: the GAL in C) |
| `capture.py` | Serial transport and capture format |
| `galsim.py` | Fuse-level GAL16V8/22V10 simulator |
| `bitsim.py` | The same, evaluated for all input vectors at once |
| `verify.py` | Rebuilt JED against the captures, and JED against JED |
| `csimcheck.py` | Test vectors from the rebuilt JED for CUPL's simulator, and its verdict |
| `designs/` | Test designs (CUPL source and compiled JEDs) |
| `tests/` | Test suite |

Run the tests from the repository root:

    python3 -m unittest discover -s galtools/tests

The CUPL round trip runs only when a CUPL compiler is found. The firmware
tests need a C compiler.
