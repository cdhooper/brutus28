# galtools: black-box GAL reverse engineering with Brutus-28

`galre.py` recovers the logic of a GAL16V8, GAL20V8 or GAL22V10 from what Brutus-28
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
- A CUPL compiler, to turn the result into a JED and check it against the
  captures (`--cupl`, `$CUPL`, or `cupl` on the PATH). It is run as
  `cupl -jaxm4 NAME.pld`, so WinCUPL's `cupl` or a native build of it
  will do. `galre board` asks for it before it starts and tries it on one
  gate for the device, because a result that was not built is not checked.
  `--no-cupl` records without it, for a replay where the compiler is.
  `sim` and `replay` run without one and say that nothing was checked.

## Use

Put the chip at the top of the 28-pin socket, pin 1 to pin 1, as described
in the main README:

- GAL16V8: VCC on socket pin 28, GND on socket pin 10.
- GAL22V10: VCC on socket pin 28, GND on socket pin 12.
- PLCC28 GAL22V10: use `-s plcc28`.
- PLCC20 GAL16V8: use `-s plcc20`; VCC is then on socket pin 20. See
  the caution in HOWTO_REVERSE.md, section 3, before you power one from
  5 V.

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
  `galre board` raises `--off` by itself for a chip that a power cycle of
  that length does not reset (see How it works).
- `--no-jumper-check` powers the socket without checking the jumpers.
- `--once` runs every sweep of clock edges once. By default `galre board`
  runs each twice, and five times where the two differ (see How it works).
  `--twice` does the same in `sim`.
- `--no-serial` changes all pins of a vector in one write, as firmware
  without `ser=` does.
- `--no-oe-first` walks pin 11 of a GAL16V8 (pin 13 of a GAL20V8) with the
  other inputs from the start. By default galre holds it low and walks it
  alone, if clock edges change the chip (see How it works).

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

The firmware writes the 28 socket pins in three port writes, and the pins
of one port write reach the chip in no certain order. A chip can take that
for something else: a GAL in a PLCC28 loaded its registers, as on a clock
edge, in one of 20 writes in which the pin that enables its outputs fell
together with four inputs, and never when they fell in two writes. That
happened where a state was read after a clock, and at the carries of a
walk that counts. So the firmware changes one pin per write (`ser`), the
lowest socket pin first: a vector that differs from the last one in n pins
takes n writes. `--no-serial` turns that off, and firmware that does not
have it is used as it is; the run says so.

A power cycle has to reset the chip, and the default 20 ms without power
do not reset every chip: the same part kept its charge. It then came up
in the state that a clock with every input low leads to, every time, and
every job started from that state as if it were the power-up state (the
supply is measured just before a job, so the chip has been powered then).
After 50 ms it came up as it should, every registered output high. So
`galre board` first reads the pins after a power cycle of `--off` and
after one of a second, and if they differ, doubles the power-off time
until they agree and adds half again. The report says so ("A power cycle
of 20 ms does not reset this chip"). It sees this only where the state
shows on the pins with every input low or high.

A GAL16V8 or GAL20V8 with registers uses pin 11 (13) to enable their
outputs and for nothing else. Walked together with the I/O pins, that pin
switches the registered outputs on against the levels the board holds
their pins at, all of them at once. The same part took that for a clock
edge: every time the pin fell while six or more of its eight outputs came
on against a pin held low, never with five or fewer, and whichever pins
they were. The supply at the socket gives way while they switch. Its
registers then changed in walks without a clock, and galre took them for
latches. (The one write in 20 above was most likely the same thing: the
outputs came on against the board there as well, and they do with one pin
per write too. It would have shown only where an input of the registers
changed in that write, because the edge otherwise led to the state the
chip was already in.) So for these two devices galre first clocks a few
input vectors in with the pin low. If that changes what the chip drives, it holds the
pin low for the whole run and walks it alone, with no I/O pin held, to
see what it switches off ("pin 13, walked alone, switches off pins ...").
That must be the registers, all of them, and nothing else. If it is not
(a chip with latches as well, say), galre starts again with the pin walked
like any other input, as it does from the start for a chip that clock
edges do not change. Holding the pin also halves every walk and sweep.
`--no-oe-first` turns it off. A GAL22V10 has no such pin: its enables are
product terms, and galre walks their inputs with the I/O pins. A chip that
does the same there shows state that changes without a clock; holding the
input that enables the outputs (`--fix`) gets around it, and the
equations then lack that enable.

A wrong reading in a sweep of clock edges would pass for the chip's logic
(one more product term), or for a latch. So `galre board` runs every sweep
twice. Where the two differ, it runs the sweep five times in all and takes
each input vector to lead where most of them said. The report then says
"the chip does not answer the same every time" for that state, the source
says so in a comment, and the run ends with a NOTE: such a result is
checked only against what most sweeps said, so run it again and compare.
The second sweep is quick: by then the firmware knows the way back from
every state the first one met. `--once` turns it off. A job that the
firmware gives up because two power cycles did not lead back to its state
is run again, twice at most.

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

   On a board such a chip may also read differently when the same walk
   is repeated. Counting order changes several inputs in one step, every
   pin has its own 1K, capacitance and threshold, and a rising pin
   crosses the threshold at another time than a falling one. A real
   GAL16V8 with `designs/oelatch16v8` in it (`L = S # L & R`) dropped L
   in about one of three steps in which S fell while R rose, and kept it
   in the others. galre then walks one input at a time, twice. If that
   reads the same both times, the chip has latches, and neither a bad
   contact nor an output that does not settle; the latch analyses change
   one input at a time throughout. `sim --race SEED` simulates such a
   chip: the inputs of one step arrive one after the other, in an order
   that differs from time to time.
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
   - A candidate that the reset term reads (an I/O pin used as an input)
     is no candidate once the term is known: reading it with its drive
     inverted would fire the term.
   - A candidate that shows one level where it is driven, and that a
     transition or another pin depends on where the chip lets it go, is
     a constant output with an enable, not a register: a register's
     feedback comes from the register, not from its pin. An example is
     an open-drain copy of a register that a register input reads back
     (`designs/aropen22v10`).
   - If clocking never changes anything, the design is taken as
     combinational and pin 1 is enumerated as an input.
3. **Register values.** galre assigns each register its value in each
   state. A register that is never enabled in a state takes the complement
   of the only value it is seen with.
4. **Equations.** For every output, output enable and register input,
   galre finds the support from single-variable flips and minimises it with
   a greedy two-level minimiser, on both polarities. The minimiser is the
   one from the MACH111 tester's mtest. galre then writes CUPL.

   A function that needs more product terms over the inputs and registers
   than its macrocell has (7 with an enable or 8 on a GAL16V8 and GAL20V8,
   8 to 16 on a GAL22V10) was built with another output's pin in the
   original: a parity of a parity, a comparison that feeds a decoder.
   galre writes such a function again over the inputs, the registers and
   the pins of the other outputs, and takes that if it has fewer terms
   (`!P15 = !P18 & !P19 & !P21 # ...`, four terms where pin 21 is a sum
   of four itself; ten over the inputs). Only outputs that are always
   driven and written over the inputs themselves are read, so nothing
   reads in a loop, and on a GAL16V8 and GAL20V8 without registers only
   the six middle output pins, the ones that feed back in complex mode.

   A register whose output is not enabled in some states, whatever the
   inputs, does not show there. If nothing seen depends on it there, it
   is taken as cleared, and its `.d` says so.

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
  predicts as many walks in a row as make 131072 steps (at least two, at
  most 128; mixed Gray code walks), learning again from those it gets
  wrong. That bounds how often the learned circuit can be wrong, it does
  not rule it out: a latch that is set by one combination of eleven
  inputs is seen to set a few times in all, and never just next to that
  combination, so nothing says which of the inputs matter. With a sixth
  of the steps, such a design passed and was wrong in two steps of 30000.
- Where a function is known only at the points the walks went through,
  its product terms claim little beyond them. Every term has the literals
  that hold wherever the function was seen on. A term that holds a latch
  (it reads the latch's own pin) is shrunk to the smallest one around the
  points it covers: what holds a latch is seen wherever it rests. The
  terms that set it are seen only in the steps that do, and stay as
  large as nothing contradicts; shrunk too, they generalise so little
  that the design stops fitting the chip. The term that keeps a latch
  from glitching between two points stays within the other terms.
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
galre leaves the statement out and says so. If CUPL's minimiser then
still does not get it to fit (registers and latches together), galre
writes its own covers without those terms, and keeps the statement.

The rebuilt JED is checked against the walks from the first read where it
agrees, and in `sim` against the original under random stimulus that
changes one input at a time. galre does not model the latches' state at
power-up, which in a cross-coupled pair is a race the rebuilt chip may
settle differently. It reports the walks that start differently.

### Outputs that do not settle

An output that feeds back inverted has no level while its loop is closed.
With

    P17 = !P1 & !P5
        # !P17 & !P5;

(from an accelerator board's GAL), pin 17 is its own complement while
pin 1 is high and pin 5 low, and toggles as fast as the chip lets it: a
ring oscillator of one gate. A read then catches an arbitrary level, and
the same walk reads differently twice. The simulated chip does the same:
a pin that does not settle reads an arbitrary level at every read.

Where a pin does not settle is itself what there is to learn. When the
power-up walk reads differently twice, galre (`osc.py`) reads every
vector twelve times in each of four walk orders or more, which says for
every output and every vector whether it settles there, and at which
level:

- A pin that does not settle at a vector can only be the complement of
  itself there: its equation is 1 with the pin low and 0 with it high.
  Where it settles, the equation is its level.
- An output that is unsteady only where such a pin is, is a function of
  that pin there, and is taken to follow it. Following its complement
  explains the same reads, because the phase is not seen; of the two,
  galre writes the one with fewer product terms and literals, and says
  so in a comment in the source.
- An output that settles where the pin does not is the same at both
  levels of the pin.

The equations above come out as they are, and `designs/osc16v8` as its
source. The rebuilt JED is put through the same reads: every output must
settle at the same vectors, at the same levels.

An oscillator fails to settle at the same vectors every time. A pin that
is unsteady at other vectors in another walk order is a bad contact or
a floating pin, and galre says that instead. It also stops when the chip
holds state as well (`designs/oscstate16v8`), at a ring through several
pins, and at outputs that are not always driven. The frequency is the
chip's own.

A pin that toggles is not at both levels equally often. On a real
GAL16V8 with `designs/osc16v8` in it, the oscillating pin read high in
45 of 100 reads, and the output of the gate behind it in 19. Twelve
reads of a vector are then all low now and then, in one walk order of
nine there, and the vector looks settled in that order. After the first
four orders galre counts how often that happened, and reads as many
more as it takes for a vector to be unlikely to look settled in all of
them (seven for that chip). If a pin looks settled in one walk order at
more than half of the vectors at which it is unsteady in another, reads
cannot tell it from a bad contact, and galre stops. Both runs of that
chip, with the unused pins held and with none held, give the source of
`designs/osc16v8`.

What reads cannot show: a loop that toggles too fast for a pin behind
it to swing at all. That pin reads as a level, and comes out as one.

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
     circuit predicts as many in a row as make 131072 steps (at least
     two, at most 128), in mixed Gray code order, so pin 1 rises often
     and in many places;
   - then walks chosen for what the circuit only guesses. A product
     term grows as far as nothing seen contradicts it, so it may lack a
     literal that the chip has: the chip was just never seen where that
     literal would make a difference. Where a term is the only one that
     is true and a walk showed what the latch does, the term was seen
     to decide. A pin that the term does not read, and at which it was
     seen to decide at one level only, may be such a missing literal:
     at the other level the latch was seen only where another term (the
     one that holds it) decides as well. A term that was seen to decide
     nowhere, by itself, is a guess as a whole: it stands for what the
     circuit was taken to do on its way through a step that moved
     several latches, and such a step does not show it. galre takes
     candidate walks through the learned circuit, runs the one that
     gets to most of these cases on the chip, and learns from it if the
     chip does otherwise, until no candidate gets to any more (at most
     64 walks).
     What is left is listed: "not seen, for no walk tried gets there".

   In a registered GAL16V8 or GAL20V8, pin 11 (13) enables the registers'
   outputs, and the array reads neither it nor pin 1. While it is high no
   register shows on its pin, and a clock edge that a latch sent another
   way would be lost. The sweeps have shown what the pin does by then, so
   galre holds it low for these walks (`designs/oelatch16v8`). An output
   that only that pin enables is a register too, with a constant input
   if it never changes.

   A pin that the sweeps took for a register may not follow the clock
   edges of a walk: a latch that is open while pin 1 is high looks like
   a register as long as the chip is only clocked. galre then takes the
   pin for a latch that reads pin 1 and starts again
   (`designs/clklatch22v10`).

   Where a clock edge leads may depend on a latch, and the latch on the
   way the chip came to the state: two ways to the same register values
   leave it at different levels. The sweeps, which take one path to each
   register state, then miss states, or the same clocks lead to two
   states. When a walk meets a state that the sweeps did not find, when
   a state's path does not lead back to it, or when the latches' circuit
   does not come out, galre explores again by the register values and
   the levels of the latch pins together (`designs/routelatch22v10`):
   - A state's observation ends with the levels of the pins that change
     without a clock, so two states with the same registers and another
     latch level are two states, each with its own path, table and
     sweeps.
   - The reads of an observation move the inputs, and so the latches.
     So the firmware follows power-up and every clocked step, in a path
     as in a sweep, with the same reads (`psig`): then a path leaves the
     latches as the sweep did, and leads to the state that the sweep
     saw.
   - Afterwards a state is the register values again. The first state
     found with them stands for it; the sweeps of the others say where
     its clock edges go with the latches at other levels.
3. The outputs that read differently at the same inputs, pin 1 level and
   register state are the latch pins, less those that are a function of
   these and the other latch pins. Every output, enable and register
   input is learned per context: a register state with the levels of the
   latch pins. A latch pin's excitation is learned as for latches alone,
   with pin 1 as an input: a step that changes one pin, or clocks the
   chip, says where the latches settle from the levels they had.

   One input change can set off a chain: a latch falls, then one that
   reads it, then a third (`designs/chain16v8`). The chip is seen only
   before and after, so such a step by itself proves nothing about any
   of them. galre runs the circuit learned so far through every step
   and adds only what a wrong one shows, the steps that change fewest
   latches first:
   - a latch that the circuit moves and the chip left alone holds, at
     the first such move only. What the circuit does after a wrong move
     is no guide, because the latches that read it follow it to levels
     the chip never had;
   - a latch that never gets to the chip's level is excited where the
     circuit comes to rest;
   - a step that comes out right only by the order in which two latches
     that chase each other are taken is not explained while one of
     their moves is a guess; that move is held. A race that is the
     chip's own stays;
   - of a step of which the circuit knows nothing, the fewest latches
     that, moved first, let the circuit do the rest are excited; failing
     that, the step is split into rounds, or all move together.

   Which of the latches of a step reacts to the input, and which to the
   others having moved, makes no difference to the walks, but it does to
   the equations: a latch taken to move with the others needs their
   terms as well as its own. If a latch then needs more product terms
   than its macrocell has, galre learns again with such latches moving
   after the others of a step (one term that reads them), and keeps
   that if it explains the walks and fits better.

   What is added this way gives way when a later walk shows otherwise,
   also when a later step does: if the circuit moves a latch at a point
   only because of such an addition, and the chip left it alone there,
   the addition is replaced by a hold, and the step that it was made
   for is explained again, one move earlier.
   The result reads the same as the chip wherever the chip comes to
   rest. Its equations need not be the original's: in the chain above
   the original holds each latch until R, and galre's hold it by the
   latch before, which is the same at rest.

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
   every step that holds it. A latch pin often reads the same as an
   input at rest (a P15 that follows P6), and the walks cannot tell
   which of the two a product term reads. So galre grows a second cover
   whose product terms drop other latches' pins before any other pin: a
   term then reads another latch only where something that was seen
   needs it. It takes that cover unless it has more product terms. From
   `designs/reglatch22v10`:

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

The same makes an edge detector learnable (`designs/edgelatch16v8`, after
a video digitiser's GAL): pin 16 is a copy of pin 4, and a latch is set
by `!P4 & P16`, which is true only for the gate delay in which pin 16
has not yet followed pin 4 down. The latch is a flip-flop clocked by the
falling edge of pin 4, in a chip whose registers pin 1 clocks. galre
writes the term as it is; a GAL has no other way to do this. A term that
is never true where the chip rests is named in the report and in a
comment in the source, because a copy behaves the same only if its
delays allow it.

On a real GAL16V8 the delays do not allow it for `designs/edgelatch16v8`
itself. The term is true for one output delay, the latch is back at the
array after one output delay, and the chip that was tried never took
the edge: 3,840 falling edges of pin 4, slow and fast, left pin 17 low,
and galre gave two constant outputs, which is what that chip does.
`designs/edge2latch16v8` has margin: pin 14 is a copy of pin 16, the
term reads pin 14 and is true for two delays, and a further term holds
the latch from the first. The same kind of chip takes the edge with it,
and galre gives from the chip what it gives from the simulator: a latch
set by a term that reads pin 16, and pins 14 and 16 as copies of pin 4.
Once the pins have settled, one delay reads the same as two. Written
like that, with one delay, and burned into another chip, the copy did
not take the edge.

So where a term is true only in passing and other outputs read the same
as the one it reads, galre writes them as a chain, the term's output
last (`P14 = P4; P16 = P14;`): the term is then true for as many delays
as there are copies, which is the most the original can have had. It
does so only if the chain still explains every walk, and the comment in
the source says which output was written to follow which. A chip burned
from that source took the edge, and galre gave the same source from it.
Whether the original has the outputs in that order does not show; a
copy of an edge detector still has to be tried on its board.

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
  is written as one, unless the design has other latches: then the walks
  with pin 1 show that it does not follow the clock edges.
- Registers that never reach a pin are not seen.
- Of state that changes without a clock, galre models the GAL22V10's
  asynchronous reset term and latches built from combinational feedback.
  It takes a design for latches when the power-up walk reads differently
  in another order. Tried in simulation on set/reset and transparent
  latches with their inputs on every pair of pins, and on set-only bits,
  the three counting orders (descending, bit-reversed, both) together
  show every latch that can leave its power-up value. No two of them do.
  In a counting order, though, a pin never rises alone: every faster pin
  falls with it (in the same write, with firmware that has no `ser=`; just
  before it, with one pin per write). A latch that holds only while
  another signal keeps still (`!P14 = !P8 # !P14 & !P13`, from a graphics
  card's GAL) may never hold then, and the chip reads as combinational. So
  the power-up state is also walked in Gray code order and in two mixed
  Gray code orders, and every other state once in a mixed one, from the
  vector the path left it at; these must read the same as the counting
  walk. A latch in a registered design that acts only in other states also
  shows when reading a new state changes it.

  A mixed Gray code changes one pin per vector like the usual one, but
  is walked as one half, one pin's change, the other half, and each half
  again so, with the pin that splits each part chosen by a hash (`mix`).
  The usual Gray code changes its lowest pin at every second vector and
  its highest once. In a mixed one, which pins change often differs from
  part to part, so most pins change about equally often, and a few
  values of `mix` cover them all. What a latch does when a pin changes
  depends on where the other pins and latches stand, so this shows much
  more of it per walk.

  A GAL20V8 is a GAL16V8 with four more inputs: the same macrocells and
  three modes, pin 13 the registers' enable in registered mode. galre
  takes its fuse layout as CUPL writes it for `g20v8s`, `g20v8ma` and
  `g20v8ms`.

  galre stops with exit code 3 before writing equations when:
  - the chip has state that no pin shows (a register whose output is
    never enabled): the same clocks then lead to different states;
  - a state is reached only with a latch at a level that no sequence of
    clocked vectors leaves it at, or a clock edge goes another way
    because of a latch and no registered pin shows it before the next
    edge;
  - a latch reacts only to several inputs changing at once;
  - a pin reads unsteadily, and not as an output that feeds back
    inverted would (see Outputs that do not settle): at other vectors
    in another walk order, or in a chip that holds state as well
    (`designs/oscstate16v8`);
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
  mix= start= skm= skv= avm= avv= quiet= chk= chkm= psig= pre= follow=
  dclk= ser= settle= off= on= vcc= gnd= supply=`
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
  each with the pins that differ. Pins left out of `mask` do not count,
  nor do pins outside `chkm`, if it is set.
  With `pre=1`, every vector's record is followed by the read of all
  pins just before its clock edge.
- `state`: the signature of the path state.

With `psig=1` (which needs probe vectors), power-up and every clocked
step, of the path, of a return path and of `trans`, are followed by the
same writes: the step's vector with the clock low, the signature, the
vector again, the signature again. The second signature is the state's:
`walk`, `trans` and `state` report it without reading again, and `trans`
with `chk=1` flags the vectors whose two reads of the vector differ.

Every clock step drives its vector with the clock pins low, then raises
them. With `follow=1`, the pins are read after the first write and every
read pin is driven at the level it read before the clock rises. With
`dclk=1`, the clock pins are driven directly by the CPU's PLD_* pins
instead of through the 1K, and only while the chip is powered; a clock
pin may not be a VCC or GND pin.

With `ser=1`, every write changes one pin, the lowest socket pin first: a
vector that differs from the last one in n pins takes n writes, and is
read after the last. Without it, a vector is written in three port writes.

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
| `osc.py` | Outputs that feed back inverted and do not settle |
| `reglatch.py` | Latches in a registered design, and pin 1 as an input |
| `rig.py` | `SimRig` (simulated chip), `BrutusRig` (board, jumper check), recording and replay, socket pin maps |
| `fwsim.py`, `fwstub/` | `fw/capture.c` built for the host, with a simulated GAL in the socket (`galmodel.c`: the GAL in C) |
| `capture.py` | Serial transport and capture format |
| `galsim.py` | Fuse-level GAL16V8/20V8/22V10 simulator |
| `bitsim.py` | The same, evaluated for all input vectors at once |
| `verify.py` | Rebuilt JED against the captures, and JED against JED |
| `csimcheck.py` | Test vectors from the rebuilt JED for CUPL's simulator, and its verdict |
| `designs/` | Test designs (CUPL source and compiled JEDs) |
| `tests/` | Test suite |

Run the tests from the repository root:

    python3 -m unittest discover -s galtools/tests

The CUPL round trip runs only when a CUPL compiler is found. The firmware
tests need a C compiler.
