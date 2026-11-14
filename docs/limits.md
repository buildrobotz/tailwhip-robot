# Limits and gotchas

## What it cannot do

- **Climb anything.** 65 mm wheels and feet that stick out 54 mm in front: door sills, cables and thick rugs stop
  it. It is a desk and hard-floor robot.
- **Lean far forward.** The feet touch the floor at 18 deg. That is plenty for balancing and slow driving, but it
  caps how hard it can brake from a forward run: brake too hard and it ends up on its feet (then gets up again).
- **Survive every shove with the motors off.** With power off or the motors cut, the tail catches backward falls
  up to about a 4 rad/s shove and the feet catch forward falls up to about 6 rad/s (`sim/test_falls.py`). Harder
  shoves need the wheels to help, which only works while the policy is running.
- **Sit unpowered.** An unpowered SG-90 can be back-driven, so if the battery dies while it stands, it slumps onto
  its back. Don't leave it standing on a table edge.
- **Take a hard forward shove well (yet).** In simulation the current policy catches backward shoves and kicks
  well, but a hard forward shove sometimes makes it drive forward and roll over its own feet. Retraining with
  more forward shoves is the obvious next step; the Simulation page has the numbers.
- **Know which way the wheels turn.** The IR encoders count edges but can't tell direction; the firmware takes the
  sign from the command. When a wheel is pushed backwards against its command, the reading is wrong. The policy is
  trained with that flaw, the PID baseline just lives with it.

## What will probably need fixing

Nobody has built one yet. These are the places a first build is most likely to need work, roughly in order:

1. **TT motor mounting.** The hole positions in the CAD are estimates from drawings. Measure your motor and edit
   `TT_HOLES` before printing. TT motors also differ: some have one mounting hole, some a tab at the can end.
2. **Tail servo torque.** Rearing up from the tail rest needs about 0.06 N m, and a clone SG-90 gives about
   0.13 N m: 2x margin on paper. If yours can't lift it, fit the MG90S (same size, metal gears, more torque).
3. **Tail landings.** A fall onto the tail can put roughly the servo's stall torque through its plastic gears for a
   few tens of milliseconds, even with the leaf spring. If the gears strip, use the MG90S.
4. **Backlash and dead zone.** TT gearboxes have several degrees of slop and need a fair push before they move.
   The simulator randomises both over a wide range, but if yours is worse, the PID will rock back and forth. Measure
   it with `host/bench.py` and retrain.
5. **The bolt-clamped motor stack.** Two M3 x 70 bolts hold both motors to the frame. If it flexes, add a zip tie
   round each can (slots are printed in).
6. **Mass.** The design comes out at about 394 g against the 250-270 g first planned. The simulator and every
   check use 394 g, but weigh your parts: if the cells or wheels are heavier, update `cad/tailwhip.py`.

## Battery safety: read this before you charge anything

The pack is two **unprotected** 18650 lithium-ion cells in series (8.4 V full). They can deliver tens of amps
into a short circuit and can catch fire if over-charged, over-discharged or punctured.

- **Use known cells** from a reputable brand (Samsung, LG, Sony/Murata, Molicel), bought from a reputable seller.
  Never use "UltraFire"-style cells with impossible capacities.
- **Always use the 2S protection board (BMS)** and the polyfuse. They are not optional.
- **Charge with a proper 2S Li-ion charger** (8.4 V, constant current then constant voltage), never above
  8.4 V, and not unattended. Both cells must be at the same voltage when you put them in.
- **Never drain below 6.0 V** (3.0 V per cell). The firmware warns at 6.6 V; stop then.
- **Check the wiring with a meter before the first power-up**, as the Electronics page says. A reversed cell in the
  holder is the classic mistake.
- If a cell is dented, hot, or swollen, stop using it.

## Simulation is not reality

Every performance number on these pages comes from the MuJoCo model. The model is deliberately pessimistic
(slop, delay, noise, weak motors, random masses), but it is still a model, built from estimated part numbers.
Expect the first real run to look worse than the simulator, and expect to close the gap by measuring and
retraining, as milestone 4 describes. That loop is the point of the project.
