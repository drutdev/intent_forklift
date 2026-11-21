# Continuous-curvature planning and pure-pursuit control

## Purpose and shared contract

The `geometric` method joins a smooth approach curve to a straight insertion segment. One speed profile and pure-pursuit tracker execute the combined path continuously. It supplies the active motion-planning and tracking stack.

The [shared runtime](DESIGN_SCAFFOLDING.md) owns sensors, EKF, timing, actuator delay, geometric clearance, recording, and physical scoring. This method consumes a `ControlContext` and produces a `MotionProposal` on a scheduled 50 Hz loop. Planning can take seconds. Deterministic mode pauses simulation during that work. The public run, replay, and batch commands exercise this implementation.

The required reference is assignment items 04-06 on p. 2 of [the brief](</home/amir/Downloads/The Last Two Meters - Intent Robotics.pdf>). Curve selection, the continuous join, lookahead, and withdrawal policy are engineering choices. The brief's p. 3 numbers remain tunable.

## Acquire the target

The supervisor holds the forklift while the EKF initializes from a pocket pair. A fresh estimate and positive uncertainty-adjusted clearance at the terminal pose permit the method to request an approach trajectory.

Invert the relative pallet estimate to obtain front-axle pose in pallet coordinates. Apply the axle-to-tip transform when calculating fork goals. The path remains in pallet coordinates as later observations correct the estimated vehicle pose.

## Construct the restricted CC path

The reference point is the front-axle midpoint in the stationary pallet frame.
The fork-tip midpoint is a derived point 1.58 m ahead. Its velocity is not
aligned with body heading during a turn. Clearance checks sweep all collision
pieces, including this overhang.

The kinematic model is

```text
dx/dt = v cos(psi)
dy/dt = v sin(psi)
dpsi/dt = v kappa
equivalent_rear_delta = -atan(wheelbase kappa)
```

`cc.py` implements forward `LSL`, `LSR`, `RSL`, and `RSR` CSC variants and
`SL` and `SR` SC variants. Here C means a continuous-curvature turn, not a
circular arc. L and R name its signed heading change. Zero-length turns and
straights are permitted degeneracies. A centered path can reduce to S.
Each path ends with the same straight insertion segment. This restricted
construction omits CCC, mixed-direction turns, and the full Reeds–Shepp set.
It has no completeness or global shortest-path guarantee.

The construction follows [Fraichard and Scheuer, sections III–IV](https://homepages.loria.fr/AScheuer/Old_W3/Articles/Fraichard_Scheuer_IEEETRA_04.pdf).
A saturated turn has a linear-curvature clothoid ramp, a constant-curvature
arc, and a clothoid ramp to zero. For a zero-curvature boundary and deflection
below `kappa_max^2 / sharpness_max`, equation 10 supplies the elementary
paired-clothoid turn. Its sharpness decreases so its endpoint remains on
the corresponding CC circle. Its peak curvature remains below the planning
limit. The implementation evaluates the Fresnel integrals with 12-point
Gaussian quadrature. It does not smooth a previously generated circular path.

The first C starts at curvature predicted from measured wheel feedback and
the pending command FIFO. For a nonzero boundary, a truncated initial ramp
replaces the zero-curvature ramp. Exact signed heading integration determines
the peak and optional arc. An initial ramp can cross zero when feedback has
the opposite sign. This boundary extension uses intrinsic coordinates instead
of the paper's zero-boundary CC-circle tangency construction. Incompatible
short deflections are rejected. Exactly zero turns collapse to zero length,
rather than the paper's nonzero straight CC-circle limit.

The planner searches 28 intermediate headings between -1.35 and +1.35 rad,
plus 17 headings spanning the initial heading and zero, with an extra
0.08 rad on each side. The initial heading is also included exactly. The
sharpness bounds are 0.15, 0.30, and 0.60 per square metre. For each CSC heading, exact lateral closure
determines a nonnegative middle straight. The resulting endpoint determines
the join location. SC similarly solves its initial straight from lateral
closure. The join must lie 0.10–1.20 m before the face. `join_distance` can
extend the outer bound. These are engineering search bounds, not assignment
requirements or a fixed mandatory alignment distance.

Position, heading, and curvature remain continuous at segment boundaries.
Sharpness can change discontinuously. The planning curvature limit is 85%
of the mechanical limit to leave steering headroom. The exact Ackermann map
checks both rear wheels. The complete path continues to the commanded depth,
including the shared allowance beyond the independently evaluated 0.60 m.

Samples retain every segment boundary at spacing no greater than 20 mm.
The shared guard refines the collision sweep to at most 5 mm of corner travel.
No endpoint is snapped to hide a failed connection. Geometric clearance and
the calibrated tracking preview must both pass before motion. The preview
also checks a full-depth straight continuation near the face with 6 mm
additional margin, and terminal tip lateral error below 6 mm and yaw below
0.008 rad. These sampled nominal checks do not prove physical tracking.

## Assign a feasible speed profile

Assign speed across the complete joined path. Convert curvature into both rear wheel angles. Each sample satisfies wheel angle limits and

```text
abs(v d(delta_i)/ds) <= 0.65 target_steering_rate_limit
```

Reserve 35% of target slew for tracking and inverse-lag compensation. Also
bound speed so `abs(kappa + v steering_response_s d(kappa)/ds)` remains
inside the mechanical curvature limit. That limit is the intersection of both
rear wheel angle bounds. The 0.92 s measured steering response remains in
the feedforward and full tracking preview.

Apply the 0.22 m/s approach cap and a 0.10 m/s cap on the straight segment. Decelerate smoothly before the join to meet that cap. A forward acceleration pass limits speed increases. A backward braking pass imposes zero speed at the final insertion endpoint.

The profile starts at measured speed and carries a shared positive speed across the join during a healthy run. Check speed and Ackermann steering continuity there. Initialize from the predicted command-application pose. Sweep a nominal tracking simulation through both segments using the calibrated target slew and servo response. The planning preview uses 100 ms integration steps and projects its simulated pose onto the path for progress. The live tracker propagates the released FIFO at up to 4 ms per step. Steering must settle sufficiently before the fork tips cross the face.

Score candidates by duration plus `12 s/m * max(0, 0.65 m - join_distance)`. This preference buys steering-settling distance without requiring a fixed join. Break ties by greater minimum clearance. Report clearance, tracking-preview, or restricted-geometry rejection when all candidates fail. A close misaligned start can trigger recovery after this rejection.

## Track the joined path

At each 50 Hz update, predict pose to the new command's application time through the pending FIFO. Find monotonic progress along the path near the previous progress index. A local search prevents jumps to a distant part of a curved path.

Use a 1.50 m base lookahead on bends and in reverse, plus 0.90 s times
absolute predicted speed. On the terminal straight, cap the base lookahead
at 0.80 m and reduce the extra fork-lever yaw damping gain from 2.0 to 0.5.
The shorter straight lookahead corrects lateral error sooner. The lower extra
yaw gain avoids overdamping that correction. These are tracking choices,
checked in the same complete-path preview before authorization. Extend the final tangent beyond the endpoint when computing steering during braking. Transform the lookahead point into the predicted body frame. Forward pure pursuit requests

```text
kappa_request = 2 y_lookahead / (x_lookahead^2 + y_lookahead^2)
```

On a bend, subtract the pursuit curvature at the reference pose and add the planned curvature plus its spatial derivative times speed and the steering time constant. Add signed yaw damping for the 1.58 m fork-tip lever arm. This feedforward term accounts for the measured 0.92 s steering response. The lookahead can cross the geometric join. Continue with the same trajectory clock, speed profile, and progress index through that join and the pallet face.

Use the time-parameterized speed profile at the command's predicted application time, with bounded progress feedback. The scheduled acceleration starts motion from rest. Remaining distance supplies a separate braking bound. Limit entry speed by the delayed braking reserve if the projected corridor or measured steering fails its check. Submit speed and curvature through the common supervisor.

A supervisor hold invalidates the trajectory's timing origin. Restart from a stopped state with a new timed profile so an elapsed reference cannot request a sudden catch-up maneuver.

The supervisor rechecks the remaining guide, tracking prediction, and delayed
stop each tick. With a fresh estimate, less than 25 mm of movement over six
seconds triggers a stop and a withdrawal request. The bounded wait permits
more detector corrections while stopped. Detection freshness, the three-second
sensing-loss abort, clearance checks, physical scoring, and the one-retry budget
remain active. Normal detector corrections update tracking without rebuilding
the trajectory on every frame.

This tracker is our implementation choice under p. 2, item 04.

## Authorize entry during motion

The approach curve establishes nominal alignment before the join. During the straight segment, the tracker corrects residual error while the forks remain outside the face. The shared guard checks each correction against the remaining corridor and delayed stopping motion.

The method starts its additional entry check when the estimated tip midpoint comes within 0.35 m of the face. It predicts 0.40 s under the candidate request, then sweeps a straight continuation at the predicted heading to the commanded depth. It requires both measured wheel angles to lie within 0.10 rad of straight. The supervisor checks the proposal and delayed stopping sweep every control tick. Forecast the fork envelopes through the remaining insertion using joint lateral and yaw uncertainty. Crossing requires a fresh accepted estimate, adequate predicted clearance, and sufficiently settled measured steering.

Until entry is permitted, the speed governor preserves enough room to stop outside the uncertainty-adjusted face. A healthy run obtains permission before braking becomes necessary and continues through the face. A failed clearance, freshness, or steering check causes deceleration and a checked stop. The method then holds, replans, or requests recovery according to the cause.

An entry decision records nominal clearance, yaw contribution, covariance allowance, tracking allowance, and stopping motion. A 600 mm straight traversal at 2 degrees yaw produces about 21 mm lateral travel. Independent lateral and yaw limits would miss the combined effect.

The geometric join and first fork-tip face crossing identify approach, alignment, and insertion progress. These events sequence the physical behavior required by p. 2, item 05. Continuous execution and the moving full-depth entry check are engineering choices under items 04-05.

## Insert and finish

Entry authorization extends the permitted motion along the existing straight segment. Continue its speed profile and brake toward the terminal depth. The evaluated minimum remains 0.60 m.

During travel, the shared guard checks the remaining pocket corridor and delayed stop. The method requests a final stop 4 mm before the commanded endpoint. The supervisor waits for standstill, an empty moving-command queue, and estimated depth and alignment before requesting assessment. The evaluator supplies the physical result under p. 2, item 06.

## Recover after an unsuccessful maneuver

Recovery begins with a stop. A sensing outage first permits a stationary reacquisition attempt. An approach that lacks correction room permits a retreat request after the supervisor authorizes recovery.

Plan a fixed 2.50 m straight reverse withdrawal. Check its swept clearance and a feasible joined forward continuation through insertion before requesting authorization. Empty surroundings leave the pallet as the relevant collision object.

If the forks are inside the pallet, withdrawal follows the verified pocket direction while its clearance remains justified. The withdrawal follows the current heading for 2.50 m. Its cruise speed cap is 0.40 m/s, with acceleration and final braking from the shared speed profile. A blocked straight withdrawal can end the attempt.

For pure pursuit in reverse, define the travel frame rotated by pi from the body frame. Compute travel curvature from its lookahead point, then negate it to obtain body curvature. This preserves `dpsi/dt = v kappa` with signed negative speed. Verify the sign using mirrored reverse-turn cases.

After retreat, stop, reacquire, and plan a new forward approach. The shared retry budget bounds repetition. Checking both retreat and continuation is our engineering choice under p. 2, item 05.

## Diagnostics and acceptance evidence

Proposal diagnostics record candidate rejection reasons, CC family, sharpness, join distance, curvature, both wheel angle profiles, estimated tracking errors, progress, entry margins, and recovery decisions. Record the join and face-crossing times with planned and measured speeds. Rerun shows the combined planned and measured paths on the shared timeline.

The method's implementation is accepted against the following scenarios.

- A centered ideal-sensor start reaches the evaluated depth with zero contact.
- A healthy run crosses the join and pallet face with measured speed above the standstill tolerance and stops at the final depth.
- Position, heading, curvature, and speed remain continuous at the join. The trajectory clock and progress index advance across it.
- Mirrored offsets and yaw errors produce mirrored corrections.
- A rotated pallet preserves pose, clearance, and depth conventions.
- Steering-rate and braking demands remain inside the declared bounds.
- A late sensing outage causes a checked stop before the available margin is exhausted.
- A close misaligned start rejects its forward candidates, retreats, stops, and retries when a tested continuation fits.
- A rejected withdrawal or exhausted retry budget ends in abort.

The shared batch runner supplies item 07 evidence on p. 2. The common dropout study supplies the initial item 08 experiment.

## Limits

The finite heading and sharpness search can reject starts for which another route exists. The planner also rejects candidate profiles longer than its 120 s preview budget. Its model-based preview and 5 mm spatial sweep are sampled checks. Pure pursuit needs lookahead tuning and gives approximate tracking under delay. The join distance must provide enough travel for steering to settle and alignment errors to reduce. Fixed height and a known pallet template simplify collision checks.

Implementation reuses the shared geometry, Ackermann mapping, sensor messages, and evaluator. Its outputs include successful demonstration trajectories with actual executed commands and method provenance. The physical evaluator confirms successful engagement independently of the tracker.

## CC runtime validation

The final study uses the public runtime with explicit matched poses, disturbance
seeds, physical criteria, and a 90 s episode budget. The study used a fixed 3.50 m withdrawal with a 0.40 m/s cruise cap. Its
checked stop, authorization, braking, reacquisition, and single-retry limit remain
in the shared runtime. The six-second movement-stall allowance is a bounded
stationary correction opportunity. A newly activated forward path resets the
movement anchor, so acquisition time cannot cause a false immediate stall. It does not extend the stale-detection limit.

[CC outcomes](recordings/cc-final-v2/results.json) and
[matched quintic outcomes](recordings/cc-baseline-fixed/results.json) retain
complete event traces, estimator replay counts, copied simulation states, and
Rerun streams. Each suite saves its resolved source under `source/`.
The quintic reference starts at `0839e89`. Three explicit edits match the fixed
3.50 m withdrawal, the 0.40 m/s reverse cap, and the six-second stall allowance.
`tools/cc_evidence.py --match-recovery` reproduces those edits in the reference
without changing the supplied file. Connector construction, join selection,
speed reserves, preview acceptance, and straight tracking differ between the
two versions. Their results measure the combined change.

The CC version completes **16/18** cases with zero contact. The matched
quintic reference also completes **16/18**. CC succeeds on the mirrored nominal
case where the reference exhausts recovery. The reference succeeds on randomized
seed 3 where CC exhausts recovery. This study shows no aggregate reliability
improvement over the reference with the updated recovery policy.

| Case | CC result | CC time (s) | Minimum depth (mm) | Matched quintic result |
| --- | --- | ---: | ---: | --- |
| `center-ideal` | success | 17.64 | 625.4 | success |
| `center-nominal` | success | 23.68 | 625.0 | success |
| `case2-ideal` | success | 50.24 | 625.1 | success |
| `case6-ideal` | success | 50.24 | 625.1 | success |
| `case2-nominal` | success | 55.30 | 626.5 | success |
| `case6-nominal` | success | 57.12 | 620.4 | retry_exhaustion |
| `rotated-nominal` | success | 55.24 | 625.7 | success |
| `close-nominal` | success | 45.44 | 626.1 | success |
| `random-3` | retry_exhaustion | 54.94 | -18.8 | success |
| `delay` | success | 23.70 | 623.9 | success |
| `stale` | uncertainty_abort | 17.00 | -50.8 | uncertainty_abort |

The prior randomized seeds 0–5 finish **5/6** with CC and **6/6** with the
matched reference. CC seed 2 needs 65.42 s; its success is outside the default
60 s budget. At 60 s, CC's six-case count is 4/6. The reference finishes all
six within 60 s. The wider 18-case counts within 60 s are 15/18 and 16/18.
These are development cases, not an independent validation envelope.

All 16 CC successes end settled with both blades inside their pockets.
Their minimum evaluated depth is 620.4 mm. Across all 18 episodes, peak measured
speed is 0.3993 m/s and peak rear-wheel rate is 20.98 degrees/s, below the
0.45 m/s and 30 degrees/s physical scoring bounds. The 0.60 m success depth,
contact latch, covariance margins, and retry budget are unchanged.

The centered nominal run crosses the true join at 0.1031 m/s and the true face
at 0.0998 m/s. Its forward motion has no intermediate standstill. Both hard
ideal cases cross the estimated join at 0.1047 m/s and the face at 0.0998 m/s,
with no intermediate forward standstill. Their recovery stop is separate.
[Crossing measurements](recordings/cc-final-v2/measurements.json) and
[axle and fork-tip plots](recordings/cc-final-v2/paths.png) distinguish the
reference points and estimated versus true events.

All 18 recordings are complete, with 38,615 matching estimator states in replay
and [18 verified Rerun streams](recordings/cc-final-v2/rerun-validation.json).
The [31 component and integration tests](recordings/cc-final-v2/component-tests.log)
and [three original XML probes](recordings/cc-final-v2/xml-probes.log) pass.
The tests include the small-deflection CC circle, nonzero initial curvature,
mirrors, join closure, both wheel maps, lag-compensation headroom, acceleration,
stopped reverse authorization, freshness, contact latching, and the delayed
planning launch regression.
[Current success video](recordings/cc-final-v2/case2-nominal/scene.mp4) and
[current seed 3 failure video](recordings/cc-final-v2/random-3/scene.mp4) use the
recorded playback states. Their sampled frames and traces were reviewed.

Computation remains a limitation. The centered nominal method has a 4.02 ms
median, 11.14 ms p95, and 532 ms maximum. The hard nominal case 2 has a 3.80 ms
median, 12.48 ms p95, and 4.57 s maximum. The largest method call across the
suite takes 6.47 s. These are wall costs during concurrent deterministic batches;
simulation pauses during computation. Baseline workers were suspended as a
group to limit memory pressure, so their wall-time samples are not performance
comparisons. Physical episode times and seeded outcomes remain comparable.

The earlier [realtime trial](recordings/cc-realtime-final/outcome.json) fails
with `expired_proposal` at 12.14 s under concurrent host load. It records 247
late results, 404 expired-proposal ticks, and 69 missed control deadlines. The
forklift remains essentially at its start and has zero contact. This is not a
validated realtime engagement controller. Its complete recording replays 608
estimates and passes Rerun verification.

The current 60 s CLI default remains useful for short demonstrations. The
90 s study budget includes acquisition, recovery, safety holds, and final
settling. Success after 60 s does not establish success at the default budget.
`tools/cc_report.py` records that distinction per case.

The reconstructed plots show measured front-axle paths and derived fork-tip
midpoint paths in the stationary pallet frame. The measurement report separates
true geometric crossings from controller events based on estimated pose.
An estimator correction can report a face crossing while the vehicle is still.
The physical evaluator supplies contact, pocket membership, depth, and settling.

The nominal seed 3 failure remains a tracking and uncertainty limit. In the
recorded pre-anchor-fix run, straight tracking develops about 8 mm of lateral
error and -0.0072 rad of yaw error. At entry, 11.01 mm nominal clearance cannot
cover 11.73 mm of covariance allowance. The method correctly stops. The next
withdrawal request exhausts the single retry. The independent video review
agrees with shallow approach followed by standstill; it cannot resolve subtle
contact visually. Physical scoring records zero contact.

A paired preview experiment increased the nominal entry-preview margin from
6 to 15 mm with the same fixed recovery policy. It rejected both tested hard
starts during planning, including the formerly successful nominal case 2.
That change was not adopted. These rejections concern the finite searched
family and do not establish geometric impossibility. [Stricter-preview study](recordings/cc-preview-15mm/results.json)

Two small paired studies retain failed alternatives. Increasing the nominal
preview entry margin from 6 mm to 15 mm rejects the hard nominal case 2 and
random seed 3 during planning within the searched family. It is not adopted.
The fixed-recovery pair with a 2.5 s stall allowance succeeds on random seed 2
and the rotated nominal case. A 6 s allowance also succeeds on both, but delays
seed 2 from 60.64 s to 65.42 s. That pair provides no evidence that the longer
wait improves reliability. The six-second choice is a bounded conservative
wait and remains a tuning limitation.
[Strict-preview study](recordings/cc-preview-15mm/results.json) and
[stall-wait pair](recordings/cc-stall-2p5/results.json) preserve the resolved
sources and disturbances. A planning rejection does not prove that every
forward route is impossible.

Earlier low-speed recovery and intermediate tracking trials remain under
`recordings/cc-validation-v4`, `recordings/cc-validation-v5`, and
`recordings/cc-validation-v6`. These are development archives with different
recovery policies. They are not the final matched comparison.

## Shorter fixed recovery

The active fixed withdrawal is now 2.50 m at a 0.40 m/s cruise cap. Frozen first-recovery contexts from ten previously successful recovery cases were checked at 0.65, 1.00, 1.25, 1.50, 2.00, 2.50, 3.00, and 3.50 m. The hard mirrored and rotated starts rejected the forward continuation through 2.00 m. Every checked context admitted a 2.50 m withdrawal and forward continuation. This is the first successful sampled distance across those contexts.

The matched 18-case widened-pallet runtime suite retains 17/18 successes, all within 60 s, with zero contact. Both policies abort on the forced sensing outage. Nominal case 2 improves from 50.98 to 49.44 s. The close nominal recovery improves from 43.74 to 37.18 s. Randomized seed 3 improves from 51.36 to 44.94 s.

The 34-test suite passes. All 18 new recordings replay exactly and pass Rerun verification. The nominal case 2 playback was reviewed. The baseline comparison helper matches the current 2.50 m policy, including references saved with the earlier 3.50 m policy. [Feasibility sweep](recordings/cc-short-retreat-study/feasibility.json) · [Matched results](recordings/cc-short-retreat-study/comparison.json) · [Nominal recovery video](recordings/cc-short-retreat-study/fixed-2p5m/case2-nominal/scene.mp4)

## Heading-grid reduction

The broad grid was reduced from 271 to 28 samples and the local grid from 161 to 17 samples. Each interval is ten times wider. The endpoint headings and exact initial heading remain available. The three sharpness choices, collision checks, tracking preview, and speed constraints are unchanged.

A matched 18-case study on the widened pallet completed 17 engagements with each grid, all within 60 s. Both grids aborted on the forced sensing outage. All 36 episodes had zero pallet contact and identical success and retry counts per case. The coarser grid changes selected paths. The close ideal recovery increased from 41.72 to 42.08 s, while nominal case 2 decreased from 52.32 to 50.98 s. These results cover the tested cases.

An interleaved five-repeat planning benchmark measured median forward-planner costs of 161.8 to 63.4 ms for the centered feasible connection and 280.1 to 71.2 ms for a feasible diagonal connection. A hard direct connection rejected by both versions decreased from 139.6 to 15.2 ms. Collision checks and tracking preview still contribute to planning cost. The benchmark uses exact synthetic estimates to isolate planning computation; it supplies no engagement success-rate evidence.

All 34 component and integration tests pass. Every estimator trace replays exactly, and all 36 Rerun streams verify. A separate centered nominal realtime seed 8 run completed in 23.38 s, with 627.4 mm settled insertion and zero contact. Its recording also passes replay and Rerun verification. The per-case records and both source snapshots preserve the comparison. [Matched outcomes](recordings/cc-heading-grid-study/comparison.json) · [Outcome plot](recordings/cc-heading-grid-study/comparison.png) · [Planning benchmark](recordings/cc-heading-grid-study/planning-benchmark.json)

## Realtime proposal validity correction

The tracker still updates at 50 Hz. Each CC proposal now has a bounded 100 ms validity window, increased from 40 ms. Worker turnaround, IPC polling, supervisor computation, and host scheduling share this window. The runtime checks wall-clock expiry before and after supervision. Every supervisor tick rechecks sensing freshness, remaining guide geometry, motion limits, and the delayed stopping sweep. A missing replacement eventually expires and requests braking.

The earlier widened-pallet realtime seed 8 run expired proposals during 45 moving ticks. Repeated braking and stopped trajectory-clock restarts left only 319.0 mm insertion at the 35 s limit. The method computation median was 3.05 ms. Several results with approximately 3 ms computation arrived one simulation tick later but had already lost their remaining wall-clock validity.

With 100 ms validity, two recorded public-entry-point realtime runs completed in 24.26 s and 23.46 s. They reached 626.1 and 627.2 mm minimum true fork depth, settled inside both pockets, with zero contact. Expired ticks fell from 68 to 10 and 7. The first corrected trial expired during three moving ticks. In the repeat, all seven expired ticks occurred during initial planning before motion. The maximum planning calls still took about 183 ms and were rejected as late. The trials recorded 51 and 14 missed control deadlines, so completion does not establish a hard realtime scheduling guarantee.

The regression test checks that a CC proposal can survive 60 ms of turnaround and that the supervisor rejects it at exactly 100 ms. The full 34-test suite passes, including slow-worker expiry, stale sensing, contact latching, braking, and recovery. Both corrected recordings pass estimator replay and Rerun validation. The repeat playback was reviewed.

```sh
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pallet_engagement run --method geometric --mode realtime --duration 35 --seed 8 --record recordings/my-cc-realtime
```

Use a fresh recording directory. [Before and after measurements](recordings/cc-realtime-deadline-study/results.json) · [Measured speed and expiry plot](recordings/cc-realtime-deadline-study/comparison.png) · [Repeat playback video](recordings/cc-realtime-deadline-100ms-repeat/scene.mp4)

## Historical quintic results

The sections below describe the controller at `0839e89`, before the CC connector.
Their outcomes are retained as historical evidence, not current CC results.

### Correction after randomized failures

The first implementation failed all three nominal randomized starts. This failed the intended engagement outcome and required further controller work.

The corrected controller accounts for the fork-tip lever arm in its yaw feedback and uses a curvature derivative to compensate steering lag. Its planning preview projects the simulated pose onto the guide for progress. If forward planning fails with nonzero estimate covariance, it holds for two more seconds of detections before choosing a retreat. It retries failed planning once per second for a bounded eight seconds. This avoids committing to an unnecessarily long retreat from an early uncertain estimate. The withdrawal family now also tests 0.30 m.

The same scenario seeds, nominal disturbances, 60 s duration, clearance margins, retry budget, and physical success criteria produced these results.

| Randomized seed | Time (s) | Minimum true depth (mm) | Retries | Pallet contact |
| --- | ---: | ---: | ---: | --- |
| 0 | 30.04 | 624.6 | 0 | Zero |
| 1 | 53.54 | 618.9 | 1 | Zero |
| 2 | 50.32 | 617.3 | 1 | Zero |

All three ended settled with both blades inside their pockets. The original centered nominal seed 8 and both mirrored rotated ideal starts also succeeded. The close ideal recovery succeeded with one retreat. These three corrected randomized runs are development cases used during tuning. Three additional randomized seeds 3–5 completed twice. Seed 3 exhausted its retry and stopped 34.8 mm outside the face with zero contact. The combined result is 5/6 for these six starts. General reliability remains unestablished. [Additional outcomes](recordings/geometric-repair-additional.json) [Seed 0 outcome](recordings/geometric-repair-0/outcome.json) · [Seed 1 outcome](recordings/geometric-repair-v2-1/outcome.json) · [Seed 2 outcome](recordings/geometric-repair-v2-2/outcome.json)

The corrected runs passed 42 component and integration tests, all three original probes, estimator replay, and Rerun validation. The seed 1 playback shows the withdrawal, reacquisition, forward approach, and insertion. [Corrected crossing measurements](recordings/geometric-repair-metrics.json) · [Corrected recovery video](recordings/geometric-repair-v2-1/scene.mp4)

The original measurements below describe the earlier controller and retain the failed batch for comparison.

### Measured runs

Run these commands with the repository environment active. Each recording path must be new.

```sh
python -m pallet_engagement run --method geometric --ideal
python -m pallet_engagement run --method geometric --seed 8 --record recordings/my-success
python -m pallet_engagement run --method geometric --ideal --pose 3.2 .08 .04 .47
python -m pallet_engagement run --method geometric --ideal --pose 3.2 -.08 -.04 -.47
python -m pallet_engagement run --method geometric --ideal --pose .55 .06 0 .47
python -m pallet_engagement run --method geometric --seed 8 --delay-uncertainty-ms 40
python -m pallet_engagement evaluate --methods geometric --suite comparison --seeds 0:3 --outages 0 5
python -m pallet_engagement run --method geometric --seed 8 --outage-index 70 --outage-count 24 --record recordings/my-outage
python -m pallet_engagement run --method geometric --seed 8 --outage-index 70 --outage-count 24 --acquire-timeout 6 --record recordings/my-retry
python tools/geometric_evidence.py recordings/my-success recordings/my-outage recordings/my-retry
```

The following deterministic runs used the shared physical evaluator. Depth is the shallower fork tip. Both blades occupied their pockets on every success. All runs below ended settled with zero pallet contact. This tests assignment items 01 and 04–07 on p. 2 and item 09 on p. 3. The p. 3 suggested limits remain tunable.

| Start and sensing | Episode time (s) | Depth (mm) | Result |
| --- | ---: | ---: | --- |
| Centered, ideal | 21.40 | 626.5 | Success |
| Centered, nominal seed 8 | 23.62 | 627.8 | Success, continuous |
| Centered, nominal seed 7 | 24.74 | 627.1 | Success, one safety hold during insertion |
| Mirrored offsets and yaw, rotated pallets, ideal | 26.96 each | 624.7 each | Two successes, opposite terminal yaw errors of 0.32° |
| Close offset, rotated pallet, ideal | 41.70 | 625.0 | One authorized 1.25 m retreat |
| Centered, nominal seed 8, ±40 ms delay variation | 23.62 | 626.6 | Success |
| Seed 8, 24 missed captures from index 70 | 17.00 | −47.3 | Uncertainty abort before the face |
| Same outage, 6 s reacquisition timeout | 54.88 | 628.0 | Success after one retreat |

At seed 8 the independent planar reconstruction put the join at 8.62 s and the first outer tip at the face at 16.62 s. Measured axle speed was 0.103 and 0.100 m/s respectively. The planner requested 0.100 m/s at both estimated events. Every 20 ms feedback sample between launch and final braking stayed above the 0.010 m/s standstill threshold. The peak estimated axle cross-track error was 15.3 mm. The smallest reported stopping margin was 1.11 mm. The evaluator measured final lateral error of −1.95 mm and yaw error of −0.032°. [Crossing and tracking measurements](recordings/geometric-metrics.json)

The close start received reverse authorization at 0.20 s and entered stationary reacquisition at 15.20 s. Forward motion crossed the join at 0.069 m/s and the face at 0.100 m/s. Peak measured wheel steering rate was 19.7°/s against the 30°/s scoring limit. The shared physical probes checked the contact latch, including the deliberately misaligned 40 mm case. [Physical outcomes](recordings/geometric-cases.json)

Geometric completed the centered nominal seeds 7 and 8 in 24.74 and 23.62 s. These are simulated episode durations. Geometric method computation in the recorded seed 8 run took 5.55 ms median and 17.33 ms at p95, with a 204 ms maximum. The recorded control-loop p95 was 55.4 ms under concurrent host load. Deterministic mode pauses simulation during that work. [Geometric seed 8](recordings/geometric-success/outcome.json)

In one unrecorded realtime seed 8 run, the evaluator measured 627.5 mm of settled insertion after 33.02 s. The worker returned 25 late results. The runtime counted 29 expired proposal ticks and four missed control deadlines. The 50 Hz rate is a scheduling target. [Realtime outcome](recordings/geometric-realtime.json)

Across three randomized starts with nominal sensing, seed 0 timed out after one retreat. Seeds 1 and 2 failed planning. With five scheduled missing captures from index 5, seed 0 completed and seeds 1 and 2 again failed planning. All six runs avoided contact. The finite curve and recovery families, acquisition pose, and changing uncertainty affect this small sample. It does not establish the suggested 2–4 m operating envelope. [Seeded outcomes](recordings/geometric-comparison.json) · [Outcome counts](recordings/geometric-batch.png)

### Failure log and robustness study

1. **Noisy approach at seed 7.** A 0.43 m base lookahead produced steering corrections large enough to lose the projected insertion corridor. The first controller stayed outside the face at −45.9 mm. A matched lookahead sweep on that tracker timed out at 0.65 m and completed at 1.00 and 1.50 m. The final 1.50 m tracker with curvature preview reached 627.1 mm. [Speed, depth, and entry margin](recordings/geometric-lookahead.png)
2. **Steering lag after retreat.** The first controller backed up, then stalled 38.1 mm before the face. Its straight-segment yaw error persisted after the planned bend ended. Subtracting nominal pursuit and previewing curve curvature by one servo time constant, together with the longer lookahead, let the same close start settle at 625.0 mm. The paired trace measures the combined change. [Retreat and continuation](recordings/geometric-recovery-tracking.png)
3. **Detection outage at the entry boundary.** On seed 8, 24 missing captures from index 70 began 261 mm from the face. The 3 s timeout produced a checked stop 47.3 mm outside. With the same samples and a 6 s timeout the forklift stopped, regained detections, withdrew, and completed. At index 80 the same outage began later. The default timeout ended with 14.9 mm of insertion and zero contact. The extended timeout again allowed one retry and completion. This paired timing experiment addresses assignment p. 2, item 08. The changed timeout also extends the age limit for guarded travel outside the face. [Outage study](recordings/geometric-outage-study.png)

The success and failure archives contain the 3D Rerun view, estimator replay, command history, and playback poses. The videos show the approach and final positions. The evaluator measures contact and pocket membership at physics rate. The hardware note covers first-week commissioning for the p. 4 question. [Success video](recordings/geometric-success/scene.mp4) · [Failure video](recordings/geometric-failure/scene.mp4) · [Hardware commissioning](REAL_FORKLIFT_FIRST_WEEK.md)
