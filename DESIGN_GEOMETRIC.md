# Geometric planning and pure-pursuit control

## Purpose and shared contract

The `geometric` method joins a smooth approach curve to a straight insertion segment. One speed profile and pure-pursuit tracker execute the combined path continuously. It supplies the active motion-planning and tracking stack.

The [shared runtime](DESIGN_SCAFFOLDING.md) owns sensors, EKF, timing, actuator delay, geometric clearance, recording, and physical scoring. This method consumes a `ControlContext` and produces a `MotionProposal` at 50 Hz. The public run, replay, and batch commands exercise this implementation.

The required reference is assignment items 04-06 on p. 2 of [the brief](</home/amir/Downloads/The Last Two Meters - Intent Robotics.pdf>). Curve selection, the continuous join, lookahead, and retreat search are engineering choices. The brief's p. 3 numbers remain tunable.

## Acquire the target

The supervisor holds the forklift while the EKF initializes from a pocket pair. A fresh estimate and positive uncertainty-adjusted clearance at the terminal pose permit the method to request an approach trajectory.

Invert the relative pallet estimate to obtain front-axle pose in pallet coordinates. Apply the axle-to-tip transform when calculating fork goals. The path remains in pallet coordinates as later observations correct the estimated vehicle pose.

## Plan the two connected segments

Choose a join pose with the fork-tip midpoint on the pocket centerline, parallel to the insertion axis, and 0.80 m before the face. This pose connects the approach curve to the straight insertion segment. The join distance is an initial engineering choice.

The straight segment continues through the face to the commanded terminal depth. Its endpoint includes the shared depth allowance beyond the 0.60 m evaluated minimum. Plan and validate both segments as one candidate before motion begins.

For endpoint tangent lengths `a` and `b`, generate a quintic Hermite curve `r(u)` for `0 <= u <= 1`. The start constraints are

```text
r(0) = current axle position
r'(0) = a heading_vector
r''(0) = a^2 current_curvature left_normal
```

The endpoint constraints are

```text
r(1) = join axle position
r'(1) = b insertion_axis
r''(1) = 0
```

The six vector constraints determine the quintic coefficients. A small configured grid of tangent lengths supplies candidate paths. The start curvature comes from measured steering. Path curvature is

```text
kappa(u) = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
```

At the join, both segments share position, heading, and zero curvature. Resample the combined path by arc length and retain the join sample once. The tracker uses one progress coordinate through the join and pallet-face crossing.

Reject cusps, loops, excessive steering, and pallet intersections. Adaptive samples bound both translation and rotation of each collision piece between checks. Include uncertainty and tracking allowance from the shared guard.

The planner's geometric sweep uses the estimated pallet transform and fixed shape templates. The independent evaluator later checks the full simulated contacts. This division answers p. 2, item 04 and p. 3, item 09.

## Assign a feasible speed profile

Assign speed across the complete joined path. Convert curvature into both rear wheel angles. Each sample satisfies wheel angle limits and

```text
abs(v d(delta_i)/ds) <= steering_rate_limit
```

Apply the 0.22 m/s approach cap and a 0.10 m/s cap on the straight segment. Decelerate smoothly before the join to meet that cap. A forward acceleration pass limits speed increases. A backward braking pass imposes zero speed at the final insertion endpoint.

The profile starts at measured speed and carries a shared positive speed across the join during a healthy run. Check speed and Ackermann steering continuity there. Initialize from the predicted command-application pose. Sweep a nominal tracking simulation through both segments using the calibrated target slew and servo response. The planning preview uses 100 ms integration steps and projects its simulated pose onto the path for progress. The live tracker propagates the released FIFO at up to 4 ms per step. Steering must settle sufficiently before the fork tips cross the face.

Choose the shortest-duration feasible candidate, breaking ties by greater minimum clearance. Return a limiting constraint when all candidates fail. A close misaligned start can trigger recovery after this rejection.

## Track the joined path

At each 50 Hz update, predict pose to the new command's application time through the pending FIFO. Find monotonic progress along the path near the previous progress index. A local search prevents jumps to a distant part of a curved path.

Use a 1.50 m base lookahead plus 0.90 s times the absolute predicted speed. Extend the final tangent beyond the endpoint when computing steering during braking. Transform the lookahead point into the predicted body frame. Forward pure pursuit requests

```text
kappa_request = 2 y_lookahead / (x_lookahead^2 + y_lookahead^2)
```

On a bend, subtract the pursuit curvature at the reference pose and add the planned curvature plus its spatial derivative times speed and the steering time constant. Add signed yaw damping for the 1.58 m fork-tip lever arm. This feedforward term accounts for the measured 0.92 s steering response. The lookahead can cross the geometric join. Continue with the same trajectory clock, speed profile, and progress index through that join and the pallet face.

Use the time-parameterized speed profile at the command's predicted application time, with bounded progress feedback. The scheduled acceleration starts motion from rest. Remaining distance supplies a separate braking bound. Limit entry speed by the delayed braking reserve if the projected corridor or measured steering fails its check. Submit speed and curvature through the common supervisor.

A supervisor hold invalidates the trajectory's timing origin. Restart from a stopped state with a new timed profile so an elapsed reference cannot request a sudden catch-up maneuver.

The supervisor rechecks the remaining guide, tracking prediction, and delayed stop each tick. A fresh estimate and less than 25 mm of movement over 2.5 s trigger a stop and a search for a withdrawal. Normal detector corrections update tracking without rebuilding the trajectory on every frame.

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

Search straight reverse distances of increasing length. For each endpoint, check the withdrawal sweep and a feasible joined forward path through insertion. Select the shortest tested retreat that passes. Empty surroundings leave the pallet as the relevant collision object.

If the forks are inside the pallet, withdrawal follows the verified pocket direction while its clearance remains justified. The implementation tests straight withdrawals of 0.30, 0.65, 1.25, 2.00, and 2.75 m along the current heading. A blocked straight withdrawal can end the attempt.

For pure pursuit in reverse, define the travel frame rotated by pi from the body frame. Compute travel curvature from its lookahead point, then negate it to obtain body curvature. This preserves `dpsi/dt = v kappa` with signed negative speed. Verify the sign using mirrored reverse-turn cases.

After retreat, stop, reacquire, and plan a new forward approach. The shared retry budget bounds repetition. Checking both retreat and continuation is our engineering choice under p. 2, item 05.

## Diagnostics and acceptance evidence

Proposal diagnostics record candidate rejection reasons, tangent lengths, curvature, both wheel angle profiles, estimated tracking errors, progress, entry margins, and recovery decisions. Record the join and face-crossing times with planned and measured speeds. Rerun shows the combined planned and measured paths on the shared timeline.

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

The finite curve family can reject starts for which another route exists. The planner also rejects candidate profiles longer than its 120 s preview budget. Its model-based preview and 5 mm spatial sweep are sampled checks. Pure pursuit needs lookahead tuning and gives approximate tracking under delay. The join distance must provide enough travel for steering to settle and alignment errors to reduce. Fixed height and a known pallet template simplify collision checks.

Implementation reuses the shared geometry, Ackermann mapping, sensor messages, and evaluator. Its outputs include successful demonstration trajectories with actual executed commands and method provenance. The physical evaluator confirms successful engagement independently of the tracker.

## Correction after randomized failures

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

## Measured runs

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

## Failure log and robustness study

1. **Noisy approach at seed 7.** A 0.43 m base lookahead produced steering corrections large enough to lose the projected insertion corridor. The first controller stayed outside the face at −45.9 mm. A matched lookahead sweep on that tracker timed out at 0.65 m and completed at 1.00 and 1.50 m. The final 1.50 m tracker with curvature preview reached 627.1 mm. [Speed, depth, and entry margin](recordings/geometric-lookahead.png)
2. **Steering lag after retreat.** The first controller backed up, then stalled 38.1 mm before the face. Its straight-segment yaw error persisted after the planned bend ended. Subtracting nominal pursuit and previewing curve curvature by one servo time constant, together with the longer lookahead, let the same close start settle at 625.0 mm. The paired trace measures the combined change. [Retreat and continuation](recordings/geometric-recovery-tracking.png)
3. **Detection outage at the entry boundary.** On seed 8, 24 missing captures from index 70 began 261 mm from the face. The 3 s timeout produced a checked stop 47.3 mm outside. With the same samples and a 6 s timeout the forklift stopped, regained detections, withdrew, and completed. At index 80 the same outage began later. The default timeout ended with 14.9 mm of insertion and zero contact. The extended timeout again allowed one retry and completion. This paired timing experiment addresses assignment p. 2, item 08. The changed timeout also extends the age limit for guarded travel outside the face. [Outage study](recordings/geometric-outage-study.png)

The success and failure archives contain the 3D Rerun view, estimator replay, command history, and playback poses. The videos show the approach and final positions. The evaluator measures contact and pocket membership at physics rate. The hardware note covers first-week commissioning for the p. 4 question. [Success video](recordings/geometric-success/scene.mp4) · [Failure video](recordings/geometric-failure/scene.mp4) · [Hardware commissioning](REAL_FORKLIFT_FIRST_WEEK.md)
