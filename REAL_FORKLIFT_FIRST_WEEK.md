# First week on the real forklift

I would bring the stack onto the forklift in the following order. Each stage must pass before I expand the operating conditions.

1. **Verify the vehicle and its safety controls.** Inspect the forklift and establish a segregated test area. Check emergency stop and manual takeover before software commands can cause movement.

2. **Validate the communication interface.** With motion disabled, check command units, steering signs, feedback, timestamps, and control ownership. Verify command expiry and command-loss braking through the vehicle controller. These checks prevent interface errors from becoming unexpected motion.

3. **Verify recording and visualization.** Record timestamped sensor data, estimates, commands, actuator feedback, and supervisor decisions. Check that replay preserves their timing and that visualization uses the correct coordinate frames. During a short manual maneuver, confirm that displayed motion matches the real forklift. This makes later calibration, latency, and tracking errors easier to diagnose.

4. **Calibrate geometry and sensing.** Measure sensor-to-vehicle and vehicle-to-fork transforms, steering zero, odometry scale, fork height, and fork spacing. Compare pallet estimates with surveyed positions across the approach. Check close-range visibility and fork occlusion before trusting perception near the pockets.

5. **Measure vehicle response and timing.** Use supervised, low-speed tests to measure steering response, acceleration, braking, and end-to-end latency. Update the motion model and stopping margins before tuning the controller.

6. **Verify perception and estimation.** Run perception and estimation during manual driving without autonomous command authority. Compare the estimates with independent measurements before using them to control the forklift.

7. **Verify path and trajectory tracking.** Track a known feasible path on clear floor, followed by the same path with a speed profile. Use independent position measurements to distinguish estimation error from control error. Establish reliable tracking before attempting pallet engagement.

8. **Attempt controlled pallet engagement.** Start with a centered approach that stops before the pallet face. Permit shallow insertion once measured uncertainty and tracking error fit within the pocket clearance. Gradually increase insertion depth and initial misalignment to test the complete loop while limiting the consequences of errors.

9. **Test failures and establish the operating limits.** Begin command-loss tests before autonomous motion, then repeat fault tests as integration progresses. Inject dropped or frozen detections, missed deadlines, feedback loss, and computer disconnects. Verify that each fault triggers a safe stop. After the fault clears, require valid feedback and a fresh pallet estimate before resuming. Permit backing out only when the recovery path is clear. Finish with repeated engagement trials and obstruction-stop tests. Record success rates, errors, stopping distances, and failure causes. Planning around obstacles would follow separately because the current stack assumes clear floor around one pallet.
