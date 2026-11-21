# MIRA — first playable mechanism setup

The gameplay prefab is `BaywatchRobotics/MIRA.prefab`. Its root has the
`BaywatchRobotics` controller. Both hoods remain fixed in their saved CAD poses.

## Controls

**Before using Intake:** check `Intake Deployed Angle` on the root controller.
Initial setup uses **0**, which inhibits automatic deployment/collection. Owner
testing values are preserved by the runtime repair tool. The real robot's
121-degree encoder reading is not a known E-pivot angle: the encoder was belted out
with an unconfirmed ratio. Enter the actual pivot travel found in testing; the
prefab is currently calibrated to **118°**, the shallowest angle that reaches floor
fuel (see "Intake deployed angle" below).
`Intake Measured Angle` shows the E-pivot encoder used by the simulation. Changes
made to tuning fields during Play Mode must be copied back after exiting Play Mode.

Uses the player's normal MoSim Rebuilt bindings, not the real robot's two-controller bindings:

| Action | Behavior |
| --- | --- |
| Intake | Deploy the six-bar linkage and collect once it is near the deployed position. |
| Release Intake | Stop the intake roller; leave the intake deployed. |
| Stow | Retract the intake and stop all rollers while held; takes priority over intake/shoot. |
| AutoShoot | Spin both shooters and feed each ready side independently. |
| Intake + AutoShoot | Collect and shoot simultaneously. |

Turrets track the alliance hub in the alliance zone and the closest passing map
target outside it. Missing target nodes inhibit feeding. Releasing shoot revokes
feed immediately. A side can be disabled independently in the root controller's Inspector.

## Tuning still required with fuel

- `Left` / `Right` -> `Hub Rpm` and `Pass Rpm` are **provisional fixed-hood curves**
  (horizontal distance in metres, output wheel RPM). They are not calibrated shots.
  Mira's real ballistic solver varies hood pitch, so its RPMs cannot be copied as-is.
- Both shooter groups command `PvffController` components, which model spin-up and
  game-piece load. Their readiness is modeled, not a measurement of physical wheel inertia.
- `Feed Ready Delay`, intake speed, indexer speeds, and kicker speeds
  are exposed on the robot controller. Group entry flips handle mirrored shaft axes.
  Verify fuel travels in the intended direction before tuning shooting distance.
- `SWM Settings` -> `Time Of Flight` starts at **0** (no velocity lead). Both turret
  references are updated independently; enable/tune lead only after stationary shots work.
- The saved stowed pose is the intake's 0-degree reference. `Intake Deployed Angle`
  must be calibrated at the actual pivot. The 120 deg/s velocity and 240 deg/s² acceleration
  profile limits are the real robot's 240/480 values halved for the simulation, not
  confirmed pivot speeds. Unity PID gains are also simulation tuning, not Spark gains.

There are 17 interaction rollers: 3 intake, 4 shooter, 2 indexer, and 8 kicker.
Kickers retain only the four outer tread interactions per side under `9.55`
assemblies. The small `am-3948-1000` spacer/core interactions are removed; their
CAD meshes and authored colliders remain intact. Roller sizes, group directions,
speed multipliers, and other owner tuning are preserved by the targeted removal tool.

Deleting an InteractionRoller component does not automatically clear its RollerGroup
entry. `Tools > Baywatch > Remove Kicker Spacer Rollers (Preserve Tuning)` removes
the remaining spacer interactions and stale entries without rebuilding the prefab.
RollerGroup commands now skip missing members with a one-time warning rather than
throwing during startup; a group with missing members cannot report readiness.

The two intake motor rollers are the TPU-covered NEO 2.0 outputs that grip the ball
and run at motor speed. The long crossbar is belted from a 17T to a 31T pulley, so
the roller group gives it a `0.548` multiplier and it turns at 17/31 of the motors.
The controller's `Intake Rpm` is the motor speed (4800 RPM ≈ 0.85 duty free speed);
the crossbar therefore runs near 2630 RPM.

## Intake deployed angle

`Intake Deployed Angle` must be deep enough for the crossbar to actually touch a fuel
ball resting on the floor. Measured on the built prefab with the roof at floor level:

| Deployed angle | Crossbar bottom | Ball top (5.9 in) | Result |
| --- | --- | --- | --- |
| 110° | 0.171 m | 0.150 m | 21 mm gap, never contacts |
| 114° | 0.156 m | 0.150 m | 6 mm gap, never contacts |
| 118° | 0.140 m | 0.150 m | 10 mm interference, picks up |
| 123° | 0.117 m | 0.150 m | linkage mechanical limit |

The prefab currently uses **118°**. The table is an unloaded, gravity-free linkage
measurement, not a guarantee of pickup in play: gravity and ball contact change the
measured pose. The pickup regression now uses mechanism gravity, the configured
torque limit, production motion profile/angle gating, and animated meshes, and
requires the ball to pass the front lip and enter the hopper. It does not impose
a hard-coded minimum deployment angle on future owner tuning.

## Physics setup

Turret physics roots and their encoder parent frame have vertical local Y axes.
The imported parent originally had a 180-degree X rotation; correcting only the
child roots left the aiming encoder in the wrong frame. Child world poses are
preserved. All linkage connected anchors are explicitly matched to their pins.
Joint frames now use conventional X/Y axes with Swap Bodies disabled, matching
the production GenericJoint PID polarity.

Each shooter tread has a visual-only `MIRA_VisualSpin` pivot at its axle. Its model
reference never points at the whole Flywheel assembly or an ancestor of a physics
roller. This prevents mesh animation from moving triggers and collider assemblies.

Intake, kicker, and indexer visuals also use axle-centred spin pivots. Indexer mesh
children are separated from the `Model/collider` branch so animation cannot rotate
solid colliders. The two oversized `3.14-1.002` indexer shaft imports have both their
CAD-frame translation and scale corrected by 1/100.

The former solid chassis envelope (`Stationary/Drivetrain/Model/collider/Box (9)`)
blocked ground fuel at the intake mouth. It is split into a rear envelope, two side
segments, and a 20 mm lower floor at the opening, leaving a 680 mm-wide intake path.
Existing same-robot exclusions are copied to the new segments; field and fuel
collision remain enabled. No robot-wide collision layer is disabled.

Disabled rollers exert no powered holding force. Fuel can roll back out through
the deployed intake; there is no simulated one-way latch or artificial retention.

Indexer directions are left positive / right negative with the saved downward
physics axles: **right clockwise, left counterclockwise when viewed from above**.
These are the owner's specified rotations, not a guessed hopper flow direction. Kicker directions
are unchanged; their facing surfaces already move upward. Lower turret mounting
and plate box approximations are trimmed to leave a 160 mm feed throat around each
turret axis. Structural collision outside the throat remains, as do the upper
shooter and hood surfaces. This prevents a ball being pinched below the turret inlet.

Kickers are a ramp-fed lift, not a pinch between the two wheel rows. Fuel arrives
at the front pair along the CAD ramp, the front pair carries it up, and the back
pair hands it into the turret inlet. Front tread `Max Squish` is **1 inch** so the
front wheels reach a ball on the ramp; the back pair keeps **0.25 inches**. All
eight use `Core Friction` 0, because the simulated solid core does not rotate and
otherwise acts as a brake on a compressed ball while the tread keeps driving.
Commands are **1500 RPM** per lane; the saved per-roller direction signs are
unchanged (left treads run negative, right positive, so both facing surfaces move
upward). Squish also sets the runtime core radius (`radius - maxSquish`).

The CAD ramp's generated stair-step colliders are disabled and replaced by thin
sloped `MIRA_SmoothKickerRamp` surfaces. The chassis aperture above the two ramps
is cleared to the rear envelope and side pieces, with a 20 mm floor retained. The
drive shaft that turns the belt has a visual-only `MIRA_VisualSpin_Drive` pivot
linked to a back roller; it adds no fuel-contact roller. `Tools > Baywatch >
Tune Kicker Compression Only` and `Tools > Baywatch > Repair Kicker Ramp Feed
(Preserve Intake and Indexers)` apply the same tread profile.

The acquisition regressions start fuel at the lane mouth on the ramp and use the
production readiness/feed path, comparing powered lift to an unpowered control.
A ball dropped at wheel axle height is not a pickup test.

The coarse front-frame collider intersects the two compound intake-coupler shapes
during their travel. Other authored intake/chassis colliders also overlap in stow.
`Collision Exclusions` records specific collider pairs to ignore within **this robot**;
it does not disable intake contact with fuel, the field, or other robots. These
exceptions are applied in `LateInit`, after runtime colliders have been constructed.
Two additional local exceptions avoid a coupler/hopper-guide box collision near
43 degrees. Linkage contact padding is configurable and starts at 0.5 mm.
Two runtime-core exceptions handle the intake roller's generated solid core
against the adjacent stationary hopper-guide boxes. Those cores do not exist
until InteractionRoller.Start, so an edit-only collider bench cannot detect them.
Refining those collision approximations can eliminate the need for the exceptions.

## Authoring and validation

The prefab is already configured. **Do not rerun setup just to play.**

- `Tools > Baywatch > Validate MIRA Prefab` checks references, seven-joint topology,
  pin positions, turret/encoder-parent axes, visual-only axle pivots, roller
  ownership, the intake command profile, independent feeding, and feed cancellation.
- `Tools > Baywatch > Repair MIRA Runtime Setup (Preserve Tuning)` repairs encoder
  parent/drive frames and shooter model pivots, normalizes PID polarity without
  changing gain magnitudes, and wires the runtime-core exceptions. It preserves
  intake travel, speeds, shot curves, and other owner tuning. This repair is
  already applied; do not run the full reset tool to get it.
- `Tools > Baywatch > Configure MIRA Prefab` is an explicit rebuild tool. It removes
  non-9.55 kicker interaction components, rebuilds functional groups, configures
  drives/controllers and references, and regenerates overlap exceptions. It resets
  group flips/speeds, intake travel calibration/profile limits, and SWM lead; back
  up subsequent manual tuning before using it.

For automated checks with the project closed in the GUI, run Unity 2023.2.22f1 with
`-batchmode -nographics -quit -projectPath <project> -executeMethod Baywatch.Editor.MiraSetup.ValidatePrefab`.
`Baywatch.Editor.MiraSetup.ProbePhysics` additionally exercises small powered intake
motions (5 and 45 degrees), return to stow, and both turret turns in a temporary,
gravity-free batch scene. These angles are actuation tests, not deployed-position
calibration. It
asserts endpoint accuracy and anchor separation, but does not substitute for a
field/fuel Play Mode test. `ConfigureAndValidate` rebuilds and runs both checks.

### Play Mode regressions

`Baywatch.Tests.MiraRuntimeTests` exercises the actual GenericTurret/GenericJoint
PID APIs and InteractionRoller/PvffController lifecycles, rather than substituting
manual drive velocities. It checks:

- Settling on a stationary target and holding it while the chassis turns.
- A **50-degree test pose** and return to stow, with gravity, the production motion
  profile, authored colliders, and runtime-generated roller cores enabled. This is
  a regression pose, **not a claimed real deployed angle**.
- Floor pickup at the calibrated deployed angle: a fuel ball rests on a real ground
  plane under the crossbar, and collection uses production angle gating, profile,
  mechanism gravity, and finite torque. Animated meshes remain enabled, and the
  ball must enter the hopper rather than stall at the front lip.
- Intake/indexer/kicker animation keeps all colliders fixed to their rigidbodies,
  maintains centred visual pivots, and rejects orbiting or oversized meshes.
- Each indexer follows the specified top-down rotation. Fuel entering a kicker lane
  from the front along the ramp is lifted past the turret inlet; reversed kicker
  commands eject it back out of the lane mouth. This does not calibrate a complete shot.
- Kicker acquisition through the production readiness/feed path with flywheel
  spin-up, aim/readiness, gravity, and an unpowered control; fuel must reach above
  the inlet.

Aim error is diagnostic only and does not block feeding. Shoot request, enabled
side, valid target/positive RPM, no turret unwind, flywheel speed readiness, and the
readiness delay still apply. A misaligned-target regression checks that kickers
receive commands after spin-up even with substantial aim error.
- Shooter visual animation while aiming, with physics offsets fixed and meshes
  staying centred on their axles.

Run through Unity's Play Mode Test Runner, or use `-runTests -testPlatform PlayMode
-testFilter Baywatch.Tests.MiraRuntimeTests -testResults <results.xml>` in a separate
batch Editor process. These isolated tests still do not calibrate fuel trajectories
or replace a complete field/input integration test.

Deferred: moving hoods, calibrated full-range ballistics/SWM, sensor-based jam
recovery/cross-feed, real-robot autonomous routines, and cosmetic swerve animation.
