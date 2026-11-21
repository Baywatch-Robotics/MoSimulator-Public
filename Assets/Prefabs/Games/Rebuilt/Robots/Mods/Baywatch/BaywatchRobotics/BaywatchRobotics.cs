using System;
using MoSimCore.BaseClasses.GameManagement;
using MoSimCore.Enums;
using RobotFramework.Components;
using RobotFramework.Controllers.PidSystems;
using RobotFramework.Enums;
using UnityEngine;

public class BaywatchRobotics : RebuiltTurretRobot
{
    [Serializable]
    private sealed class CollisionPair
    {
        public Collider first;
        public Collider second;
    }

    [Serializable]
    private sealed class RuntimeCorePair
    {
        public InteractionRoller roller;
        public Collider other;
    }

    [Serializable]
    private sealed class ShooterSide
    {
        public GenericTurret turret;
        public Transform swmReference;
        public RollerGroup flywheels;
        public RollerGroup kickers;
        public InteractionRoller indexer;
        public bool sideEnabled = true;
        public float indexerRpm = 1200f;
        public float kickerRpm = 1000f;

        [Tooltip("Provisional fixed-hood calibration: horizontal distance in metres -> wheel RPM. Tune with fuel in Play Mode.")]
        public AnimationCurve hubRpm = AnimationCurve.Linear(1.5f, 2600f, 6f, 5000f);
        public AnimationCurve passRpm = AnimationCurve.Linear(2f, 2400f, 10f, 5000f);

        [Header("Runtime diagnostics")]
        public bool targetValid;
        public bool ready;
        public float aimError;
        public float requestedRpm;
        public float targetDistance;

        [NonSerialized] public float readyTime;
        [NonSerialized] public Vector3 smoothedVelocity;
    }

    [Header("Intake")]
    [SerializeField] private GenericJoint intakePivot;
    [SerializeField] private RollerGroup intakeRollers;
    [SerializeField] private PidConstants intakePid = new(0.15f, 0f, 0.002f, 4.2f, 0f);
    [Tooltip("Actual E-pivot travel from the saved stowed pose, NOT the remotely belted encoder angle. Zero inhibits deployment and collection until calibrated. Floor pickup needs at least 118 deg: below that the crossbar roller cannot reach a fuel ball resting on the floor.")]
    [SerializeField] private float intakeDeployedAngle;
    [SerializeField, Min(1f)] private float intakeMaxVelocity = 120f;
    [SerializeField, Min(1f)] private float intakeMaxAcceleration = 240f;
    [SerializeField, Min(0f)] private float intakeDeployTolerance = 5f;
    [Tooltip("Motor roller speed. The two TPU-covered NEO 2.0 rollers run at 0.85 duty (about 4800 RPM); the crossbar is belt-reduced 17:31 by the roller group.")]
    [SerializeField] private float intakeRpm = 4800f;
    [SerializeField, Min(0f)] private float intakeMaxTorque = 80f;

    [Header("Independent shooters (hoods fixed)")]
    [SerializeField] private ShooterSide left = new();
    [SerializeField] private ShooterSide right = new();
    [Tooltip("Max is 360 deg/s. The real robot allows 720 deg/s, halved here so full wraps are less violent.")]
    [SerializeField] private PidConstants turretPid = new(0.2f, 0f, 0.002f, 6.28f, 0f);
    [Tooltip("Legacy tuning value retained for existing prefabs. Aim error is diagnostic only and no longer gates feeding.")]
    [SerializeField, Min(0.1f)] private float aimTolerance = 3f;
    [SerializeField, Min(0f)] private float feedReadyDelay = 0.08f;
    [SerializeField, Range(1f, 6000f)] private float maxShooterRpm = 6000f;
    [SerializeField, Min(0f)] private float turretMaxTorque = 25f;

    [Header("Authored collider overlap exceptions (same robot only)")]
    [SerializeField] private CollisionPair[] collisionExclusions = Array.Empty<CollisionPair>();
    [SerializeField] private RuntimeCorePair[] runtimeCoreExclusions = Array.Empty<RuntimeCorePair>();
    [SerializeField, Min(0.0001f)] private float linkageContactOffset = 0.0005f;

    [Header("Runtime intake diagnostics")]
    [SerializeField] private bool intakeDeployed;
    [SerializeField] private float intakeCommandAngle;
    [SerializeField] private float intakeMeasuredAngle;
    private float intakeCommandVelocity;
    private ConfigurableJoint intakeDrive;
    private ConfigurableJoint leftDrive;
    private ConfigurableJoint rightDrive;

    private bool RobotEnabled => BaseGameManager.Instance != null
        && BaseGameManager.Instance.RobotState == RobotState.Enabled;

    private bool IntakeCalibrated => !float.IsNaN(intakeDeployedAngle) && !float.IsInfinity(intakeDeployedAngle)
        && Mathf.Abs(intakeDeployedAngle) > 0.1f;

    protected override void Init()
    {
        base.Init();
        if (!ReferencesValid())
        {
            Debug.LogError("MIRA: mechanism references are incomplete. Run Tools > Baywatch > Configure MIRA Prefab.", this);
            enabled = false;
            return;
        }

        intakePivot.SetPid(intakePid);
        intakePivot.angularAxis = JointAxis.X;
        intakeDrive = intakePivot.GetComponent<ConfigurableJoint>();
        leftDrive = left.turret.GetComponent<ConfigurableJoint>();
        rightDrive = right.turret.GetComponent<ConfigurableJoint>();
        left.turret.SetPid(turretPid);
        right.turret.SetPid(turretPid);
        left.turret.GetComponent<Rigidbody>().maxAngularVelocity = 7f;
        right.turret.GetComponent<Rigidbody>().maxAngularVelocity = 7f;
        foreach (Rigidbody body in intakePivot.transform.parent.GetComponentsInChildren<Rigidbody>(true))
        {
            body.solverIterations = 64;
            body.solverVelocityIterations = 16;
        }
        StopRollers();
        if (!IntakeCalibrated)
            Debug.LogWarning("MIRA: set Intake Deployed Angle to the tested E-pivot travel before deploying/collecting. The real encoder angle is not a confirmed pivot angle.", this);
        Scheduler.AddProtectedSystem(UpdateJoints);
        Scheduler.AddLateRecursive(UpdatePids);
    }

    private bool ReferencesValid()
    {
        return intakePivot != null && intakeRollers != null && SideReferencesValid(left)
            && SideReferencesValid(right);
    }

    private static bool SideReferencesValid(ShooterSide side)
    {
        return side != null && side.turret != null && side.swmReference != null
            && side.swmReference.parent != null && side.flywheels != null
            && side.kickers != null && side.indexer != null;
    }

    private void UpdatePids()
    {
        intakePivot.UpdatePid(intakePid);
        left.turret.UpdatePid(turretPid);
        right.turret.UpdatePid(turretPid);
    }

    protected override void LateInit()
    {
        base.LateInit();
        ApplyCollisionExclusions();
    }

    private void ApplyCollisionExclusions()
    {
        // Collider.contactOffset is runtime-only. Default 10 mm padding exceeds the actual
        // gaps between the thin linkage plates; apply sub-millimetre padding after Start.
        foreach (Collider collider in intakePivot.transform.parent.GetComponentsInChildren<Collider>(true))
            collider.contactOffset = linkageContactOffset;
        // Only the explicitly recorded overlapping collider pairs are excluded. Fuel, field
        // contact, and collisions with other robots remain enabled.
        foreach (CollisionPair pair in collisionExclusions)
            if (pair.first != null && pair.second != null)
                Physics.IgnoreCollision(pair.first, pair.second);
        foreach (RuntimeCorePair pair in runtimeCoreExclusions)
        {
            if (pair.roller == null || pair.other == null) continue;
            Transform core = pair.roller.transform.Find("Wheel_InternalCore");
            if (core != null && core.TryGetComponent(out Collider collider))
                Physics.IgnoreCollision(collider, pair.other);
        }
    }

    protected override void HandleSwm()
    {
        if (rb == null) return;
        UpdateSwm(left);
        UpdateSwm(right);
    }

    private void UpdateSwm(ShooterSide side)
    {
        if (side.swmReference == null || side.swmReference.parent == null) return;
        Vector3 origin = side.swmReference.parent.position;
        // Each shooter gets its own point velocity, including chassis rotation about the COM.
        side.smoothedVelocity = Vector3.Lerp(side.smoothedVelocity, rb.GetPointVelocity(origin),
            Mathf.Clamp01(swmSettings.velocityDampeningValue));
        side.swmReference.position = origin + side.smoothedVelocity * Mathf.Max(0f, swmSettings.timeOfFlight);
    }

    protected override void AllianceZone()
    {
        SelectTarget(left, false);
        SelectTarget(right, false);
    }

    protected override void NeutralZone()
    {
        SelectTarget(left, true);
        SelectTarget(right, true);
    }

    private void SelectTarget(ShooterSide side, bool passing)
    {
        // The framework leaves the previous target intact when a map is missing. Clear it first
        // so missing field nodes never silently keep a stale hub/pass target or permit feeding.
        side.turret.ClearTarget();
        side.targetValid = false;
        if (!side.sideEnabled) return;
        if (passing) side.turret.SetPassMapClosest();
        else side.turret.SetDefaultMap(GenericTurret.DefaultMap.Hub);

        Vector3 target = side.turret.GetCurrentTarget();
        side.targetValid = target != Vector3.zero;
        if (!side.targetValid) return;
        Vector3 delta = target - side.swmReference.position;
        side.targetDistance = new Vector2(delta.x, delta.z).magnitude;
        AnimationCurve curve = passing ? side.passRpm : side.hubRpm;
        float rpm = curve != null && curve.length > 0 ? curve.Evaluate(side.targetDistance) : 0f;
        side.requestedRpm = float.IsNaN(rpm) || float.IsInfinity(rpm)
            ? 0f : Mathf.Clamp(rpm, 0f, maxShooterRpm);
    }

    protected override void ShooterHandler()
    {
        if (!ReferencesValid()) return;
        if (!RobotEnabled)
        {
            StopRollers();
            ResetReadiness(left);
            ResetReadiness(right);
            // Resume the profile from the measured pose after a disabled sag, not an old command.
            intakeCommandAngle = Mathf.DeltaAngle(0f, intakePivot.GetAngle());
            intakeCommandVelocity = 0f;
            return;
        }

        if (StowAction)
        {
            intakeDeployed = false;
            Idling();
            return;
        }
        base.ShooterHandler();
    }

    protected override void Intaking() => RunMechanisms(true, false);
    protected override void IntakingAndShooting() => RunMechanisms(true, true);
    protected override void Shooting() => RunMechanisms(false, true);
    protected override void Idling() => RunMechanisms(false, false);

    private void RunMechanisms(bool collecting, bool shooting)
    {
        if (collecting) intakeDeployed = IntakeCalibrated;
        bool deployed = IntakeCalibrated && Mathf.Abs(Mathf.DeltaAngle(intakePivot.GetAngle(), intakeDeployedAngle))
            <= intakeDeployTolerance;
        intakeRollers.SetRpm(collecting && deployed ? intakeRpm : 0f);
        PrepareSide(left, shooting);
        PrepareSide(right, shooting);
    }

    private void PrepareSide(ShooterSide side, bool shooting)
    {
        // Feed is the only place allowed to energize either lane. Always revoke last tick's feed.
        side.indexer.SetRpm(0f);
        side.kickers.SetRpm(0f);
        bool spin = shooting && side.sideEnabled && side.targetValid && side.requestedRpm > 0f;
        side.flywheels.SetRpm(spin ? side.requestedRpm : 0f);
        side.aimError = side.targetValid ? Mathf.Abs(side.turret.GetError()) : 180f;
        bool readyNow = spin && !side.turret.IsWrapping()
            && side.flywheels.AtSpeed();
        side.readyTime = readyNow ? side.readyTime + Time.fixedDeltaTime : 0f;
        side.ready = readyNow && side.readyTime >= feedReadyDelay;
    }

    protected override bool Aligned() => left.ready || right.ready;

    protected override void Feed()
    {
        FeedSide(left);
        FeedSide(right);
    }

    private static void FeedSide(ShooterSide side)
    {
        if (!side.ready) return;
        side.indexer.SetRpm(side.indexerRpm);
        side.kickers.SetRpm(side.kickerRpm);
    }

    private void UpdateJoints()
    {
        intakeMeasuredAngle = Mathf.DeltaAngle(0f, intakePivot.GetAngle());
        if (!RobotEnabled) return;
        // GenericJoint restores infinite drive force when re-enabled. Reapply MIRA's finite
        // mechanism torque limits before each physics step, including after disable/enable.
        LimitDrive(intakeDrive, intakeMaxTorque, false);
        LimitDrive(leftDrive, turretMaxTorque, true);
        LimitDrive(rightDrive, turretMaxTorque, true);
        float target = intakeDeployed && IntakeCalibrated ? intakeDeployedAngle : 0f;
        StepIntakeProfile(target, Time.fixedDeltaTime);
        intakePivot.SetTargetAngle(intakeCommandAngle).withAxis(JointAxis.X).withFixedDTiming();
        float steer = RotateAction.ReadValue<float>();
        if (left.targetValid) left.turret.Align(steer);
        if (right.targetValid) right.turret.Align(steer);
    }

    private static void LimitDrive(ConfigurableJoint joint, float torque, bool swing)
    {
        JointDrive drive = swing ? joint.angularYZDrive : joint.angularXDrive;
        drive.maximumForce = Mathf.Max(0f, torque);
        if (swing) joint.angularYZDrive = drive;
        else joint.angularXDrive = drive;
    }

    private void StepIntakeProfile(float target, float dt)
    {
        float error = target - intakeCommandAngle;
        float acceleration = Mathf.Max(1f, intakeMaxAcceleration);
        float brakingVelocity = Mathf.Sqrt(2f * acceleration * Mathf.Abs(error));
        float desiredVelocity = Mathf.Sign(error) * Mathf.Min(Mathf.Max(1f, intakeMaxVelocity), brakingVelocity);
        intakeCommandVelocity = Mathf.MoveTowards(intakeCommandVelocity, desiredVelocity, acceleration * dt);
        float step = intakeCommandVelocity * dt;
        if (Mathf.Abs(step) >= Mathf.Abs(error) && step * error >= 0f)
        {
            intakeCommandAngle = target;
            intakeCommandVelocity = 0f;
        }
        else intakeCommandAngle += step;
    }

    private static void ResetReadiness(ShooterSide side)
    {
        side.ready = false;
        side.readyTime = 0f;
    }

    private void StopRollers()
    {
        if (intakeRollers != null) intakeRollers.SetRpm(0f);
        StopSide(left);
        StopSide(right);
    }

    private static void StopSide(ShooterSide side)
    {
        if (side == null) return;
        if (side.flywheels != null) side.flywheels.SetRpm(0f);
        if (side.kickers != null) side.kickers.SetRpm(0f);
        if (side.indexer != null) side.indexer.SetRpm(0f);
        ResetReadiness(side);
    }

    private void OnDisable() => StopRollers();
}
