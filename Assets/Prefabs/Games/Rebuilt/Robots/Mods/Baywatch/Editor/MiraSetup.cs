using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using RobotFramework.Components;
using RobotFramework.Enums;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Baywatch.Editor
{
    // Explicit authoring tool: never rewrites the prefab on import or when entering Play Mode.
    public static class MiraSetup
    {
        public const string PrefabPath = "Assets/Prefabs/Games/Rebuilt/Robots/Mods/Baywatch/BaywatchRobotics/MIRA.prefab";

        public static void ConfigureAndValidate()
        {
            Configure();
            ValidatePrefab();
            ProbePhysics();
        }

        [MenuItem("Tools/Baywatch/Configure MIRA Prefab")]
        public static void Configure()
        {
            if (!Application.isBatchMode && !EditorUtility.DisplayDialog("Rebuild MIRA setup?",
                    "Save your prefab changes first. This rebuilds roller groups and resets intake calibration/profile limits, group directions, indexer speeds, and SWM lead. Back up any manual tuning before continuing.",
                    "Rebuild", "Cancel")) return;
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                Transform intake = Require(robot.transform, "Intake/Main Link");
                Transform kickers = Require(robot.transform, "Kickers");
                Transform turrets = Require(robot.transform, "Turrets");
                Transform indexers = Require(robot.transform, "Indexer");
                // GenericJoint's encoder is relative to the PARENT, not to world axes.
                // The CAD assembly parent originally had an X=180 degree import rotation.
                ReorientPreservingChildren(turrets, robot.transform.rotation);
                // Preserve every renderer/model and authored collider. Only remove generated
                // interaction components outside the user's explicitly retained 9.55 assemblies.
                int removed = 0;
                foreach (InteractionRoller roller in kickers.GetComponentsInChildren<InteractionRoller>(true))
                {
                    if (Has955Ancestor(roller.transform, kickers)
                        && !AnimationUtility.CalculateTransformPath(roller.transform, kickers).Contains("am-3948-1000")) continue;
                    Object.DestroyImmediate(roller);
                    removed++;
                }

                // Replace per-part generated groups with one unambiguous command owner per mechanism.
                foreach (Transform branch in new[] { intake, kickers, turrets, indexers })
                    foreach (RollerGroup group in branch.GetComponentsInChildren<RollerGroup>(true))
                        Object.DestroyImmediate(group);

                RollerGroup intakeGroup = MakeGroup(intake,
                    intake.GetComponentsInChildren<InteractionRoller>(true), r => Vector3.Dot(r.transform.right, Vector3.right));
                ApplyIntakeBeltRatio(robot);
                var script = robot.GetComponent<BaywatchRobotics>();
                bool addedScript = script == null;
                if (addedScript) script = robot.AddComponent<BaywatchRobotics>();
                var serialized = new SerializedObject(script);
                CorrectPidPolarity(serialized);
                serialized.FindProperty("intakePivot").objectReferenceValue = Require(robot.transform, "Intake/Back Link").GetComponent<GenericJoint>();
                serialized.FindProperty("intakeRollers").objectReferenceValue = intakeGroup;
                // The real 121-degree reading is from a belted-out encoder with unknown ratio.
                // Do not treat it as linkage output travel or deploy to that angle automatically.
                serialized.FindProperty("intakeDeployedAngle").floatValue = 0f;
                serialized.FindProperty("intakeMaxVelocity").floatValue = 120f;
                serialized.FindProperty("intakeMaxAcceleration").floatValue = 240f;
                serialized.FindProperty("intakeRpm").floatValue = 4800f;
                if (addedScript) serialized.FindProperty("preloads").arraySize = 0;
                ConfigureSide(robot.transform, serialized.FindProperty("left"), "Left");
                ConfigureSide(robot.transform, serialized.FindProperty("right"), "Right");
                serialized.FindProperty("swmRef").objectReferenceValue = Require(robot.transform, "Turrets/Right/AimReference/SWM_Ref");
                SerializedProperty swm = serialized.FindProperty("swmSettings");
                swm.FindPropertyRelative("timeOfFlight").floatValue = 0f;
                swm.FindPropertyRelative("velocityDampeningValue").floatValue = 0.2f;
                serialized.ApplyModifiedPropertiesWithoutUndo();

                ConfigureDrive(Require(robot.transform, "Intake/Back Link").GetComponent<ConfigurableJoint>(), false);
                foreach (HingeJoint hinge in Require(robot.transform, "Intake").GetComponentsInChildren<HingeJoint>(true))
                    MatchConnectedAnchor(hinge);
                foreach (InteractionRoller roller in robot.GetComponentsInChildren<InteractionRoller>(true))
                {
                    var settings = new SerializedObject(roller);
                    settings.FindProperty("rpm").floatValue = 0f;
                    // Contact forces should act on fuel, not on other moving bodies of this robot.
                    settings.FindProperty("ignoreLayers").intValue |= LayerMask.GetMask("Robot");
                    settings.ApplyModifiedPropertiesWithoutUndo();
                }

                RecordOverlapExceptions(robot, script);
                RecordRuntimeCoreExceptions(robot, script);
                RepairFeedVisuals(robot);
                RepairIntakeAperture(robot);
                RepairKickerPassages(robot);

                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log($"MIRA configured: removed {removed} non-9.55 kicker rollers; kept {robot.GetComponentsInChildren<InteractionRoller>(true).Length} interaction rollers. Hoods and CAD geometry preserved.");
            }
            finally
            {
                PrefabUtility.UnloadPrefabContents(robot);
            }
        }

        private static void ConfigureSide(Transform root, SerializedProperty side, string name)
        {
            Transform turret = Require(root, "Turrets/" + name);
            InteractionRoller[] wheels = Require(turret, "Flywheel").GetComponentsInChildren<InteractionRoller>(true);
            Check(wheels.Length == 2, name + ": expected two existing shooter rollers.");

            // The saved wheel centres lie in the initial shooting direction from the turret pivot.
            // Give the physics root that forward direction with Y up, while preserving all children.
            Vector3 centre = (wheels[0].transform.position + wheels[1].transform.position) * 0.5f;
            Vector3 forward = Vector3.ProjectOnPlane(centre - turret.position, Vector3.up).normalized;
            Check(forward.sqrMagnitude > 0.9f, name + ": cannot determine the initial shooter direction.");
            ReorientPreservingChildren(turret, Quaternion.LookRotation(forward, Vector3.up));
            ConfigureDrive(turret.GetComponent<ConfigurableJoint>(), true);

            var aim = new SerializedObject(turret.GetComponent<GenericTurret>());
            aim.FindProperty("turretAxis").enumValueIndex = 1; // JointAxis.Y
            aim.FindProperty("angleOffset").floatValue = 0f;
            aim.FindProperty("useStartingOffset").boolValue = false;
            aim.FindProperty("flipDirection").boolValue = false;
            aim.FindProperty("wrappingSettings.useNoWrap").boolValue = true;
            aim.FindProperty("wrappingSettings.noWrapMin").floatValue = -15f;
            aim.FindProperty("wrappingSettings.noWrapMax").floatValue = 345f;
            aim.ApplyModifiedPropertiesWithoutUndo();

            foreach (InteractionRoller wheel in wheels)
            {
                RepairShooterVisual(wheel);
                var wheelSettings = new SerializedObject(wheel);
                // The fuel collider is a rigid sphere. A solid wheel core would leave the
                // ballistic pinch narrower than the ball, so the wheel keeps only its tread
                // contact model. This matches the game robots' 4-inch shooter wheels.
                wheelSettings.FindProperty("maxSquish").floatValue = wheelSettings.FindProperty("radius").floatValue;
                wheelSettings.ApplyModifiedPropertiesWithoutUndo();
                var controller = wheel.GetComponent<PvffController>() ?? wheel.gameObject.AddComponent<PvffController>();
                var config = new SerializedObject(controller);
                config.FindProperty("m_GamePieceLayers").intValue = LayerMask.GetMask("GamePiece1", "GamePiece2", "GamePieceAlt");
                config.FindProperty("m_MaxAccelerationRpmPerSecond").floatValue = 12000f;
                config.ApplyModifiedPropertiesWithoutUndo();
            }

            // Fuel is pulled up the back plate by the wheel, so the surface beside the ball
            // must move upward: the opposite of the old down-and-inward direction.
            RollerGroup shooterGroup = MakeGroup(Require(turret, "Flywheel"), wheels,
                r => -Vector3.Dot(Vector3.Cross(r.transform.right, Vector3.down), forward));
            Transform lane = Require(root, "Kickers/" + name);
            RollerGroup kickerGroup = MakeGroup(lane, lane.GetComponentsInChildren<InteractionRoller>(true), r =>
            {
                bool front = AncestorNamed(r.transform, lane, "Front");
                Vector3 contactDirection = front ? Vector3.back : Vector3.forward;
                return Vector3.Dot(Vector3.Cross(r.transform.right, contactDirection), Vector3.up);
            });
            side.FindPropertyRelative("turret").objectReferenceValue = turret.GetComponent<GenericTurret>();
            side.FindPropertyRelative("swmReference").objectReferenceValue = Require(turret, "AimReference/SWM_Ref");
            side.FindPropertyRelative("flywheels").objectReferenceValue = shooterGroup;
            side.FindPropertyRelative("kickers").objectReferenceValue = kickerGroup;
            side.FindPropertyRelative("indexer").objectReferenceValue = Require(root, "Indexer/" + name).GetComponent<InteractionRoller>();
            side.FindPropertyRelative("indexerRpm").floatValue = name == "Right" ? -1200f : 1200f;
        }

        [MenuItem("Tools/Baywatch/Repair MIRA Feed Direction and Passages (Preserve Speeds)")]
        public static void RepairFeedDirection()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                var settings = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
                settings.FindProperty("left.indexerRpm").floatValue = Mathf.Abs(settings.FindProperty("left.indexerRpm").floatValue);
                settings.FindProperty("right.indexerRpm").floatValue = -Mathf.Abs(settings.FindProperty("right.indexerRpm").floatValue);
                settings.ApplyModifiedPropertiesWithoutUndo();
                RepairKickerPassages(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        private static void RepairKickerPassages(GameObject robot)
        {
            foreach (string side in new[] { "Left", "Right" })
            {
                Transform turret = Require(robot.transform, "Turrets/" + side);
                Vector3 inlet = turret.position;
                // Clear only the lower feed throat, not the shooter/hood surfaces above it.
                var boxes = Require(robot.transform, "Stationary/Turrets/" + side).GetComponentsInChildren<BoxCollider>(true)
                    .Concat(Require(turret, "Stationary").GetComponentsInChildren<BoxCollider>(true)).ToArray();
                foreach (BoxCollider box in boxes)
                {
                    if (!box.enabled || box.isTrigger) continue;
                    Vector3 worldCenter = box.transform.TransformPoint(box.center);
                    if (worldCenter.y < inlet.y - 0.16f || worldCenter.y > inlet.y + 0.015f) continue;
                    Vector3[] axes = { box.transform.right, box.transform.up, box.transform.forward };
                    int vertical = Enumerable.Range(0, 3).OrderByDescending(i => Mathf.Abs(Vector3.Dot(axes[i], robot.transform.up))).First();
                    if (Mathf.Abs(Vector3.Dot(axes[vertical], robot.transform.up)) < 0.999f) continue;
                    int a = (vertical + 1) % 3;
                    int b = (vertical + 2) % 3;
                    Vector3 axis = box.transform.InverseTransformPoint(new Vector3(inlet.x, worldCenter.y, inlet.z));
                    Vector3 scale = box.transform.lossyScale;
                    float radiusA = 0.080f / Mathf.Abs(scale[a]);
                    float radiusB = 0.080f / Mathf.Abs(scale[b]);
                    Vector3 min = box.center - box.size * 0.5f;
                    Vector3 max = box.center + box.size * 0.5f;
                    float loA = Mathf.Max(min[a], axis[a] - radiusA);
                    float hiA = Mathf.Min(max[a], axis[a] + radiusA);
                    float loB = Mathf.Max(min[b], axis[b] - radiusB);
                    float hiB = Mathf.Min(max[b], axis[b] + radiusB);
                    if ((hiA - loA) * Mathf.Abs(scale[a]) < 0.0001f || (hiB - loB) * Mathf.Abs(scale[b]) < 0.0001f) continue;
                    var pieces = new List<(Vector3 center, Vector3 size)>();
                    void Piece(float a0, float a1, float b0, float b1)
                    {
                        if ((a1 - a0) * Mathf.Abs(scale[a]) < 0.0001f || (b1 - b0) * Mathf.Abs(scale[b]) < 0.0001f) return;
                        Vector3 center = box.center;
                        Vector3 size = box.size;
                        center[a] = (a0 + a1) * 0.5f;
                        center[b] = (b0 + b1) * 0.5f;
                        size[a] = a1 - a0;
                        size[b] = b1 - b0;
                        pieces.Add((center, size));
                    }
                    Piece(min[a], loA, min[b], max[b]);
                    Piece(hiA, max[a], min[b], max[b]);
                    Piece(loA, hiA, min[b], loB);
                    Piece(loA, hiA, hiB, max[b]);
                    if (pieces.Count == 0) { box.enabled = false; continue; }
                    for (int i = 1; i < pieces.Count; i++)
                    {
                        BoxCollider extra = box.gameObject.AddComponent<BoxCollider>();
                        extra.center = pieces[i].center;
                        extra.size = pieces[i].size;
                        extra.sharedMaterial = box.sharedMaterial;
                    }
                    box.center = pieces[0].center;
                    box.size = pieces[0].size;
                }
            }
        }

        [MenuItem("Tools/Baywatch/Remove Kicker Spacer Rollers (Preserve Tuning)")]
        public static void RemoveKickerSpacerRollers()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                int removed = 0;
                foreach (string side in new[] { "Left", "Right" })
                {
                    Transform lane = Require(robot.transform, "Kickers/" + side);
                    InteractionRoller[] spacers = lane.GetComponentsInChildren<InteractionRoller>(true)
                        .Where(r => AnimationUtility.CalculateTransformPath(r.transform, lane).Contains("am-3948-1000"))
                        .ToArray();
                    var settings = new SerializedObject(lane.GetComponent<RollerGroup>());
                    SerializedProperty entries = settings.FindProperty("rollers");
                    for (int i = entries.arraySize - 1; i >= 0; i--)
                    {
                        var entry = entries.GetArrayElementAtIndex(i);
                        var roller = (InteractionRoller)entry.FindPropertyRelative("IntRoller").objectReferenceValue;
                        bool missing = roller == null
                            && entry.FindPropertyRelative("PvffController").objectReferenceValue == null
                            && entry.FindPropertyRelative("GenRoller").objectReferenceValue == null
                            && entry.FindPropertyRelative("Flywheel").objectReferenceValue == null;
                        if (missing || spacers.Contains(roller)) entries.DeleteArrayElementAtIndex(i);
                    }
                    settings.ApplyModifiedPropertiesWithoutUndo();
                    foreach (InteractionRoller spacer in spacers) { Object.DestroyImmediate(spacer); removed++; }
                    Check(lane.GetComponentsInChildren<InteractionRoller>(true).Length == 4,
                        side + ": expected four outer tread rollers after removing spacer interactions.");
                }
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log($"MIRA: removed {removed} remaining kicker spacer interactions and stale group entries; tread tuning and geometry preserved.");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        [MenuItem("Tools/Baywatch/Tune Kicker Compression Only")]
        public static void TuneKickerCompression()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                InteractionRoller[] rollers = Require(robot.transform, "Kickers").GetComponentsInChildren<InteractionRoller>(true);
                Check(rollers.Length == 8, "Expected eight outer kicker tread interactions.");
                foreach (InteractionRoller roller in rollers)
                {
                    var settings = new SerializedObject(roller);
                    // Inches: retain the real 3-inch tread diameter. Front wheels meet the
                    // fuel along the ramp and need 1 in of compliance to reach it; the back
                    // pair keeps the 1/4-inch layer that supports the ball into the nip.
                    bool front = AnimationUtility.CalculateTransformPath(roller.transform, robot.transform).Contains("/Front/");
                    settings.FindProperty("maxSquish").floatValue = front ? 1f : 0.25f;
                    // The tread solver supplies the drive grip. A high-friction core does not
                    // rotate and would brake a ball compressed against the ramp.
                    settings.FindProperty("coreFriction").floatValue = 0f;
                    settings.ApplyModifiedPropertiesWithoutUndo();
                }
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log("MIRA kicker squish set to 1 in front / 0.25 in back with frictionless cores; radii, lengths, RPMs, directions, intake, and indexers preserved.");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        [MenuItem("Tools/Baywatch/Repair Shooter Passage (Preserve Tuning)")]
        public static void RepairShooterPassage()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                foreach (string name in new[] { "Left", "Right" })
                {
                    Transform turret = Require(robot.transform, "Turrets/" + name);
                    InteractionRoller[] wheels = Require(turret, "Flywheel").GetComponentsInChildren<InteractionRoller>(true);
                    Check(wheels.Length == 2, name + ": expected two shooter rollers.");
                    Vector3 centre = (wheels[0].transform.position + wheels[1].transform.position) * 0.5f;
                    Vector3 forward = Vector3.ProjectOnPlane(centre - turret.position, Vector3.up).normalized;
                    RollerGroup group = Require(turret, "Flywheel").GetComponent<RollerGroup>();
                    Check(group != null, name + ": Flywheel has no roller group.");
                    var groupSettings = new SerializedObject(group);
                    SerializedProperty members = groupSettings.FindProperty("rollers");
                    Check(members.arraySize == wheels.Length, name + ": Flywheel group does not match its rollers.");
                    foreach (InteractionRoller wheel in wheels)
                    {
                        var wheelSettings = new SerializedObject(wheel);
                        // Zero the solid core: the rigid fuel sphere must pass between the tread
                        // and the fixed hood. The roller impulse model still supplies grip.
                        wheelSettings.FindProperty("maxSquish").floatValue = wheelSettings.FindProperty("radius").floatValue;
                        wheelSettings.ApplyModifiedPropertiesWithoutUndo();
                    }
                    for (int i = 0; i < members.arraySize; i++)
                    {
                        var member = members.GetArrayElementAtIndex(i);
                        var controller = (PvffController)member.FindPropertyRelative("PvffController").objectReferenceValue;
                        var roller = controller != null
                            ? controller.GetComponent<InteractionRoller>()
                            : (InteractionRoller)member.FindPropertyRelative("IntRoller").objectReferenceValue;
                        Check(roller != null, name + ": Flywheel group has a missing roller reference.");
                        float direction = -Vector3.Dot(Vector3.Cross(roller.transform.right, Vector3.down), forward);
                        member.FindPropertyRelative("flipped").boolValue = direction < 0f;
                    }
                    groupSettings.ApplyModifiedPropertiesWithoutUndo();
                }
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log("MIRA shooter passage repaired: frictionless wheel cores and upward pull; intake, kickers, indexers, and RPM curves preserved.");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        [MenuItem("Tools/Baywatch/Repair Kicker Ramp Feed (Preserve Intake and Indexers)")]
        public static void RepairKickerRampAndDriveVisuals()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                foreach (InteractionRoller roller in Require(robot.transform, "Kickers").GetComponentsInChildren<InteractionRoller>(true))
                {
                    var tuning = new SerializedObject(roller);
                    bool front = AnimationUtility.CalculateTransformPath(roller.transform, robot.transform).Contains("/Front/");
                    // Front wheels meet the ball along the ramp, so they need the extra
                    // compliance to reach it; the back pair keeps the tuned 0.25 in.
                    tuning.FindProperty("maxSquish").floatValue = front ? 1f : 0.25f;
                    // The tread solver supplies grip; a stationary high-friction core
                    // otherwise brakes a fuel ball compressed against the ramp.
                    tuning.FindProperty("coreFriction").floatValue = 0f;
                    tuning.ApplyModifiedPropertiesWithoutUndo();
                }
                ClearChassisAboveKickerRamps(robot);
                foreach (string side in new[] { "Left", "Right" })
                {
                    Transform station = Require(robot.transform, "Stationary/Turrets/" + side);
                    MeshRenderer ramp = station.GetComponentsInChildren<MeshRenderer>(true).Single(m => m.name.StartsWith("4.83-1"));
                    Transform oldShapes = ramp.transform.Find("GeneratedClusterColliders");
                    Check(oldShapes != null, "Missing ramp collision approximation.");
                    foreach (Collider collider in oldShapes.GetComponentsInChildren<Collider>(true)) collider.enabled = false;
                    if (station.Find("MIRA_SmoothKickerRamp") == null)
                    {
                        var root = new GameObject("MIRA_SmoothKickerRamp").transform;
                        root.SetParent(station, false);
                        root.gameObject.layer = ramp.gameObject.layer;
                        float x = robot.transform.InverseTransformPoint(ramp.bounds.center).x;
                        // Thin, sloped surfaces replace the generated stair-step boxes. These
                        // follow the flat lead-in, rising middle, and steep rear of the CAD ramp.
                        Vector3[] points = { new(x, 0.130f, -0.018f), new(x, 0.135f, -0.088f),
                            new(x, 0.180f, -0.150f), new(x, 0.250f, -0.200f) };
                        for (int i = 0; i < points.Length - 1; i++)
                        {
                            Vector3 start = robot.transform.TransformPoint(points[i]);
                            Vector3 end = robot.transform.TransformPoint(points[i + 1]);
                            var part = new GameObject("Surface" + i).transform;
                            part.SetParent(root, false);
                            part.gameObject.layer = ramp.gameObject.layer;
                            part.SetPositionAndRotation((start + end) * 0.5f, Quaternion.LookRotation(end - start, robot.transform.up));
                            Vector3 scale = part.lossyScale;
                            part.localScale = new Vector3(part.localScale.x / scale.x, part.localScale.y / scale.y, part.localScale.z / scale.z);
                            BoxCollider surface = part.gameObject.AddComponent<BoxCollider>();
                            surface.size = new Vector3(0.150f, 0.003f, Vector3.Distance(start, end) + 0.001f);
                            surface.center = new Vector3(0f, -0.0015f, 0f);
                            surface.contactOffset = 0.0005f;
                        }
                    }
                    RepairKickerDriveVisual(robot.transform, side);
                }
                var controller = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
                foreach (string side in new[] { "left", "right" })
                {
                    var rpm = controller.FindProperty(side + ".kickerRpm");
                    rpm.floatValue = rpm.floatValue < 0f ? -1500f : 1500f;
                }
                controller.ApplyModifiedPropertiesWithoutUndo();
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        private static void ClearChassisAboveKickerRamps(GameObject robot)
        {
            Transform shape = Require(robot.transform, "Stationary/Drivetrain/Model/collider/Box (9)");
            if (shape.Find("MIRA_KickerRampClearance") != null) return;
            BoxCollider original = shape.GetComponent<BoxCollider>();
            Vector3 oldCenter = original.center;
            Vector3 oldSize = original.size;
            var pieces = new List<(Vector3 center, Vector3 size)> { (oldCenter, oldSize) };
            foreach (string side in new[] { "Left", "Right" })
            {
                float x = robot.transform.InverseTransformPoint(Require(robot.transform, "Turrets/" + side).position).x;
                Vector3 p0 = shape.InverseTransformPoint(robot.transform.TransformPoint(new Vector3(x - 0.080f, 0f, -0.220f)));
                Vector3 p1 = shape.InverseTransformPoint(robot.transform.TransformPoint(new Vector3(x + 0.080f, 0f, 0.100f)));
                var next = new List<(Vector3 center, Vector3 size)>();
                foreach (var piece in pieces)
                {
                    Vector3 min = piece.center - piece.size * 0.5f;
                    Vector3 max = piece.center + piece.size * 0.5f;
                    float a0 = Mathf.Max(min.x, Mathf.Min(p0.x, p1.x));
                    float a1 = Mathf.Min(max.x, Mathf.Max(p0.x, p1.x));
                    float b0 = Mathf.Max(min.z, Mathf.Min(p0.z, p1.z));
                    float b1 = Mathf.Min(max.z, Mathf.Max(p0.z, p1.z));
                    if (a1 <= a0 || b1 <= b0) { next.Add(piece); continue; }
                    void Add(float x0, float x1, float z0, float z1)
                    {
                        if (x1 - x0 < 0.0001f || z1 - z0 < 0.0001f) return;
                        next.Add((new Vector3((x0 + x1) * 0.5f, oldCenter.y, (z0 + z1) * 0.5f), new Vector3(x1 - x0, oldSize.y, z1 - z0)));
                    }
                    Add(min.x, a0, min.z, max.z);
                    Add(a1, max.x, min.z, max.z);
                    Add(a0, a1, min.z, b0);
                    Add(a0, a1, b1, max.z);
                }
                pieces = next;
            }
            // Preserve the low floor and all chassis volume outside the two ramp lanes.
            original.size = new Vector3(oldSize.x, 0.020f, oldSize.z);
            original.center = oldCenter + Vector3.up * (oldSize.y - 0.020f) * 0.5f;
            var marker = new GameObject("MIRA_KickerRampClearance").transform;
            marker.SetParent(shape, false);
            foreach (var piece in pieces) AddApertureBox(marker, "ChassisSegment", piece.center, piece.size, original);
            CopyApertureExclusions(robot, shape);
        }

        private static void RepairKickerDriveVisual(Transform robot, string side)
        {
            Transform lane = Require(robot, "Kickers/" + side);
            Transform drive = Require(lane, "Drive");
            if (drive.Find("MIRA_VisualSpin_Drive") != null) return;
            MeshFilter shaft = drive.GetComponentsInChildren<MeshFilter>(true).Single(m => m.name.StartsWith("4.72-1"));
            InteractionRoller source = Require(lane, "Back").GetComponentsInChildren<InteractionRoller>(true).First();
            MeshFilter[] meshes = drive.GetComponentsInChildren<MeshFilter>(true);
            var pivot = new GameObject("MIRA_VisualSpin_Drive").transform;
            pivot.SetParent(drive, false);
            pivot.gameObject.layer = shaft.gameObject.layer;
            pivot.SetPositionAndRotation(shaft.GetComponent<Renderer>().bounds.center, source.transform.rotation);
            Vector3 scale = pivot.lossyScale;
            pivot.localScale = new Vector3(pivot.localScale.x / scale.x, pivot.localScale.y / scale.y, pivot.localScale.z / scale.z);
            foreach (MeshFilter mesh in meshes)
            {
                Check(mesh.GetComponentsInChildren<Collider>(true).Length == 0, "Drive mesh owns collision geometry: " + mesh.name);
                mesh.transform.SetParent(pivot, true);
            }
            var settings = new SerializedObject(source);
            var models = settings.FindProperty("models");
            models.arraySize++;
            var entry = models.GetArrayElementAtIndex(models.arraySize - 1);
            entry.FindPropertyRelative("model").objectReferenceValue = pivot.gameObject;
            entry.FindPropertyRelative("jointAxis").enumValueIndex = 0;
            entry.FindPropertyRelative("reversed").boolValue = false;
            entry.FindPropertyRelative("multiplier").floatValue = 1f;
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        [MenuItem("Tools/Baywatch/Repair MIRA Runtime Setup (Preserve Tuning)")]
        public static void RepairRuntimeSetup()
        {
            Debug.Log("MIRA runtime repair: loading prefab.");
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                Debug.Log("MIRA runtime repair: correcting encoder parent and visual pivots.");
                ReorientPreservingChildren(Require(robot.transform, "Turrets"), robot.transform.rotation);
                foreach (string name in new[] { "Left", "Right" })
                {
                    RepairDriveFrame(Require(robot.transform, "Turrets/" + name).GetComponent<ConfigurableJoint>());
                    foreach (InteractionRoller wheel in Require(robot.transform, "Turrets/" + name + "/Flywheel").GetComponentsInChildren<InteractionRoller>(true))
                        RepairShooterVisual(wheel);
                }
                RepairDriveFrame(Require(robot.transform, "Intake/Back Link").GetComponent<ConfigurableJoint>());
                ApplyIntakeBeltRatio(robot);
                RepairFeedVisuals(robot);
                var settings = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
                CorrectPidPolarity(settings);
                settings.ApplyModifiedPropertiesWithoutUndo();
                RecordRuntimeCoreExceptions(robot, robot.GetComponent<BaywatchRobotics>());
                RepairIntakeAperture(robot);
                RepairKickerPassages(robot);
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log("MIRA runtime repair saved; intake angle, speeds, shot curves, and all other tuning preserved.");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        [MenuItem("Tools/Baywatch/Apply Feed, Speed, and Visual Tuning (Preserve Calibration)")]
        public static void ApplyFeedAndSpeedTuning()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                var script = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
                script.FindProperty("intakeMaxVelocity").floatValue = 120f;
                script.FindProperty("intakeMaxAcceleration").floatValue = 240f;
                script.FindProperty("intakeRpm").floatValue = 4800f;
                script.FindProperty("turretPid.Max").floatValue = 6.28f;
                script.ApplyModifiedPropertiesWithoutUndo();
                ApplyIntakeBeltRatio(robot);
                RepairFeedVisuals(robot);
                RepairIntakeAperture(robot);
                Validate(robot);
                PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
                Debug.Log("MIRA feed/speed tuning applied: intake rollers at 4800 RPM with the 17:31 crossbar belt, " +
                          "turret slew limited to 360 deg/s, kicker visuals spin on their real axle, indexer visuals enabled. " +
                          "Intake calibration preserved.");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        private static void RepairFeedVisuals(GameObject robot)
        {
            // CAD origins are not axle centres. Rotate only visual children around a dedicated
            // pivot at the physics axle; changing the axis alone still lets meshes orbit.
            foreach (string branchName in new[] { "Kickers", "Indexer", "Intake" })
                foreach (InteractionRoller roller in Require(robot.transform, branchName).GetComponentsInChildren<InteractionRoller>(true))
                {
                    var serialized = new SerializedObject(roller);
                    SerializedProperty models = serialized.FindProperty("models");
                    Check(models.arraySize > 0, branchName + " roller has no visual model: " + roller.name);
                    for (int i = 0; i < models.arraySize; i++)
                    {
                        SerializedProperty entry = models.GetArrayElementAtIndex(i);
                        var model = (GameObject)entry.FindPropertyRelative("model").objectReferenceValue;
                        Check(model != null, branchName + " visual reference is null: " + roller.name);
                        Check(!roller.transform.IsChildOf(model.transform), "Visual spin would move its own physics roller: " + roller.name);
                        Check(model.GetComponentsInChildren<Rigidbody>(true).Length == 0,
                            "Visual spin must not move rigidbodies: " + model.name);
                        Transform pivot = model.transform;
                        if (!model.name.StartsWith("MIRA_VisualSpin"))
                        {
                            pivot = new GameObject("MIRA_VisualSpin_" + model.name).transform;
                            pivot.SetParent(model.transform.parent, false);
                            pivot.gameObject.layer = model.layer;
                            pivot.SetPositionAndRotation(roller.transform.position, roller.transform.rotation);
                            if (model.GetComponentsInChildren<Collider>(true).Length == 0)
                                model.transform.SetParent(pivot, true);
                            else
                            {
                                // The indexer's Model also contains its solid collider assembly.
                                // Leave that assembly stationary and extract only mesh leaves.
                                MeshFilter[] meshes = model.GetComponentsInChildren<MeshFilter>(true);
                                Check(meshes.Length > 0, "No visual meshes in " + model.name);
                                foreach (MeshFilter mesh in meshes)
                                {
                                    Check(mesh.GetComponentsInChildren<Collider>(true).Length == 0,
                                        "Cannot animate a mesh that owns colliders: " + mesh.name);
                                    mesh.transform.SetParent(pivot, true);
                                }
                            }
                        }
                        if (branchName == "Indexer")
                        {
                            Transform cadFrame = roller.transform.Find("Model");
                            foreach (MeshFilter mesh in pivot.GetComponentsInChildren<MeshFilter>(true))
                            {
                                // This shaft was imported in centimetres while its siblings use
                                // metres (both translation and scale are 100x too large).
                                if (mesh.name != "3.14-1.002" || mesh.GetComponent<Renderer>().bounds.size.magnitude < 1f) continue;
                                Check(cadFrame != null, "Missing original indexer CAD frame.");
                                mesh.transform.SetParent(cadFrame, true);
                                mesh.transform.localPosition *= 0.01f;
                                mesh.transform.localScale *= 0.01f;
                                mesh.transform.SetParent(pivot, true);
                            }
                        }
                        entry.FindPropertyRelative("model").objectReferenceValue = pivot.gameObject;
                        entry.FindPropertyRelative("jointAxis").enumValueIndex = 0;
                        entry.FindPropertyRelative("reversed").boolValue = false;
                        if (Mathf.Abs(entry.FindPropertyRelative("multiplier").floatValue) < 0.0001f)
                            entry.FindPropertyRelative("multiplier").floatValue = 1f;
                    }
                    serialized.ApplyModifiedPropertiesWithoutUndo();
                }
        }

        private static void RepairIntakeAperture(GameObject robot)
        {
            Transform shape = Require(robot.transform, "Stationary/Drivetrain/Model/collider/Box (9)");
            if (shape.Find("MIRA_IntakeOpeningFloor") != null)
            {
                CopyApertureExclusions(robot, shape);
                return;
            }
            BoxCollider rear = shape.GetComponent<BoxCollider>();
            Vector3 size = rear.size;
            Check(Vector3.Dot(shape.right, robot.transform.forward) > 0.999f
                && Vector3.Dot(shape.up, -robot.transform.up) > 0.999f,
                "Chassis envelope axes changed; inspect intake opening before rebuilding it.");
            Check(Mathf.Abs(size.x - 0.7874f) < 0.001f && Mathf.Abs(size.z - 0.9144f) < 0.001f,
                "Chassis envelope changed; inspect intake aperture before rebuilding it.");
            // Box local +X is robot forward, -Y is up, Z is robot width. The old
            // solid envelope capped the entire mouth at z=0.403 m. Keep the rear
            // envelope and side bumpers, but leave a 0.68 m intake-width opening.
            const float mouthStart = 0.19f;
            const float openingWidth = 0.68f;
            float frontLength = size.x * 0.5f - mouthStart;
            Vector3 originalCenter = rear.center;
            rear.center = originalCenter + new Vector3((mouthStart - size.x * 0.5f) * 0.5f, 0f, 0f);
            rear.size = new Vector3(size.x - frontLength, size.y, size.z);
            float sideWidth = (size.z - openingWidth) * 0.5f;
            for (int sign = -1; sign <= 1; sign += 2)
                AddApertureBox(shape, "MIRA_IntakeOpeningSide" + sign,
                    originalCenter + new Vector3(mouthStart + frontLength * 0.5f, 0f, sign * (openingWidth + sideWidth) * 0.5f),
                    new Vector3(frontLength, size.y, sideWidth), rear);
            const float floorThickness = 0.02f;
            AddApertureBox(shape, "MIRA_IntakeOpeningFloor",
                originalCenter + new Vector3(mouthStart + frontLength * 0.5f, (size.y - floorThickness) * 0.5f, 0f),
                new Vector3(frontLength, floorThickness, openingWidth), rear);
            CopyApertureExclusions(robot, shape);
        }

        private static void CopyApertureExclusions(GameObject robot, Transform shape)
        {
            var settings = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
            SerializedProperty pairs = settings.FindProperty("collisionExclusions");
            Collider original = shape.GetComponent<Collider>();
            int originalCount = pairs.arraySize;
            foreach (Collider added in shape.GetComponentsInChildren<Collider>(true).Where(c => c != original))
                for (int i = 0; i < originalCount; i++)
                {
                    var pair = pairs.GetArrayElementAtIndex(i);
                    var first = (Collider)pair.FindPropertyRelative("first").objectReferenceValue;
                    var second = (Collider)pair.FindPropertyRelative("second").objectReferenceValue;
                    Collider other = first == original ? second : second == original ? first : null;
                    if (other == null) continue;
                    bool exists = Enumerable.Range(0, pairs.arraySize).Any(j =>
                    {
                        var existing = pairs.GetArrayElementAtIndex(j);
                        return existing.FindPropertyRelative("first").objectReferenceValue == added
                            && existing.FindPropertyRelative("second").objectReferenceValue == other;
                    });
                    if (exists) continue;
                    pairs.arraySize++;
                    var copy = pairs.GetArrayElementAtIndex(pairs.arraySize - 1);
                    copy.FindPropertyRelative("first").objectReferenceValue = added;
                    copy.FindPropertyRelative("second").objectReferenceValue = other;
                }
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void AddApertureBox(Transform parent, string name, Vector3 center, Vector3 size, BoxCollider source)
        {
            var part = new GameObject(name);
            part.layer = parent.gameObject.layer;
            part.transform.SetParent(parent, false);
            BoxCollider collider = part.AddComponent<BoxCollider>();
            collider.center = center;
            collider.size = size;
            collider.sharedMaterial = source.sharedMaterial;
            collider.contactOffset = source.contactOffset;
        }


        private static void RepairShooterVisual(InteractionRoller wheel)
        {
            // Generated rollers live under Part/DynamicPhysics/RollerGroup. Recover this
            // wheel's own tread mesh, never an assembly ancestor that also owns colliders.
            Transform part = wheel.transform.parent.parent.parent;
            MeshFilter[] meshes = part.GetComponentsInChildren<MeshFilter>(true);
            Check(meshes.Length == 1, "Expected one tread mesh for shooter roller: " + part.name);
            Check(meshes[0].GetComponentsInChildren<Collider>(true).Length == 0,
                "Visual spin pivot must not animate collider geometry: " + part.name);
            Transform pivot = part.Find("MIRA_VisualSpin");
            if (pivot == null)
            {
                pivot = new GameObject("MIRA_VisualSpin").transform;
                pivot.SetParent(part, false);
                pivot.gameObject.layer = part.gameObject.layer;
                pivot.SetPositionAndRotation(wheel.transform.position, wheel.transform.rotation);
                Vector3 parentScale = part.lossyScale;
                pivot.localScale = new Vector3(1f / parentScale.x, 1f / parentScale.y, 1f / parentScale.z);
                meshes[0].transform.SetParent(pivot, true);
            }
            var serialized = new SerializedObject(wheel);
            SerializedProperty models = serialized.FindProperty("models");
            models.arraySize = 1;
            SerializedProperty model = models.GetArrayElementAtIndex(0);
            model.FindPropertyRelative("model").objectReferenceValue = pivot.gameObject;
            model.FindPropertyRelative("jointAxis").enumValueIndex = 0; // local X is the axle
            model.FindPropertyRelative("reversed").boolValue = false;
            model.FindPropertyRelative("multiplier").floatValue = 1f;
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static RollerGroup MakeGroup(Transform parent, InteractionRoller[] rollers, Func<InteractionRoller, float> direction)
        {
            Check(rollers.Length > 0, parent.name + ": empty functional roller group.");
            var group = parent.gameObject.AddComponent<RollerGroup>();
            var serialized = new SerializedObject(group);
            SerializedProperty list = serialized.FindProperty("rollers");
            list.arraySize = rollers.Length;
            for (int i = 0; i < rollers.Length; i++)
            {
                SerializedProperty entry = list.GetArrayElementAtIndex(i);
                PvffController controller = rollers[i].GetComponent<PvffController>();
                entry.FindPropertyRelative("IntRoller").objectReferenceValue = controller == null ? rollers[i] : null;
                entry.FindPropertyRelative("PvffController").objectReferenceValue = controller;
                entry.FindPropertyRelative("GenRoller").objectReferenceValue = null;
                entry.FindPropertyRelative("Flywheel").objectReferenceValue = null;
                entry.FindPropertyRelative("Multiplier").floatValue = 1f;
                entry.FindPropertyRelative("flipped").boolValue = direction(rollers[i]) < 0f;
            }
            serialized.ApplyModifiedPropertiesWithoutUndo();
            return group;
        }

        // Real robot: the two TPU-covered NEO 2.0 rollers are direct driven and run at motor
        // speed. A 17T pulley belts the long crossbar to a 31T pulley, so the crossbar turns
        // at 17/31 of the motors while all three share one commanded RPM.
        private const float IntakeBeltRatio = 17f / 31f;

        private static void ApplyIntakeBeltRatio(GameObject robot)
        {
            RollerGroup group = Require(robot.transform, "Intake").GetComponentInChildren<RollerGroup>(true);
            Check(group != null, "Intake has no functional roller group.");
            var serialized = new SerializedObject(group);
            SerializedProperty rollers = serialized.FindProperty("rollers");
            int crossbars = 0;
            for (int i = 0; i < rollers.arraySize; i++)
            {
                SerializedProperty entry = rollers.GetArrayElementAtIndex(i);
                var roller = (InteractionRoller)entry.FindPropertyRelative("IntRoller").objectReferenceValue;
                if (roller == null) continue;
                float length = new SerializedObject(roller).FindProperty("length").floatValue;
                if (length <= 10f) continue; // the motor rollers are about 3 in wide
                entry.FindPropertyRelative("Multiplier").floatValue = IntakeBeltRatio;
                crossbars++;
            }
            Check(crossbars == 1, "Expected exactly one belt-driven intake crossbar roller, found " + crossbars + ".");
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void ReorientPreservingChildren(Transform root, Quaternion rotation)
        {
            Check(root.GetComponents<Collider>().Length == 0, root.name + ": root colliders need explicit geometry conversion.");
            Transform[] children = root.Cast<Transform>().ToArray();
            var positions = children.Select(c => c.position).ToArray();
            var rotations = children.Select(c => c.rotation).ToArray();
            Matrix4x4[] before = root.GetComponentsInChildren<MeshFilter>(true).Select(m => m.transform.localToWorldMatrix).ToArray();
            root.rotation = rotation;
            for (int i = 0; i < children.Length; i++) children[i].SetPositionAndRotation(positions[i], rotations[i]);
            MeshFilter[] meshes = root.GetComponentsInChildren<MeshFilter>(true);
            for (int i = 0; i < meshes.Length; i++)
            {
                Matrix4x4 after = meshes[i].transform.localToWorldMatrix;
                for (int element = 0; element < 16; element++)
                    Check(Mathf.Abs(before[i][element] - after[element]) < 0.001f, "Turret normalization changed CAD geometry: " + meshes[i].name);
            }
        }

        private static void ConfigureDrive(ConfigurableJoint joint, bool turret)
        {
            Check(joint != null && joint.connectedBody != null, "Powered joint is missing its chassis connection.");
            RepairDriveFrame(joint);
            joint.xMotion = joint.yMotion = joint.zMotion = ConfigurableJointMotion.Locked;
            joint.angularXMotion = turret ? ConfigurableJointMotion.Locked : ConfigurableJointMotion.Free;
            joint.angularYMotion = turret ? ConfigurableJointMotion.Free : ConfigurableJointMotion.Locked;
            joint.angularZMotion = ConfigurableJointMotion.Locked;
            joint.rotationDriveMode = RotationDriveMode.XYAndZ;
            var drive = new JointDrive { positionSpring = 0f, positionDamper = turret ? 20f : 100f, maximumForce = turret ? 25f : 80f };
            if (turret) joint.angularYZDrive = drive;
            else joint.angularXDrive = drive;
            MatchConnectedAnchor(joint);
            joint.enableCollision = false;
        }

        private static void RepairDriveFrame(ConfigurableJoint joint)
        {
            // Keep a conventional joint frame. Drive polarity belongs in the feedback gains,
            // not a flipped encoder or an imported/tilted parent coordinate system.
            joint.axis = Vector3.right;
            joint.secondaryAxis = Vector3.up;
            joint.swapBodies = false;
        }

        private static void CorrectPidPolarity(SerializedObject robot)
        {
            foreach (string field in new[] { "intakePid", "turretPid" })
                foreach (string gain in new[] { "kP", "kI", "kD" })
                {
                    SerializedProperty value = robot.FindProperty(field + "." + gain);
                    value.floatValue = Mathf.Abs(value.floatValue);
                }
        }

        private static void RecordRuntimeCoreExceptions(GameObject robot, BaywatchRobotics script)
        {
            var serialized = new SerializedObject(script);
            SerializedProperty list = serialized.FindProperty("runtimeCoreExclusions");
            list.arraySize = 2;
            InteractionRoller intakeRoller = Require(robot.transform, "Intake/Main Link/Roller/DynamicPhysics/RollerGroup/Roller").GetComponent<InteractionRoller>();
            string[] guides = { "3.01-1", "3.01-1.001" };
            for (int i = 0; i < guides.Length; i++)
            {
                SerializedProperty pair = list.GetArrayElementAtIndex(i);
                pair.FindPropertyRelative("roller").objectReferenceValue = intakeRoller;
                pair.FindPropertyRelative("other").objectReferenceValue = Require(robot.transform,
                    "Stationary/Hopper/occurrence of " + guides[i] + "/Model/collider/Box").GetComponent<Collider>();
            }
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void MatchConnectedAnchor(Joint joint)
        {
            Check(joint.connectedBody != null, "Cannot anchor a linkage to the stationary world.");
            joint.autoConfigureConnectedAnchor = false;
            joint.connectedAnchor = joint.connectedBody.transform.InverseTransformPoint(joint.transform.TransformPoint(joint.anchor));
        }

        private static void RecordOverlapExceptions(GameObject robot, BaywatchRobotics script)
        {
            Collider[] chassis = robot.GetComponentsInChildren<Collider>(true)
                .Where(c => c.attachedRigidbody == robot.GetComponent<Rigidbody>()).ToArray();
            Joint[] joints = robot.GetComponentsInChildren<Joint>(true);
            var pairs = new List<(Collider first, Collider second)>();
            foreach (Collider a in Require(robot.transform, "Intake").GetComponentsInChildren<Collider>(true))
                foreach (Collider b in chassis)
                {
                    if (!a.enabled || !b.enabled || a.isTrigger || b.isTrigger || a.attachedRigidbody == b.attachedRigidbody) continue;
                    if (joints.Any(j => !j.enableCollision && j.GetComponent<Rigidbody>() == a.attachedRigidbody && j.connectedBody == b.attachedRigidbody)) continue;
                    if (Physics.ComputePenetration(a, a.transform.position, a.transform.rotation, b, b.transform.position, b.transform.rotation,
                        out Vector3 direction, out float depth) && depth > 0.0005f)
                        pairs.Add((a, b));
                }
            // The frame box intrudes into the two couplers' swept path. The bench found
            // additional contacts near deployment, so exclude this ONE frame collider against
            // those two compound shapes, not the intake against the robot layer as a whole.
            Collider frameBox = Require(robot.transform, "Stationary/Drivetrain/Model/collider/Box (9)").GetComponent<Collider>();
            foreach (string coupler in new[] { "2.01-3", "2.01-3.001" })
            {
                foreach (Collider linkBox in Require(robot.transform, "Intake/Main Link/Model/collider/collider-complex-" + coupler).GetComponentsInChildren<Collider>(true))
                    if (!pairs.Any(p => p.first == linkBox && p.second == frameBox)) pairs.Add((linkBox, frameBox));
                // Near 43 degrees the hopper guide's box approximation also blocks the
                // small coupler box at its cutout. Keep this exception local to those shapes.
                Collider couplerBox = Require(robot.transform, "Intake/Main Link/Model/collider/collider-complex-" + coupler + "/collider-box (8)").GetComponent<Collider>();
                Collider hopperGuide = Require(robot.transform, "Intake/Hopper/occurrence of 3.02-1/Model/collider/Box").GetComponent<Collider>();
                pairs.Add((couplerBox, hopperGuide));
            }
            var serialized = new SerializedObject(script);
            SerializedProperty list = serialized.FindProperty("collisionExclusions");
            list.arraySize = pairs.Count;
            for (int i = 0; i < pairs.Count; i++)
            {
                list.GetArrayElementAtIndex(i).FindPropertyRelative("first").objectReferenceValue = pairs[i].first;
                list.GetArrayElementAtIndex(i).FindPropertyRelative("second").objectReferenceValue = pairs[i].second;
            }
            serialized.ApplyModifiedPropertiesWithoutUndo();
            Debug.Log($"MIRA: recorded {pairs.Count} specific same-robot collider overlap exceptions.");
        }

        private static bool Has955Ancestor(Transform item, Transform stop)
        {
            for (Transform t = item; t != null && t != stop; t = t.parent)
                if (t.name.Contains("9.55")) return true;
            return false;
        }

        private static bool AncestorNamed(Transform item, Transform stop, string name)
        {
            for (Transform t = item; t != null && t != stop; t = t.parent)
                if (t.name == name) return true;
            return false;
        }

        [MenuItem("Tools/Baywatch/Validate MIRA Prefab")]
        public static void ValidatePrefab()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try { Validate(robot); ValidateControlLogic(robot); }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
            Debug.Log("MIRA validation passed: references, joint topology/drives, axes, retained rollers, PVFF ownership, intake profile, and independent feed gating.");
        }

        private static void Validate(GameObject robot)
        {
            Check(robot.GetComponents<BaywatchRobotics>().Length == 1, "Expected exactly one MIRA robot controller.");
            Transform intake = Require(robot.transform, "Intake");
            Check(intake.GetComponentsInChildren<Rigidbody>(true).Length == 5, "Intake must have five moving bodies.");
            Check(intake.GetComponentsInChildren<HingeJoint>(true).Length == 6, "Intake must have six passive hinges.");
            Check(intake.GetComponentsInChildren<ConfigurableJoint>(true).Length == 1, "Intake must have one powered joint.");
            foreach (Joint joint in intake.GetComponentsInChildren<Joint>(true))
            {
                Check(joint.connectedBody != null, "Joint attaches to world instead of a robot body: " + joint.name);
                Check(Vector3.Distance(joint.transform.TransformPoint(joint.anchor), joint.connectedBody.transform.TransformPoint(joint.connectedAnchor)) < 0.0001f,
                    "Linkage pin anchors do not coincide: " + joint.name);
            }
            foreach (string name in new[] { "Left", "Right" })
            {
                Transform turret = Require(robot.transform, "Turrets/" + name);
                Check(Vector3.Dot(turret.up, robot.transform.up) > 0.999f, name + ": Y axis is not vertical.");
                Check(Vector3.Dot(turret.parent.up, robot.transform.up) > 0.999f,
                    name + ": the encoder parent frame is not aligned to the chassis.");
                Check(turret.GetComponent<ConfigurableJoint>().angularYZDrive.positionDamper > 0f, name + ": no powered angular drive.");
                PvffController[] controllers = turret.GetComponentsInChildren<PvffController>(true);
                Check(controllers.Length == 2, name + ": both shooter wheels need PVFF controllers.");
                foreach (PvffController controller in controllers)
                {
                    Check(new SerializedObject(controller).FindProperty("m_GamePieceLayers").intValue != 0, "PVFF load sensing must filter fuel layers.");
                    var roller = controller.GetComponent<InteractionRoller>();
                    var rollerSettings = new SerializedObject(roller);
                    Check(rollerSettings.FindProperty("maxSquish").floatValue
                        >= rollerSettings.FindProperty("radius").floatValue - 0.05f,
                        "Shooter wheel keeps a solid core; rigid fuel cannot pass the fixed hood.");
                    SerializedProperty models = rollerSettings.FindProperty("models");
                    Check(models.arraySize > 0, "Shooter has no visual model.");
                    for (int i = 0; i < models.arraySize; i++)
                    {
                        var model = (GameObject)models.GetArrayElementAtIndex(i).FindPropertyRelative("model").objectReferenceValue;
                        Check(model != null, "Shooter visual reference is null.");
                        Check(!roller.transform.IsChildOf(model.transform), "Shooter animation owns a physics ancestor.");
                        Check(model.GetComponentsInChildren<Collider>(true).Length == 0, "Shooter animation would move colliders.");
                        Check(Vector3.Distance(model.transform.position, roller.transform.position) < 0.0001f,
                            "Shooter visual pivot is not centred on its axle.");
                    }
                }
            }
            Check(Require(robot.transform, "Intake/Back Link").GetComponent<ConfigurableJoint>().angularXDrive.positionDamper > 0f, "Intake has no powered drive.");
            ValidateIntakeBeltRatio(intake);
            Transform kickers = Require(robot.transform, "Kickers");
            foreach (InteractionRoller roller in kickers.GetComponentsInChildren<InteractionRoller>(true))
            {
                Check(Has955Ancestor(roller.transform, kickers), "Unwanted kicker roller remains: " + roller.name);
                Check(!AnimationUtility.CalculateTransformPath(roller.transform, kickers).Contains("am-3948-1000"),
                    "Kicker spacer must not have an interaction roller: " + roller.name);
                ValidateFeedVisual(roller, "Kicker");
            }
            foreach (InteractionRoller roller in Require(robot.transform, "Indexer").GetComponentsInChildren<InteractionRoller>(true))
                ValidateFeedVisual(roller, "Indexer");
            foreach (InteractionRoller roller in intake.GetComponentsInChildren<InteractionRoller>(true))
                ValidateFeedVisual(roller, "Intake");

            var script = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
            foreach (string field in new[] { "intakePivot", "intakeRollers", "left.turret", "right.turret", "left.swmReference", "right.swmReference", "left.flywheels", "right.flywheels", "left.kickers", "right.kickers", "left.indexer", "right.indexer" })
                Check(script.FindProperty(field).objectReferenceValue != null, "Missing reference: " + field);

            var ownership = new Dictionary<Object, int>();
            foreach (RollerGroup group in robot.GetComponentsInChildren<RollerGroup>(true))
            {
                SerializedProperty entries = new SerializedObject(group).FindProperty("rollers");
                Check(entries.arraySize > 0, "Empty roller group: " + group.name);
                for (int i = 0; i < entries.arraySize; i++)
                {
                    SerializedProperty entry = entries.GetArrayElementAtIndex(i);
                    Object controller = entry.FindPropertyRelative("PvffController").objectReferenceValue;
                    Object roller = controller != null ? ((PvffController)controller).GetComponent<InteractionRoller>() : entry.FindPropertyRelative("IntRoller").objectReferenceValue;
                    Check(roller != null, "Broken roller group entry.");
                    ownership[roller] = ownership.TryGetValue(roller, out int count) ? count + 1 : 1;
                }
            }
            foreach (int count in ownership.Values) Check(count == 1, "A roller has more than one command owner.");
        }

        private static void ValidateIntakeBeltRatio(Transform intake)
        {
            RollerGroup group = intake.GetComponentInChildren<RollerGroup>(true);
            Check(group != null, "Intake has no functional roller group.");
            SerializedProperty rollers = new SerializedObject(group).FindProperty("rollers");
            int crossbars = 0;
            for (int i = 0; i < rollers.arraySize; i++)
            {
                SerializedProperty entry = rollers.GetArrayElementAtIndex(i);
                var roller = (InteractionRoller)entry.FindPropertyRelative("IntRoller").objectReferenceValue;
                if (roller == null) continue;
                float length = new SerializedObject(roller).FindProperty("length").floatValue;
                float multiplier = entry.FindPropertyRelative("Multiplier").floatValue;
                if (length <= 10f)
                {
                    Check(Mathf.Abs(multiplier - 1f) < 0.0001f, "Intake motor roller should run at motor speed: " + roller.name);
                    continue;
                }
                Check(Mathf.Abs(multiplier - IntakeBeltRatio) < 0.0001f,
                    "Intake crossbar is not belt-reduced 17:31: " + roller.name);
                crossbars++;
            }
            Check(crossbars == 1, "Expected exactly one belt-driven intake crossbar roller.");
        }

        private static void ValidateFeedVisual(InteractionRoller roller, string label)
        {
            SerializedProperty models = new SerializedObject(roller).FindProperty("models");
            Check(models.arraySize > 0, label + " has no visual model: " + roller.name);
            for (int i = 0; i < models.arraySize; i++)
            {
                SerializedProperty entry = models.GetArrayElementAtIndex(i);
                var model = (GameObject)entry.FindPropertyRelative("model").objectReferenceValue;
                Check(model != null, label + " visual reference is null: " + roller.name);
                Check(model.name == "MIRA_VisualSpin_Drive" || Vector3.Distance(model.transform.position, roller.transform.position) < 0.0001f,
                    label + " visual pivot is not centred on its axle: " + roller.name);
                Check(!roller.transform.IsChildOf(model.transform), label + " visual owns its physics roller.");
                Check(model.GetComponentsInChildren<Collider>(true).Length == 0, label + " visual owns colliders.");
                Vector3 axle = model.transform.InverseTransformDirection(roller.transform.right).normalized;
                int dominant = Mathf.Abs(axle.x) >= Mathf.Abs(axle.y) && Mathf.Abs(axle.x) >= Mathf.Abs(axle.z)
                    ? 0 : Mathf.Abs(axle.y) >= Mathf.Abs(axle.z) ? 1 : 2;
                Check(Mathf.Abs(axle[dominant]) > 0.98f, label + " axle is not axis-aligned: " + roller.name);
                Check(entry.FindPropertyRelative("jointAxis").enumValueIndex == dominant,
                    label + " visual spins on the wrong local axis: " + roller.name);
                Check(Mathf.Abs(entry.FindPropertyRelative("multiplier").floatValue) > 0.0001f,
                    label + " visual multiplier is zero: " + roller.name);
            }
        }

        private static void ValidateControlLogic(GameObject robot)
        {
            var script = robot.GetComponent<BaywatchRobotics>();
            Type type = typeof(BaywatchRobotics);
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
            MethodInfo profile = type.GetMethod("StepIntakeProfile", flags);
            FieldInfo angle = type.GetField("intakeCommandAngle", flags);
            FieldInfo velocity = type.GetField("intakeCommandVelocity", flags);
            // Synthetic profile endpoints exercise the algorithm, not the real deployed angle.
            foreach (float target in new[] { 90f, 0f, 45f, 0f })
            {
                for (int i = 0; i < 300; i++)
                {
                    profile.Invoke(script, new object[] { target, 0.02f });
                    float value = (float)angle.GetValue(script);
                    Check(value >= -0.001f && value <= 90.001f, "Intake profile overshot synthetic travel.");
                    Check(Mathf.Abs((float)velocity.GetValue(script)) <= (float)type.GetField("intakeMaxVelocity", flags).GetValue(script) + 0.001f, "Intake profile exceeded velocity.");
                }
                Check(Mathf.Abs((float)angle.GetValue(script) - target) < 0.001f, "Intake profile did not reach its endpoint.");
            }

            object left = type.GetField("left", flags).GetValue(script);
            object right = type.GetField("right", flags).GetValue(script);
            Type sideType = left.GetType();
            foreach (bool leftReady in new[] { false, true })
                foreach (bool rightReady in new[] { false, true })
                {
                    sideType.GetField("ready").SetValue(left, leftReady);
                    sideType.GetField("ready").SetValue(right, rightReady);
                    foreach (object side in new[] { left, right })
                        ((InteractionRoller)sideType.GetField("indexer").GetValue(side)).SetRpm(0f);
                    type.GetMethod("Feed", flags).Invoke(script, null);
                    Check((((InteractionRoller)sideType.GetField("indexer").GetValue(left)).TargetRpm != 0f) == leftReady, "Left feed gate is not independent.");
                    Check((((InteractionRoller)sideType.GetField("indexer").GetValue(right)).TargetRpm != 0f) == rightReady, "Right feed gate is not independent.");
                }
            foreach (object side in new[] { left, right })
            {
                type.GetMethod("PrepareSide", flags).Invoke(script, new[] { side, (object)false });
                Check(((InteractionRoller)sideType.GetField("indexer").GetValue(side)).TargetRpm == 0f, "Releasing shoot left the indexer running.");
                Check(!(bool)sideType.GetField("ready").GetValue(side), "Readiness survived release of shoot.");
            }
            type.GetField("intakeDeployedAngle", flags).SetValue(script, 0f);
            type.GetMethod("RunMechanisms", flags).Invoke(script, new object[] { true, false });
            Check(!(bool)type.GetField("intakeDeployed", flags).GetValue(script), "Uncalibrated intake was allowed to deploy.");
        }

        // Isolated edit-mode bench: no field, no user scene changes, and no gameplay scripts run.
        // Useful for finding closed-loop geometry/collider problems before testing with fuel.
        public static void ProbePhysics()
        {
            Check(Application.isBatchMode, "Run the physics bench in a separate Unity batch process, not a user's open scene.");
            UnityEditor.SceneManagement.EditorSceneManager.NewScene(UnityEditor.SceneManagement.NewSceneSetup.EmptyScene,
                UnityEditor.SceneManagement.NewSceneMode.Single);
            SimulationMode oldMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            GameObject robot = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath));
            try
            {
                foreach (MonoBehaviour component in robot.GetComponentsInChildren<MonoBehaviour>(true)) component.enabled = false;
                foreach (Rigidbody body in robot.GetComponentsInChildren<Rigidbody>(true))
                {
                    body.useGravity = false;
                    body.solverIterations = 64;
                    body.solverVelocityIterations = 16;
                }
                robot.GetComponent<Rigidbody>().isKinematic = true;
                if (Environment.GetEnvironmentVariable("MIRA_BENCH_NO_COLLISIONS") == "1")
                    foreach (Collider collider in robot.GetComponentsInChildren<Collider>(true)) collider.enabled = false;
                if (Environment.GetEnvironmentVariable("MIRA_BENCH_NO_HINGES") == "1")
                    foreach (HingeJoint hinge in robot.GetComponentsInChildren<HingeJoint>(true)) Object.DestroyImmediate(hinge);
                PhysicsScene physics = Physics.defaultPhysicsScene;
                ConfigurableJoint intake = Require(robot.transform, "Intake/Back Link").GetComponent<ConfigurableJoint>();
                BaywatchRobotics control = robot.GetComponent<BaywatchRobotics>();
                const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
                typeof(BaywatchRobotics).GetMethod("ApplyCollisionExclusions", flags).Invoke(control, null);
                MethodInfo stepProfile = typeof(BaywatchRobotics).GetMethod("StepIntakeProfile", flags);
                FieldInfo commandAngle = typeof(BaywatchRobotics).GetField("intakeCommandAngle", flags);
                bool motionPassed = true;
                // Small bench motions only; the actual deployed angle must be calibrated by the owner.
                foreach (float target in new[] { 5f, 45f, 0f })
                {
                    for (int i = 0; i < 500; i++)
                    {
                        // Euler X folds back after 90 degrees; use the same quaternion encoder
                        // as the real controller, including synthetic tests beyond 90 degrees.
                        float current = Mathf.DeltaAngle(0f, intake.GetComponent<GenericJoint>().GetSingleAxisAngle(JointAxis.X));
                        stepProfile.Invoke(control, new object[] { target, 0.02f });
                        float command = (float)commandAngle.GetValue(control);
                        intake.targetAngularVelocity = Vector3.right * Mathf.Clamp(Mathf.DeltaAngle(current, command) * 0.15f, -4.2f, 4.2f);
                        physics.Simulate(0.02f);
                    }
                    float anchorError = robot.GetComponentsInChildren<Joint>(true).Max(j => Vector3.Distance(j.transform.TransformPoint(j.anchor), j.connectedBody.transform.TransformPoint(j.connectedAnchor)));
                    float measured = Mathf.DeltaAngle(0f, intake.GetComponent<GenericJoint>().GetSingleAxisAngle(JointAxis.X));
                    Debug.Log($"MIRA physics bench: intake target {target}, measured {measured:F2} degrees, angular velocity {intake.GetComponent<Rigidbody>().angularVelocity}, max anchor error {anchorError:F4}m.");
                    motionPassed &= Mathf.Abs(Mathf.DeltaAngle(measured, target)) < 2f && anchorError < 0.005f;
                    if (target == 45f) DescribeBlockingContacts(robot);
                }
                foreach (string name in new[] { "Left", "Right" })
                {
                    ConfigurableJoint turret = Require(robot.transform, "Turrets/" + name).GetComponent<ConfigurableJoint>();
                    float start = turret.transform.localEulerAngles.y;
                    float target = start + 45f;
                    for (int i = 0; i < 300; i++)
                    {
                        float error = Mathf.DeltaAngle(turret.transform.localEulerAngles.y, target);
                        turret.targetAngularVelocity = Vector3.up * Mathf.Clamp(error * 0.2f, -7f, 7f);
                        physics.Simulate(0.02f);
                    }
                    Debug.Log($"MIRA physics bench: {name} turret requested +45 degrees, measured {Mathf.DeltaAngle(start, turret.transform.localEulerAngles.y):F2}, up alignment {Vector3.Dot(turret.transform.up, Vector3.up):F4}.");
                    motionPassed &= Mathf.Abs(Mathf.DeltaAngle(turret.transform.localEulerAngles.y, target)) < 1f
                        && Vector3.Dot(turret.transform.up, Vector3.up) > 0.999f;
                }
                Check(motionPassed, "MIRA physics bench failed: powered motion did not reach its target or linkage pins separated.");
            }
            finally
            {
                Object.DestroyImmediate(robot);
                Physics.simulationMode = oldMode;
            }
        }

        public static void InspectInterference()
        {
            GameObject robot = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                Collider[] colliders = robot.GetComponentsInChildren<Collider>(true);
                Joint[] joints = robot.GetComponentsInChildren<Joint>(true);
                var overlaps = new List<(float depth, string pair)>();
                foreach (Collider a in Require(robot.transform, "Intake").GetComponentsInChildren<Collider>(true))
                    foreach (Collider b in colliders)
                    {
                        if (a == b || a.attachedRigidbody == b.attachedRigidbody || a.isTrigger || b.isTrigger) continue;
                        if (joints.Any(j => !j.enableCollision && ((j.GetComponent<Rigidbody>() == a.attachedRigidbody && j.connectedBody == b.attachedRigidbody)
                            || (j.GetComponent<Rigidbody>() == b.attachedRigidbody && j.connectedBody == a.attachedRigidbody)))) continue;
                        if (Physics.ComputePenetration(a, a.transform.position, a.transform.rotation, b, b.transform.position, b.transform.rotation,
                            out Vector3 direction, out float depth))
                            overlaps.Add((depth, AnimationUtility.CalculateTransformPath(a.transform, robot.transform) + " <> "
                                + AnimationUtility.CalculateTransformPath(b.transform, robot.transform)));
                    }
                foreach (var overlap in overlaps.OrderByDescending(o => o.depth).Take(30))
                    Debug.Log($"MIRA overlap {overlap.depth:F4}m: {overlap.pair}");
            }
            finally { PrefabUtility.UnloadPrefabContents(robot); }
        }

        private static void DescribeBlockingContacts(GameObject robot)
        {
            Collider[] chassis = robot.GetComponentsInChildren<Collider>(true);
            Joint[] joints = robot.GetComponentsInChildren<Joint>(true);
            var pairs = new List<(float depth, string path)>();
            foreach (Collider a in Require(robot.transform, "Intake").GetComponentsInChildren<Collider>(true))
                foreach (Collider b in chassis)
                {
                    if (!a.enabled || !b.enabled || a.isTrigger || b.isTrigger || a.attachedRigidbody == b.attachedRigidbody || Physics.GetIgnoreCollision(a, b)) continue;
                    if (joints.Any(j => !j.enableCollision && ((j.GetComponent<Rigidbody>() == a.attachedRigidbody && j.connectedBody == b.attachedRigidbody)
                        || (j.GetComponent<Rigidbody>() == b.attachedRigidbody && j.connectedBody == a.attachedRigidbody)))) continue;
                    if (Physics.ComputePenetration(a, a.transform.position, a.transform.rotation, b, b.transform.position, b.transform.rotation, out Vector3 direction, out float depth))
                        pairs.Add((depth, AnimationUtility.CalculateTransformPath(a.transform, robot.transform) + " <> " + AnimationUtility.CalculateTransformPath(b.transform, robot.transform)));
                }
            foreach (var pair in pairs.OrderByDescending(p => p.depth).Take(10)) Debug.Log($"MIRA deployed overlap {pair.depth:F4}m: {pair.path}");
        }

        private static Transform Require(Transform root, string path)
        {
            Transform found = root.Find(path);
            Check(found != null, "Missing MIRA object: " + path);
            return found;
        }

        private static void Check(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }
    }
}
