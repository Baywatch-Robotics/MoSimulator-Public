#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MoSimCore.BaseClasses.GameManagement;
using MoSimCore.BaseClasses.GameManagement.TimerManagement;
using MoSimCore.Enums;
using NUnit.Framework;
using RobotFramework.Components;
using RobotFramework.Controllers.PidSystems;
using RobotFramework.Enums;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace Baywatch.Tests
{
    public class MiraTestTimer : BaseTimerManager
    {
        protected override float MatchDuration => 1000f;
        protected override float TeleopStartTime => 1000f;
        protected override float EndgameStartTime => 0f;
        protected override void StartTeleopTransition() { }
        public override void StartMatch()
        {
            CurrentRobotState = RobotState.Enabled;
            CurrentGameState = GameState.Teleop;
            IsCountingDown = false;
        }
        protected override void Update() { }
    }

    public class MiraTestManager : BaseGameManager
    {
        protected override void InitializeComponents() => TimerManager = GetComponent<MiraTestTimer>();
        protected override void OnGameStateChanged(GameState state) { }
        protected override IEnumerator PerformGameSpecificReset() { yield break; }
    }

    public class MiraRuntimeTests
    {
        private const string PrefabPath = "Assets/Prefabs/Games/Rebuilt/Robots/Mods/Baywatch/BaywatchRobotics/MIRA.prefab";
        private GameObject fixture;
        private GameObject robot;
        private readonly List<(Collider first, Collider second)> exclusions = new();
        private readonly List<(InteractionRoller roller, Collider other)> coreExclusions = new();
        private SimulationMode oldSimulationMode;
        private PidConstants intakePid;
        private PidConstants turretPid;
        private float intakeVelocity;
        private float intakeAcceleration;
        private float intakeRpm;
        private BaywatchRobotics productionController;

        private IEnumerator CreateRobot(bool animateModels)
        {
            oldSimulationMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            fixture = new GameObject("MIRA runtime test");
            GameObject manager = new GameObject("Test game state");
            manager.transform.SetParent(fixture.transform);
            manager.AddComponent<MiraTestTimer>();
            manager.AddComponent<MiraTestManager>();

            GameObject staging = new GameObject("Inactive staging");
            staging.transform.SetParent(fixture.transform);
            staging.SetActive(false);
            robot = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath), staging.transform);
            var controller = new SerializedObject(robot.GetComponent<BaywatchRobotics>());
            intakePid = ReadPid(controller.FindProperty("intakePid"));
            turretPid = ReadPid(controller.FindProperty("turretPid"));
            intakeVelocity = controller.FindProperty("intakeMaxVelocity").floatValue;
            intakeAcceleration = controller.FindProperty("intakeMaxAcceleration").floatValue;
            intakeRpm = controller.FindProperty("intakeRpm").floatValue;
            var commands = new GameObject("Inactive production commands");
            commands.transform.SetParent(fixture.transform);
            commands.SetActive(false);
            productionController = commands.AddComponent<BaywatchRobotics>();
            EditorUtility.CopySerialized(robot.GetComponent<BaywatchRobotics>(), productionController);
            SerializedProperty pairs = controller.FindProperty("collisionExclusions");
            for (int i = 0; i < pairs.arraySize; i++)
            {
                SerializedProperty pair = pairs.GetArrayElementAtIndex(i);
                exclusions.Add(((Collider)pair.FindPropertyRelative("first").objectReferenceValue,
                    (Collider)pair.FindPropertyRelative("second").objectReferenceValue));
            }
            SerializedProperty corePairs = controller.FindProperty("runtimeCoreExclusions");
            for (int i = 0; i < corePairs.arraySize; i++)
            {
                SerializedProperty pair = corePairs.GetArrayElementAtIndex(i);
                coreExclusions.Add(((InteractionRoller)pair.FindPropertyRelative("roller").objectReferenceValue,
                    (Collider)pair.FindPropertyRelative("other").objectReferenceValue));
            }
            // Keep the actual joint/turret/roller/PVFF lifecycles, but not field spawning,
            // player input, or the robot scheduler. Commands below use the production APIs.
            foreach (MonoBehaviour component in robot.GetComponentsInChildren<MonoBehaviour>(true))
                if (component is not GenericJoint && component is not GenericTurret
                    && component is not InteractionRoller && component is not PvffController
                    && component is not RollerGroup)
                    Object.DestroyImmediate(component);
            if (!animateModels)
                foreach (InteractionRoller roller in robot.GetComponentsInChildren<InteractionRoller>(true))
                {
                    var settings = new SerializedObject(roller);
                    settings.FindProperty("models").arraySize = 0;
                    settings.ApplyModifiedPropertiesWithoutUndo();
                }
            foreach (Rigidbody body in robot.GetComponentsInChildren<Rigidbody>(true))
            {
                body.useGravity = false;
                body.solverIterations = 64;
                body.solverVelocityIterations = 16;
                body.maxAngularVelocity = 7f;
            }
            robot.GetComponent<Rigidbody>().isKinematic = true;
            robot.transform.SetParent(fixture.transform, true);
            yield return null;
            foreach (var pair in exclusions)
                if (pair.first && pair.second) Physics.IgnoreCollision(pair.first, pair.second);
            foreach (Collider collider in robot.transform.Find("Intake").GetComponentsInChildren<Collider>(true))
                collider.contactOffset = 0.0005f;
            foreach (var pair in coreExclusions)
            {
                Collider core = pair.roller.transform.Find("Wheel_InternalCore").GetComponent<Collider>();
                Physics.IgnoreCollision(core, pair.other);
            }
            Physics.simulationMode = oldSimulationMode;
            yield return new WaitForFixedUpdate();
        }

        private static PidConstants ReadPid(SerializedProperty value)
        {
            return new PidConstants(value.FindPropertyRelative("kP").floatValue, value.FindPropertyRelative("kI").floatValue,
                value.FindPropertyRelative("kD").floatValue, value.FindPropertyRelative("Max").floatValue,
                value.FindPropertyRelative("Isaturation").floatValue);
        }

        [UnityTearDown]
        public IEnumerator Cleanup()
        {
            if (fixture) Object.Destroy(fixture);
            exclusions.Clear();
            coreExclusions.Clear();
            Physics.simulationMode = oldSimulationMode;
            yield return null;
        }

        [UnityTest]
        public IEnumerator TurretsHoldTargetsWhileChassisTurns()
        {
            yield return CreateRobot(false);
            GenericTurret[] turrets = robot.GetComponentsInChildren<GenericTurret>();
            foreach (GenericTurret turret in turrets)
            {
                turret.SetPid(turretPid);
                turret.SetCustomTarget(new Vector3(0f, 0.5f, 4f));
            }
            for (int i = 0; i < 250; i++)
            {
                foreach (GenericTurret turret in turrets) turret.Align();
                yield return new WaitForFixedUpdate();
            }
            foreach (GenericTurret turret in turrets)
                Assert.Less(Mathf.Abs(turret.GetError()), 3f, turret.name + " never settled on a stationary target");
            for (int i = 0; i < 200; i++)
            {
                robot.GetComponent<Rigidbody>().MoveRotation(Quaternion.Euler(0f, i * 0.3f, 0f));
                foreach (GenericTurret turret in turrets) turret.Align();
                yield return new WaitForFixedUpdate();
            }
            for (int i = 0; i < 150; i++)
            {
                foreach (GenericTurret turret in turrets) turret.Align();
                yield return new WaitForFixedUpdate();
            }
            foreach (GenericTurret turret in turrets)
                Assert.Less(Mathf.Abs(turret.GetError()), 3f, turret.name + " lost its target when the chassis turned");
        }

        [UnityTest]
        public IEnumerator IntakeReachesFiftyDegreesWithRuntimeRollerCores()
        {
            yield return CreateRobot(false);
            GenericJoint intake = robot.transform.Find("Intake/Back Link").GetComponent<GenericJoint>();
            intake.SetPid(intakePid);
            intake.angularAxis = JointAxis.X;
            foreach (Rigidbody body in robot.transform.Find("Intake").GetComponentsInChildren<Rigidbody>())
                body.useGravity = true;
            // Exercise the production profile without starting another player/robot scheduler.
            GameObject profileObject = new GameObject("Inactive production profile");
            profileObject.transform.SetParent(fixture.transform);
            profileObject.SetActive(false);
            BaywatchRobotics profile = profileObject.AddComponent<BaywatchRobotics>();
            var profileSettings = new SerializedObject(profile);
            profileSettings.FindProperty("intakeMaxVelocity").floatValue = intakeVelocity;
            profileSettings.FindProperty("intakeMaxAcceleration").floatValue = intakeAcceleration;
            profileSettings.ApplyModifiedPropertiesWithoutUndo();
            const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            var step = typeof(BaywatchRobotics).GetMethod("StepIntakeProfile", flags);
            var angle = typeof(BaywatchRobotics).GetField("intakeCommandAngle", flags);
            foreach (float target in new[] { 50f, 0f })
            {
                for (int i = 0; i < 350; i++)
                {
                    step.Invoke(profile, new object[] { target, Time.fixedDeltaTime });
                    intake.SetTargetAngle((float)angle.GetValue(profile)).withAxis(JointAxis.X).withFixedDTiming();
                    yield return new WaitForFixedUpdate();
                }
                if (Mathf.Abs(Mathf.DeltaAngle(intake.GetAngle(), target)) >= 3f) LogIntakeContacts();
                Assert.Less(Mathf.Abs(Mathf.DeltaAngle(intake.GetAngle(), target)), 3f,
                    "Intake jittered or jammed with runtime cores present, target " + target);
            }
        }

        private void LogIntakeContacts()
        {
            var contacts = new List<(float depth, string pair)>();
            Joint[] joints = robot.GetComponentsInChildren<Joint>();
            foreach (Collider a in robot.transform.Find("Intake").GetComponentsInChildren<Collider>())
                foreach (Collider b in robot.GetComponentsInChildren<Collider>())
                {
                    if (a == b || a.attachedRigidbody == b.attachedRigidbody || a.isTrigger || b.isTrigger || Physics.GetIgnoreCollision(a, b)) continue;
                    if (joints.Any(j => !j.enableCollision && ((j.GetComponent<Rigidbody>() == a.attachedRigidbody && j.connectedBody == b.attachedRigidbody)
                        || (j.GetComponent<Rigidbody>() == b.attachedRigidbody && j.connectedBody == a.attachedRigidbody)))) continue;
                    if (Physics.ComputePenetration(a, a.transform.position, a.transform.rotation, b, b.transform.position, b.transform.rotation, out Vector3 direction, out float depth))
                        contacts.Add((depth, AnimationUtility.CalculateTransformPath(a.transform, robot.transform) + " <> " + AnimationUtility.CalculateTransformPath(b.transform, robot.transform)));
                }
            foreach (var contact in contacts.OrderByDescending(p => p.depth).Take(10)) Debug.Log($"MIRA runtime overlap {contact.depth:F5}: {contact.pair}");
        }

        [UnityTest]
        public IEnumerator IntakeRollersPullFuelInFromTheFloor()
        {
            yield return CreateRobot(true);
            GenericJoint intake = robot.transform.Find("Intake/Back Link").GetComponent<GenericJoint>();
            intake.SetPid(intakePid);
            intake.angularAxis = JointAxis.X;
            foreach (Rigidbody body in robot.transform.Find("Intake").GetComponentsInChildren<Rigidbody>())
                body.useGravity = true;
            // Deploy to the owner-calibrated travel, not a synthetic test pose.
            var controller = new SerializedObject(AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath)
                .GetComponent<BaywatchRobotics>());
            float deployedAngle = controller.FindProperty("intakeDeployedAngle").floatValue;
            for (int i = 0; i < 500; i++)
            {
                CommandCollection(intake, deployedAngle, false);
                yield return new WaitForFixedUpdate();
            }
            Assert.Less(Mathf.Abs(Mathf.DeltaAngle(intake.GetAngle(), deployedAngle)), 5f,
                "Intake did not reach the calibrated deployed pose");

            Transform intakeRoot = robot.transform.Find("Intake");
            RollerGroup group = intakeRoot.GetComponentInChildren<RollerGroup>(true);
            Assert.IsNotNull(group, "Intake has no functional roller group");
            InteractionRoller front = intakeRoot.GetComponentsInChildren<InteractionRoller>(true)
                .OrderByDescending(r => r.transform.position.z).First();
            float radius = new SerializedObject(front).FindProperty("radius").floatValue * 0.0254f;

            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "MIRA test floor";
            floor.transform.SetParent(fixture.transform);
            floor.transform.position = new Vector3(0f, -0.5f, 0f);
            floor.transform.localScale = new Vector3(20f, 1f, 20f);

            // The ball rests on the floor exactly where a real fuel ball would, under the roller.
            Vector3 roller = front.transform.position;
            Vector3 spawn = new Vector3(roller.x, 0.075f, roller.z + 0.03f);
            Debug.Log($"MIRA intake pickup probe: roller={front.name} pos={roller} up={front.transform.up} " +
                      $"radius={radius:F4} rollerBottom={roller.y - radius:F3} rpm={intakeRpm} spawn={spawn}");

            GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"),
                spawn, Quaternion.identity, fixture.transform);
            foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true))
                Object.DestroyImmediate(component);
            Rigidbody ball = fuel.GetComponent<Rigidbody>();
            for (int i = 0; i < 60; i++)
            {
                CommandCollection(intake, deployedAngle, false);
                yield return new WaitForFixedUpdate();
            }
            Vector3 start = ball.position;
            Assert.Less(Vector3.Distance(start, spawn), 0.03f,
                "Fuel moved before collection started; pickup fixture is not a stationary floor ball");
            float maxLift = 0f;
            for (int i = 0; i < 200; i++)
            {
                CommandCollection(intake, deployedAngle);
                yield return new WaitForFixedUpdate();
                maxLift = Mathf.Max(maxLift, ball.position.y - start.y);
                if (i % 50 == 0)
                    Debug.Log($"MIRA intake pickup {i}: pos={ball.position} velocity={ball.velocity} " +
                              $"inward={start.z - ball.position.z:F4} rpm={front.realRpm} angle={(float)intake.GetAngle():F2}");
            }
            group.SetRpm(0f);
            Vector3 end = ball.position;
            float inward = start.z - end.z;
            Debug.Log($"MIRA intake pickup result: start={start} end={end} inward={inward:F4} maxLift={maxLift:F3}");
            Assert.Greater(inward, 0.05f, "Intake rollers spun but did not pull the floor fuel inward");
            Assert.Greater(maxLift, 0.02f, "Intake never lifted the fuel off the floor");
            Assert.Less(end.z, 0.30f, "Fuel stalled at the front lip instead of entering the hopper");
        }

        private void CommandCollection(GenericJoint intake, float target, bool collecting = true)
        {
            const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            typeof(BaywatchRobotics).GetMethod("RunMechanisms", flags)
                .Invoke(productionController, new object[] { collecting, false });
            typeof(BaywatchRobotics).GetMethod("StepIntakeProfile", flags)
                .Invoke(productionController, new object[] { target, Time.fixedDeltaTime });
            float angle = (float)typeof(BaywatchRobotics).GetField("intakeCommandAngle", flags).GetValue(productionController);
            intake.SetTargetAngle(angle).withAxis(JointAxis.X).withFixedDTiming();
            var drive = intake.GetComponent<ConfigurableJoint>();
            JointDrive settings = drive.angularXDrive;
            settings.maximumForce = new SerializedObject(productionController).FindProperty("intakeMaxTorque").floatValue;
            drive.angularXDrive = settings;
        }

        [UnityTest]
        public IEnumerator MissingGroupMemberDoesNotAbortCommandsOrReportReady()
        {
            yield return CreateRobot(false);
            RollerGroup group = robot.transform.Find("Kickers/Right").GetComponent<RollerGroup>();
            var serialized = new SerializedObject(group);
            SerializedProperty entries = serialized.FindProperty("rollers");
            entries.arraySize++;
            SerializedProperty missing = entries.GetArrayElementAtIndex(entries.arraySize - 1);
            foreach (string field in new[] { "IntRoller", "GenRoller", "PvffController", "Flywheel" })
                missing.FindPropertyRelative(field).objectReferenceValue = null;
            serialized.ApplyModifiedPropertiesWithoutUndo();
            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("RollerGroup 'Right' has a missing roller reference"));
            Assert.DoesNotThrow(() => group.SetRpm(321f));
            foreach (InteractionRoller roller in group.GetComponentsInChildren<InteractionRoller>())
                Assert.AreEqual(321f, Mathf.Abs(roller.TargetRpm), "A missing entry prevented a valid member receiving commands");
            Assert.IsFalse(group.AtSpeed().Value, "A broken group must not permit shooting readiness");
            Assert.DoesNotThrow(() => group.SetRpm(0f));
            Assert.DoesNotThrow(() => group.Stop());
            yield return null;
        }

        [UnityTest]
        public IEnumerator IndexersFollowRequestedTopDownRotation()
        {
            yield return CreateRobot(true);
            var settings = new SerializedObject(productionController);
            foreach (string name in new[] { "Left", "Right" })
            {
                string field = name.ToLowerInvariant();
                var side = settings.FindProperty(field);
                var indexer = (InteractionRoller)side.FindPropertyRelative("indexer").objectReferenceValue;
                Vector3 start = indexer.transform.position + new Vector3(name == "Left" ? 0.085f : -0.085f, 0.075f, 0f);
                float rpm = side.FindPropertyRelative("indexerRpm").floatValue;
                // In Unity's X/Z top view, positive world-Y rotation moves forward
                // toward right: clockwise. Both authored physics axles point down.
                float worldYSpin = Vector3.Dot(indexer.transform.right * rpm, Vector3.up);
                Assert.Greater(worldYSpin * (name == "Right" ? 1f : -1f), 0f,
                    name + " indexer violates right-clockwise / left-counterclockwise top view");
                GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"),
                    start, Quaternion.identity, fixture.transform);
                foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                Rigidbody ball = fuel.GetComponent<Rigidbody>();
                indexer.SetRpm(rpm);
                for (int tick = 0; tick < 20; tick++) yield return new WaitForFixedUpdate();
                indexer.SetRpm(0f);
                Debug.Log($"MIRA indexer {name} start={start} end={ball.position}");
                Assert.Greater(ball.position.z, start.z + 0.005f, name + " indexer contact motion disagrees with its rotation");
                Object.DestroyImmediate(fuel);
            }
        }

        [UnityTest]
        public IEnumerator AimErrorDoesNotBlockFeedReadiness()
        {
            yield return CreateRobot(true);
            var settings = new SerializedObject(productionController);
            foreach (string name in new[] { "left", "right" })
            {
                var side = settings.FindProperty(name);
                var turret = (GenericTurret)side.FindPropertyRelative("turret").objectReferenceValue;
                turret.SetCustomTarget(turret.transform.position + (turret.transform.forward + turret.transform.right) * 4f);
                side.FindPropertyRelative("targetValid").boolValue = true;
                side.FindPropertyRelative("requestedRpm").floatValue = 3000f;
            }
            settings.ApplyModifiedPropertiesWithoutUndo();
            const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            for (int tick = 0; tick < 300; tick++)
            {
                typeof(BaywatchRobotics).GetMethod("RunMechanisms", flags).Invoke(productionController, new object[] { false, true });
                typeof(BaywatchRobotics).GetMethod("Feed", flags).Invoke(productionController, null);
                yield return new WaitForFixedUpdate();
            }
            settings.Update();
            foreach (string name in new[] { "left", "right" })
            {
                var side = settings.FindProperty(name);
                Assert.Greater(side.FindPropertyRelative("aimError").floatValue, 10f, "Test accidentally aligned the turret");
                Assert.IsTrue(side.FindPropertyRelative("ready").boolValue, "Aim error still blocks feed readiness");
                var kickers = (RollerGroup)side.FindPropertyRelative("kickers").objectReferenceValue;
                foreach (InteractionRoller roller in kickers.GetComponentsInChildren<InteractionRoller>())
                    Assert.Greater(Mathf.Abs(roller.TargetRpm), 0f, "Misaligned shooter never commanded its kickers");
            }
        }

        [UnityTest]
        public IEnumerator KickersAcquireFuelFromBelowThroughProductionGate()
        {
            yield return CreateRobot(true);
            var settings = new SerializedObject(productionController);
            foreach (string name in new[] { "left", "right" })
            {
                SerializedProperty side = settings.FindProperty(name);
                var turret = (GenericTurret)side.FindPropertyRelative("turret").objectReferenceValue;
                turret.SetPid(turretPid);
                turret.SetCustomTarget(turret.transform.position + turret.transform.forward * 4f);
                side.FindPropertyRelative("targetValid").boolValue = true;
                side.FindPropertyRelative("requestedRpm").floatValue = 3000f;
            }
            settings.ApplyModifiedPropertiesWithoutUndo();
            const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            var run = typeof(BaywatchRobotics).GetMethod("RunMechanisms", flags);
            var feed = typeof(BaywatchRobotics).GetMethod("Feed", flags);
            void Tick(bool shooting = true)
            {
                foreach (GenericTurret turret in robot.GetComponentsInChildren<GenericTurret>()) turret.Align();
                run.Invoke(productionController, new object[] { false, shooting });
                feed.Invoke(productionController, null);
            }
            for (int tick = 0; tick < 300; tick++) { Tick(); yield return new WaitForFixedUpdate(); }
            settings.Update();
            foreach (string name in new[] { "left", "right" })
            {
                SerializedProperty side = settings.FindProperty(name);
                Debug.Log($"MIRA gate {name} ready={side.FindPropertyRelative("ready").boolValue} error={side.FindPropertyRelative("aimError").floatValue}");
                Assert.IsTrue(side.FindPropertyRelative("ready").boolValue, "Production gate never enabled " + name + " kicker");
            }
            foreach (string laneName in new[] { "Left", "Right" })
            {
                InteractionRoller[] rollers = robot.transform.Find("Kickers/" + laneName).GetComponentsInChildren<InteractionRoller>();
                InteractionRoller[] front = robot.transform.Find("Kickers/" + laneName + "/Front").GetComponentsInChildren<InteractionRoller>();
                Vector3 centre = front.Aggregate(Vector3.zero, (sum, roller) => sum + roller.transform.position) / front.Length;
                // Fuel reaches these wheels from the hopper along the ramp, not from below.
                centre += Vector3.forward * 0.10f;
                centre.y = 0.236f;
                GameObject unpoweredFuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"), centre, Quaternion.identity, fixture.transform);
                foreach (MonoBehaviour component in unpoweredFuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                Rigidbody unpoweredBall = unpoweredFuel.GetComponent<Rigidbody>();
                unpoweredBall.velocity = Vector3.back * 1.2f;
                float unpoweredPeak = centre.y;
                for (int tick = 0; tick < 200; tick++)
                {
                    Tick(false);
                    yield return new WaitForFixedUpdate();
                    unpoweredPeak = Mathf.Max(unpoweredPeak, unpoweredBall.position.y);
                }
                Debug.Log($"MIRA unpowered acquisition {laneName} peak={unpoweredPeak:F3} end={unpoweredBall.position}");
                Object.DestroyImmediate(unpoweredFuel);
                for (int tick = 0; tick < 300; tick++) { Tick(); yield return new WaitForFixedUpdate(); }
                GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"), centre, Quaternion.identity, fixture.transform);
                foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                Rigidbody ball = fuel.GetComponent<Rigidbody>();
                ball.velocity = Vector3.back * 1.2f;
                float peak = centre.y;
                for (int tick = 0; tick < 200; tick++)
                {
                    Tick();
                    yield return new WaitForFixedUpdate();
                    peak = Mathf.Max(peak, ball.position.y);
                }
                Debug.Log($"MIRA ramp pickup {laneName} start={centre} end={ball.position} peak={peak:F3} rpms={string.Join(",", rollers.Select(r => r.realRpm))}");
                Object.DestroyImmediate(fuel);
                Assert.Greater(peak, unpoweredPeak + 0.05f, "Core collision rather than powered kickers caused the lift");
                Assert.Greater(peak, robot.transform.Find("Turrets/" + laneName).position.y + 0.03f,
                    laneName + " production gate did not carry fuel into the turret inlet");
            }
        }

        [UnityTest]
        public IEnumerator KickersLiftFuelEnteringFromFrontAlongRamp()
        {
            yield return CreateRobot(true);
            var settings = new SerializedObject(productionController);
            foreach (string name in new[] { "Left", "Right" })
            {
                Transform lane = robot.transform.Find("Kickers/" + name);
                RollerGroup group = lane.GetComponent<RollerGroup>();
                InteractionRoller[] front = lane.Find("Front").GetComponentsInChildren<InteractionRoller>();
                Vector3 centre = front.Aggregate(Vector3.zero, (sum, roller) => sum + roller.transform.position) / front.Length;
                float commanded = settings.FindProperty(name.ToLowerInvariant() + ".kickerRpm").floatValue;
                // A fuel ball rolling into the lane mouth along the ramp, as it arrives from
                // the hopper rather than dropped between the two wheel rows.
                Vector3 start = centre + Vector3.forward * 0.10f;
                start.y = 0.236f;
                GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"), start, Quaternion.identity, fixture.transform);
                foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                Rigidbody ball = fuel.GetComponent<Rigidbody>();
                ball.velocity = Vector3.back * 1.2f;
                group.SetRpm(commanded);
                float peak = start.y;
                for (int tick = 0; tick < 200; tick++)
                {
                    yield return new WaitForFixedUpdate();
                    peak = Mathf.Max(peak, ball.position.y);
                }
                group.SetRpm(0f);
                Debug.Log($"MIRA ramp entry {name} start={start} end={ball.position} peak={peak:F3}");
                Object.DestroyImmediate(fuel);
                Assert.Greater(peak, start.y + 0.10f,
                    name + " front wheels did not carry fuel up the ramp");
            }
        }

        [UnityTest]
        public IEnumerator KickersLiftFuelThroughTurretInlets()
        {
            yield return CreateRobot(true);
            foreach (string laneName in new[] { "Left", "Right" })
            {
                Transform lane = robot.transform.Find("Kickers/" + laneName);
                RollerGroup group = lane.GetComponent<RollerGroup>();
                InteractionRoller[] front = lane.Find("Front").GetComponentsInChildren<InteractionRoller>();
                Vector3 centre = front.Aggregate(Vector3.zero, (sum, roller) => sum + roller.transform.position) / front.Length;
                centre += Vector3.forward * 0.10f;
                centre.y = 0.236f;
                float inletHeight = robot.transform.Find("Turrets/" + laneName).position.y;
                float rpm = new SerializedObject(productionController).FindProperty(laneName.ToLowerInvariant() + ".kickerRpm").floatValue;
                foreach (float sign in new[] { 1f, -1f })
                {
                    GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"),
                        centre, Quaternion.identity, fixture.transform);
                    foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                    Rigidbody ball = fuel.GetComponent<Rigidbody>();
                    ball.velocity = Vector3.back * 1.2f;
                    float peak = ball.position.y;
                    group.SetRpm(sign * rpm);
                    for (int tick = 0; tick < 150; tick++)
                    {
                        yield return new WaitForFixedUpdate();
                        peak = Mathf.Max(peak, ball.position.y);
                    }
                    group.SetRpm(0f);
                    Debug.Log($"MIRA kicker {laneName} sign={sign} start={centre} end={ball.position} peak={peak:F3}");
                    if (sign > 0f)
                        Assert.Greater(peak, inletHeight + 0.03f, laneName + " kicker failed to lift fuel through the turret inlet");
                    else
                    {
                        Assert.Less(ball.position.y, inletHeight, "Reversed kicker control delivered fuel to the turret inlet");
                        Assert.Greater(ball.position.z, centre.z + 0.05f,
                            "Reversed kickers did not reject fuel back out of the lane mouth");
                    }
                    Object.DestroyImmediate(fuel);
                }
            }
        }

        [UnityTest]
        public IEnumerator FeedAndIntakeAnimationStaysOnAxlesAndLeavesCollidersFixed()
        {
            yield return CreateRobot(true);
            InteractionRoller[] rollers = new[] { "Intake", "Indexer", "Kickers" }
                .SelectMany(name => robot.transform.Find(name).GetComponentsInChildren<InteractionRoller>()).ToArray();
            Collider[] colliders = robot.GetComponentsInChildren<Collider>();
            var owners = colliders.Select(c => (c.attachedRigidbody ?? c.GetComponentInParent<Rigidbody>()).transform).ToArray();
            var positions = colliders.Select((c, i) => owners[i].InverseTransformPoint(c.transform.position)).ToArray();
            var rotations = colliders.Select((c, i) => Quaternion.Inverse(owners[i].rotation) * c.transform.rotation).ToArray();
            foreach (InteractionRoller roller in rollers) roller.SetRpm(1234f);
            for (int tick = 0; tick < 120; tick++)
            {
                yield return new WaitForFixedUpdate();
                for (int i = 0; i < colliders.Length; i++)
                {
                    Assert.Less(Vector3.Distance(owners[i].InverseTransformPoint(colliders[i].transform.position), positions[i]), 0.001f,
                        "Animation moved collider " + colliders[i].name);
                    Assert.Less(Quaternion.Angle(Quaternion.Inverse(owners[i].rotation) * colliders[i].transform.rotation, rotations[i]), 0.1f,
                        "Animation rotated collider " + colliders[i].name);
                }
                foreach (InteractionRoller roller in rollers)
                {
                    SerializedProperty models = new SerializedObject(roller).FindProperty("models");
                    for (int i = 0; i < models.arraySize; i++)
                    {
                        var model = (GameObject)models.GetArrayElementAtIndex(i).FindPropertyRelative("model").objectReferenceValue;
                        if (model.name != "MIRA_VisualSpin_Drive")
                            Assert.Less(Vector3.Distance(model.transform.position, roller.transform.position), 0.001f, "Visual pivot left axle");
                        Assert.AreEqual(0, model.GetComponentsInChildren<Collider>().Length, "Animation owns colliders");
                        var bounds = new Bounds();
                        bool first = true;
                        foreach (Renderer renderer in model.GetComponentsInChildren<Renderer>())
                        {
                            if (first) { bounds = renderer.bounds; first = false; }
                            else bounds.Encapsulate(renderer.bounds);
                        }
                        float radialOffset = Vector3.ProjectOnPlane(bounds.center - model.transform.position, roller.transform.right).magnitude;
                        Assert.Less(radialOffset, 0.04f, "Visual geometry orbits axle: " + AnimationUtility.CalculateTransformPath(roller.transform, robot.transform));
                        Assert.Less(bounds.size.magnitude, 1f, "Visual mesh has an oversized CAD import scale");
                    }
                }
            }
        }

        [UnityTest]
        public IEnumerator FrontBumperProbe()
        {
            yield return CreateRobot(true);
            Transform model = robot.transform.Find("Stationary/Drivetrain/Model");
            foreach (Collider collider in model.GetComponentsInChildren<Collider>(true))
                Debug.Log($"MIRA chassis collider {AnimationUtility.CalculateTransformPath(collider.transform, robot.transform)} enabled={collider.enabled} center={collider.bounds.center:F3} size={collider.bounds.size:F3}");
            foreach (MeshRenderer mesh in model.GetComponentsInChildren<MeshRenderer>(true))
                if (mesh.bounds.max.z > 0.30f)
                    Debug.Log($"MIRA chassis mesh {AnimationUtility.CalculateTransformPath(mesh.transform, robot.transform)} center={mesh.bounds.center:F3} size={mesh.bounds.size:F3}");
        }

        [UnityTest]
        public IEnumerator ShootersCarryFuelPastTheFixedHood()
        {
            yield return CreateRobot(true);
            var settings = new SerializedObject(productionController);
            foreach (string name in new[] { "left", "right" })
            {
                SerializedProperty side = settings.FindProperty(name);
                var turret = (GenericTurret)side.FindPropertyRelative("turret").objectReferenceValue;
                turret.SetPid(turretPid);
                turret.SetCustomTarget(turret.transform.position + turret.transform.forward * 4f);
                side.FindPropertyRelative("targetValid").boolValue = true;
                side.FindPropertyRelative("requestedRpm").floatValue = 4000f;
            }
            settings.ApplyModifiedPropertiesWithoutUndo();
            const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            var run = typeof(BaywatchRobotics).GetMethod("RunMechanisms", flags);
            var feed = typeof(BaywatchRobotics).GetMethod("Feed", flags);
            for (int tick = 0; tick < 300; tick++)
            {
                foreach (GenericTurret turret in robot.GetComponentsInChildren<GenericTurret>()) turret.Align();
                run.Invoke(productionController, new object[] { false, true });
                feed.Invoke(productionController, null);
                yield return new WaitForFixedUpdate();
            }
            settings.Update();
            foreach (string laneName in new[] { "Left", "Right" })
            {
                Assert.IsTrue(settings.FindProperty(laneName.ToLowerInvariant()).FindPropertyRelative("ready").boolValue,
                    laneName + " shooter never reached feed readiness");
                Transform lane = robot.transform.Find("Kickers/" + laneName);
                InteractionRoller[] front = lane.Find("Front").GetComponentsInChildren<InteractionRoller>();
                Vector3 start = front.Aggregate(Vector3.zero, (sum, roller) => sum + roller.transform.position) / front.Length
                    + Vector3.forward * 0.10f;
                start.y = 0.236f;
                GameObject fuel = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab"), start, Quaternion.identity, fixture.transform);
                foreach (MonoBehaviour component in fuel.GetComponentsInChildren<MonoBehaviour>(true)) Object.DestroyImmediate(component);
                Rigidbody ball = fuel.GetComponent<Rigidbody>();
                ball.velocity = Vector3.back * 1.2f;
                float peak = start.y;
                for (int tick = 0; tick < 240; tick++)
                {
                    foreach (GenericTurret turret in robot.GetComponentsInChildren<GenericTurret>()) turret.Align();
                    run.Invoke(productionController, new object[] { false, true });
                    feed.Invoke(productionController, null);
                    yield return new WaitForFixedUpdate();
                    peak = Mathf.Max(peak, ball.position.y);
                }
                float travel = Vector3.ProjectOnPlane(ball.position - start, Vector3.up).magnitude;
                Debug.Log($"MIRA shot {laneName} peak={peak:F3} end={ball.position:F3} travel={travel:F3}");
                Object.DestroyImmediate(fuel);
                Assert.Greater(peak, robot.transform.Find("Turrets/" + laneName).position.y + 0.6f,
                    laneName + " fuel never cleared the fixed hood");
                Assert.Greater(travel, 1f, laneName + " fuel did not launch away from the shooter");
            }
        }

        [UnityTest]
        public IEnumerator ShooterAnimationDoesNotMovePhysicsOrOrbitTheRobot()
        {
            yield return CreateRobot(true);
            InteractionRoller[] wheels = robot.GetComponentsInChildren<InteractionRoller>()
                .Where(r => r.GetComponent<PvffController>() != null).ToArray();
            Rigidbody[] owners = wheels.Select(w => w.GetComponentInParent<Rigidbody>()).ToArray();
            var positions = wheels.Select((w, i) => owners[i].transform.InverseTransformPoint(w.transform.position)).ToArray();
            foreach (GenericTurret turret in robot.GetComponentsInChildren<GenericTurret>())
            {
                turret.SetPid(turretPid);
                turret.SetCustomTarget(new Vector3(0f, 0.5f, 4f));
            }
            foreach (InteractionRoller wheel in wheels) wheel.GetComponent<PvffController>().SetRpm(3000f);
            for (int i = 0; i < 100; i++)
            {
                yield return new WaitForFixedUpdate();
                for (int j = 0; j < wheels.Length; j++)
                {
                    Assert.Less(Vector3.Distance(owners[j].transform.InverseTransformPoint(wheels[j].transform.position), positions[j]), 0.005f,
                        "Animating a shooter wheel moved its physics object");
                    SerializedProperty models = new SerializedObject(wheels[j]).FindProperty("models");
                    Assert.Greater(models.arraySize, 0);
                    for (int k = 0; k < models.arraySize; k++)
                    {
                        var model = (GameObject)models.GetArrayElementAtIndex(k).FindPropertyRelative("model").objectReferenceValue;
                        Assert.IsNotNull(model, "Shooter visual reference is null");
                        Assert.IsFalse(wheels[j].transform.IsChildOf(model.transform), "Animation owns a physics ancestor");
                        foreach (Renderer renderer in model.GetComponentsInChildren<Renderer>())
                            Assert.Less(Vector3.Distance(renderer.bounds.center, wheels[j].transform.position), 0.04f,
                                "Shooter mesh is orbiting its CAD origin instead of spinning at the axle");
                    }
                }
            }
        }
    }
}
#endif
