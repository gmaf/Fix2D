using System;
using Unity.Assertions;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public static class Solver
    {
        // Settings for controlling the solver stabilization heuristic.
        public struct StabilizationHeuristicSettings
        {
            private byte m_EnableSolverStabilization;

            // Global switch to enable/disable the whole heuristic (false by default)
            public bool EnableSolverStabilization
            {
                get => m_EnableSolverStabilization > 0;
                set => m_EnableSolverStabilization = (byte)(value ? 1 : 0);
            }

            // Individual features control (only valid when EnableSolverStabilizationHeuristic is true)

            private byte m_EnableFrictionVelocities;

            // Switch to enable/disable heuristic when calculating friction velocities.
            // Should be disabled only if it is causing behavior issues.
            public bool EnableFrictionVelocities
            {
                get => m_EnableFrictionVelocities > 0;
                set => m_EnableFrictionVelocities = (byte)(value ? 1 : 0);
            }

            // Controls the intensity of the velocity clipping.
            // Defaults to 1.0f, while other values will scale the intensity up/down.
            // Shouldn't go higher than 5.0f, as it will result in bad behavior (too aggressive velocity clipping).
            // Set it to 0.0f to disable the feature.
            public float VelocityClippingFactor;

            // Controls the intensity of inertia scaling.
            // Defaults to 1.0f, while other values will scale the intensity up/down.
            // Shouldn't go higher than 5.0f, as it will result in bad behavior (too high inertia of bodies).
            // Set it to 0.0f to disable the feature.
            public float InertiaScalingFactor;

            public static readonly StabilizationHeuristicSettings Default = new StabilizationHeuristicSettings
            {
                m_EnableSolverStabilization = 1,
                m_EnableFrictionVelocities = 1,
                VelocityClippingFactor = 1.0f,
                InertiaScalingFactor = 1.0f
            };
        }

        // Data used for solver stabilization
        public struct StabilizationData
        {
            public StabilizationData(StabilizationHeuristicSettings heuristicSettings, float2 gravity, SimulationContext context)
            {
                StabilizationHeuristicSettings = heuristicSettings;
                Gravity = gravity;
                if (heuristicSettings.EnableSolverStabilization)
                {
                    InputVelocities = context.InputVelocities;
                    MotionData = context.SolverStabilizationMotionData;
                }
                else
                {
                    InputVelocities = default;
                    MotionData = default;
                }
            }

            // Settings for stabilization heuristics
            internal StabilizationHeuristicSettings StabilizationHeuristicSettings;

            // Disable container safety restriction because it will complain about aliasing
            // with SimulationContext buffers, and it is aliasing, but completely safe.
            // Also, we need the ability to have these not allocated when the feature is not used.

            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<PhysicsVelocity> InputVelocities;

            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<StabilizationMotionData> MotionData;

            // Gravity is used to define thresholds for stabilization,
            // and it's not needed in the solver unless stabilization is required.
            internal float2 Gravity;
        }

        // Per motion data for solver stabilization
        internal struct StabilizationMotionData
        {
            public float InverseInertiaScale;
            public byte NumPairs;
        }

        // Internal motion data input for the solver stabilization
        internal struct MotionStabilizationInput
        {
            public PhysicsVelocity InputVelocity;
            public float InverseInertiaScale;

            public static readonly MotionStabilizationInput Default = new MotionStabilizationInput
            {
                InputVelocity = PhysicsVelocity.Zero,
                InverseInertiaScale = 1.0f
            };
        }

        internal struct StepInput
        {
            public bool IsFirstIteration;
            public bool IsLastIteration;
            public float InvNumSolverIterations;
            public float Timestep;
            public float InvTimestep;
        }

        // Schedule the job to apply gravity to all dynamic bodies and copy input velocities
        internal static JobHandle ScheduleApplyGravityAndCopyInputVelocitiesJob(
            ref DynamicsWorld world,
            NativeArray<PhysicsVelocity> inputVelocities,
            float2 gravityAcceleration,
            JobHandle inputDeps, int threadCountHint = 0)
        {
            if (threadCountHint <= 0)
            {
                var job = new ApplyGravityAndCopyInputVelocitiesJob
                {
                    BodyMotionVelocity = world.BodyMotionVelocity,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };

                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelApplyGravityAndCopyInputVelocitiesJob
                {
                    BodyMotionVelocity = world.BodyMotionVelocity,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };

                return job.Schedule(world.BodyMotionCount, 64, inputDeps);
            }
        }

        // Apply gravity to all dynamic bodies and copy input velocities
        internal static void ApplyGravityAndCopyInputVelocities(
            NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
            NativeArray<PhysicsVelocity> inputVelocities,
            float2 gravity)
        {
            for (var i = 0; i < bodyMotionVelocity.Length; i++)
            {
                ParallelApplyGravityAndCopyInputVelocitiesJob.Execute(i, gravity, bodyMotionVelocity, inputVelocities);
            }
        }

        // Schedule jobs to build Jacobians from the contacts stored in the simulation context
        internal static SimulationJobHandles ScheduleBuildJacobiansJobs(ref PhysicsWorld world, float timeStep, float2 gravity,
            int numSolverIterations, JobHandle inputDeps, ref NativeList<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            ref NativeStream contacts, ref NativeStream jacobians, int threadCountHint = 0)
        {
            SimulationJobHandles returnHandles = default;

            if (threadCountHint <= 0)
            {
                returnHandles.FinalExecutionHandle = new BuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    Gravity = gravity,
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray()
                }.Schedule(inputDeps);
            }
            else
            {
                var buildJob = new ParallelBuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    InvTimeStep = timeStep > 0.0f ? 1.0f / timeStep : 0.0f,
                    GravityAcceleration = math.length(gravity),
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray(),
                    SolverSchedulerInfo = solverSchedulerInfo
                };

                JobHandle handle = buildJob.ScheduleUnsafeIndex0(solverSchedulerInfo.NumWorkItems, 1, inputDeps);

                returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                    dispatchPairs.Dispose(handle),
                    contacts.Dispose(handle));
                returnHandles.FinalExecutionHandle = handle;
            }

            return returnHandles;
        }

        // Build Jacobians from the contacts and joints stored in the simulation context
        public static void BuildJacobians(ref PhysicsWorld world,
            float timeStep, float2 gravity, int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref NativeStream.Reader contactsReader, ref NativeStream.Writer jacobiansWriter)
        {
            contactsReader.BeginForEachIndex(0);
            jacobiansWriter.BeginForEachIndex(0);
            float invTimeStep = timeStep > 0.0f ? 1.0f / timeStep : 0.0f;
            float gravityAcceleration = math.length(gravity);
            BuildJacobians(ref world, timeStep, invTimeStep, gravityAcceleration, numSolverIterations,
                dispatchPairs, 0, dispatchPairs.Length, ref contactsReader, ref jacobiansWriter);
        }

        // Solve the Jacobians stored in the simulation context
        public static void SolveJacobians(ref NativeStream.Reader jacobiansReader, NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            float timeStep, int numIterations, ref NativeStream.Writer collisionEventsWriter, ref NativeStream.Writer triggerEventsWriter,
            StabilizationData solverStabilizationData)
        {
            float invNumIterations = math.rcp(numIterations);
            float invTimeStep = timeStep > 0.0f ? 1.0f / timeStep : 0.0f;
            for (int solverIterationId = 0; solverIterationId < numIterations; solverIterationId++)
            {
                var stepInput = new StepInput
                {
                    InvNumSolverIterations = invNumIterations,
                    IsFirstIteration = solverIterationId == 0,
                    IsLastIteration = solverIterationId == numIterations - 1,
                    Timestep = timeStep,
                    InvTimestep = invTimeStep
                };

                Solve(motionVelocities, ref jacobiansReader, ref collisionEventsWriter, ref triggerEventsWriter, 0, stepInput, solverStabilizationData);

                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    StabilizeVelocities(motionVelocities, stepInput.IsFirstIteration, timeStep, solverStabilizationData);
                }
            }
        }

        // Schedule jobs to solve the Jacobians stored in the simulation context
        internal static unsafe SimulationJobHandles ScheduleSolveJacobiansJobs(
            ref DynamicsWorld dynamicsWorld, float timestep, int numIterations,
            ref NativeStream jacobians, ref NativeStream collisionEvents, ref NativeStream triggerEvents,
            ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            StabilizationData solverStabilizationData, JobHandle inputDeps, int threadCountHint = 0)
        {
            SimulationJobHandles returnHandles = default;

            if (threadCountHint <= 0)
            {
                collisionEvents = new NativeStream(1, Allocator.Persistent);
                triggerEvents = new NativeStream(1, Allocator.Persistent);
                returnHandles.FinalExecutionHandle = new SolverJob
                {
                    CollisionEventsWriter = collisionEvents.AsWriter(),
                    JacobiansReader = jacobians.AsReader(),
                    NumIterations = numIterations,
                    Timestep = timestep,
                    TriggerEventsWriter = triggerEvents.AsWriter(),
                    MotionVelocities = dynamicsWorld.BodyMotionVelocity,
                    SolverStabilizationData = solverStabilizationData,
                }.Schedule(inputDeps);

                return returnHandles;
            }

            JobHandle handle;

            int numPhases = solverSchedulerInfo.NumPhases;

            // Use persistent allocator to allow these to live until the start of next step
            {
                NativeArray<int> workItemList = solverSchedulerInfo.NumWorkItems;

                //TODO: Change this to Allocator.TempJob when https://github.com/Unity-Technologies/Unity.Physics/issues/7 is resolved
                JobHandle collisionEventStreamHandle = NativeStream.ScheduleConstruct(out collisionEvents, workItemList, inputDeps, Allocator.Persistent);
                JobHandle triggerEventStreamHandle = NativeStream.ScheduleConstruct(out triggerEvents, workItemList, inputDeps, Allocator.Persistent);

                handle = JobHandle.CombineDependencies(collisionEventStreamHandle, triggerEventStreamHandle);

                float invNumIterations = math.rcp(numIterations);

                var phaseInfoPtrs = (DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo*)NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(solverSchedulerInfo.PhaseInfo);

                float2 gravityNormalized = float2.zero;
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);
                }

                for (int solverIterationId = 0; solverIterationId < numIterations; solverIterationId++)
                {
                    bool firstIteration = solverIterationId == 0;
                    bool lastIteration = solverIterationId == numIterations - 1;
                    for (int phaseId = 0; phaseId < numPhases; phaseId++)
                    {
                        var job = new ParallelSolverJob
                        {
                            JacobiansReader = jacobians.AsReader(),
                            PhaseIndex = phaseId,
                            Phases = solverSchedulerInfo.PhaseInfo,
                            MotionVelocities = dynamicsWorld.BodyMotionVelocity,
                            SolverStabilizationData = solverStabilizationData,
                            StepInput = new StepInput
                            {
                                InvNumSolverIterations = invNumIterations,
                                IsFirstIteration = firstIteration,
                                IsLastIteration = lastIteration,
                                Timestep = timestep,
                                InvTimestep = timestep > 0.0f ? 1.0f / timestep : 0.0f
                            }
                        };

                        // Only initialize event writers for last solver iteration jobs
                        if (lastIteration)
                        {
                            job.CollisionEventsWriter = collisionEvents.AsWriter();
                            job.TriggerEventsWriter = triggerEvents.AsWriter();
                        }

                        // NOTE: The last phase must be executed on a single job since it
                        // int.MaxValue can't be used as batchSize since 19.1 overflows in that case...
                        bool isLastPhase = phaseId == numPhases - 1;
                        int batchSize = isLastPhase ? (int.MaxValue / 2) : 1;

                        int* numWorkItems = &(phaseInfoPtrs[phaseId].NumWorkItems);
                        handle = job.Schedule(numWorkItems, batchSize, handle);
                    }

                    // Stabilize velocities
                    if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                    {
                        var stabilizeVelocitiesJob = new StabilizeVelocitiesJob
                        {
                            MotionVelocities = dynamicsWorld.BodyMotionVelocity,
                            SolverStabilizationData = solverStabilizationData,
                            GravityPerStep = solverStabilizationData.Gravity * timestep,
                            GravityNormalized = gravityNormalized,
                            IsFirstIteration = firstIteration
                        };

                        handle = stabilizeVelocitiesJob.Schedule(dynamicsWorld.BodyMotionCount, 64, handle);
                    }
                }
            }

            // Dispose processed data
            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                jacobians.Dispose(handle),
                solverSchedulerInfo.Dispose(handle));
            returnHandles.FinalExecutionHandle = handle;

            return returnHandles;
        }


        #region Jobs

        [BurstCompile]
        struct ParallelApplyGravityAndCopyInputVelocitiesJob : IJobParallelFor
        {
            public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;
            public NativeArray<PhysicsVelocity> InputVelocities;
            public float2 GravityAcceleration;

            public void Execute(int index)
            {
                Execute(index, GravityAcceleration, BodyMotionVelocity, InputVelocities);
            }

            internal static void Execute(
                int index,
                float2 gravityAcceleration,
                NativeArray<PhysicsBody.MotionVelocity> bodyMotionVelocity,
                NativeArray<PhysicsVelocity> inputVelocities)
            {
                var motionVelocity = bodyMotionVelocity[index];

                // Apply gravity
                motionVelocity.LinearVelocity += gravityAcceleration * motionVelocity.GravityFactor;

                // Write back
                bodyMotionVelocity[index] = motionVelocity;

                // Make a copy
                inputVelocities[index] = new PhysicsVelocity
                {
                    LinearFloat = motionVelocity.LinearVelocity,
                    AngularFloat = motionVelocity.AngularVelocity
                };
            }
        }

        [BurstCompile]
        struct ApplyGravityAndCopyInputVelocitiesJob : IJob
        {
            public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;
            public NativeArray<PhysicsVelocity> InputVelocities;
            public float2 GravityAcceleration;

            public void Execute()
            {
                ApplyGravityAndCopyInputVelocities(BodyMotionVelocity, InputVelocities, GravityAcceleration);
            }
        }

        [BurstCompile]
        private struct ParallelBuildJacobiansJob : IJobParallelForDefer
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public float TimeStep;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSolverIterations;
            public float InvTimeStep;
            public float GravityAcceleration;
            [ReadOnly] public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

            public void Execute(int workItemIndex)
            {
                int firstDispatchPairIndex = SolverSchedulerInfo.GetWorkItemReadOffset(workItemIndex, out int dispatchPairCount);

                ContactsReader.BeginForEachIndex(workItemIndex);
                JacobiansWriter.BeginForEachIndex(workItemIndex);
                BuildJacobians(ref World, TimeStep, InvTimeStep, GravityAcceleration, NumSolverIterations,
                    DispatchPairs, firstDispatchPairIndex, dispatchPairCount,
                    ref ContactsReader, ref JacobiansWriter);
            }
        }

        [BurstCompile]
        private struct BuildJacobiansJob : IJob
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public float TimeStep;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSolverIterations;
            public float2 Gravity;

            public void Execute()
            {
                BuildJacobians(ref World, TimeStep, Gravity, NumSolverIterations,
                    DispatchPairs, ref ContactsReader, ref JacobiansWriter);
            }
        }

        [BurstCompile]
        [NoAlias]
        private struct ParallelSolverJob : IJobParallelForDefer
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<PhysicsBody.MotionVelocity> MotionVelocities;

            [NativeDisableParallelForRestriction]
            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            [ReadOnly]
            public NativeArray<DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo> Phases;
            public int PhaseIndex;
            public StepInput StepInput;

            public void Execute(int workItemIndex)
            {
                int workItemStartIndexOffset = Phases[PhaseIndex].FirstWorkItemIndex;

                CollisionEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                TriggerEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);

                Solve(MotionVelocities, ref JacobiansReader, ref CollisionEventsWriter, ref TriggerEventsWriter,
                    workItemIndex + workItemStartIndexOffset, StepInput, SolverStabilizationData);
            }
        }

        [BurstCompile]
        [NoAlias]
        private struct SolverJob : IJob
        {
            public NativeArray<PhysicsBody.MotionVelocity> MotionVelocities;

            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            public int NumIterations;
            public float Timestep;

            public void Execute()
            {
                SolveJacobians(ref JacobiansReader, MotionVelocities, Timestep, NumIterations,
                    ref CollisionEventsWriter, ref TriggerEventsWriter, SolverStabilizationData);
            }
        }

        [BurstCompile]
        private struct StabilizeVelocitiesJob : IJobParallelFor
        {
            public NativeArray<PhysicsBody.MotionVelocity> MotionVelocities;
            public StabilizationData SolverStabilizationData;
            public float2 GravityPerStep;
            public float2 GravityNormalized;
            public bool IsFirstIteration;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionVelocities, IsFirstIteration, GravityPerStep, GravityNormalized, SolverStabilizationData);
            }

            internal static void ExecuteImpl(int i, NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
                bool isFirstIteration, float2 gravityPerStep, float2 gravityNormalized,
                StabilizationData solverStabilizationData)
            {
                var motionData = solverStabilizationData.MotionData[i];
                int numPairs = motionData.NumPairs;
                if (numPairs == 0)
                {
                    return;
                }

                PhysicsBody.MotionVelocity motionVelocity = motionVelocities[i];

                // Skip kinematic bodies
                if (motionVelocity.InverseMass == 0.0f)
                {
                    return;
                }

                // Scale up inertia for other iterations
                if (isFirstIteration && numPairs > 1)
                {
                    float inertiaScale = 1.0f + 0.2f * (numPairs - 1) * solverStabilizationData.StabilizationHeuristicSettings.InertiaScalingFactor;
                    motionData.InverseInertiaScale = math.rcp(inertiaScale);
                    solverStabilizationData.MotionData[i] = motionData;
                }

                // Don't stabilize velocity component along the gravity vector
                float2 linVelVertical = math.dot(motionVelocity.LinearVelocity, gravityNormalized) * gravityNormalized;
                float2 linVelSideways = motionVelocity.LinearVelocity - linVelVertical;

                // Choose a very small gravity coefficient for clipping threshold
                float gravityCoefficient = (numPairs == 1 ? 0.1f : 0.25f) * solverStabilizationData.StabilizationHeuristicSettings.VelocityClippingFactor;

                // Linear velocity threshold
                float smallLinVelThresholdSq = math.lengthsq(gravityPerStep * motionVelocity.GravityFactor * gravityCoefficient);

                // Stabilize the velocities
                if (math.lengthsq(linVelSideways) < smallLinVelThresholdSq)
                {
                    motionVelocity.LinearVelocity = linVelVertical;

                    // Only clip angular if in contact with at least 2 bodies
                    if (numPairs > 1)
                    {
                        // Angular velocity threshold
                        if (motionVelocity.AngularExpansionFactor > 0.0f)
                        {
                            float angularFactorSq = math.rcp(motionVelocity.AngularExpansionFactor * motionVelocity.AngularExpansionFactor) * 0.01f;
                            float smallAngVelThresholdSq = smallLinVelThresholdSq * angularFactorSq;
                            if (math.lengthsq(motionVelocity.AngularVelocity) < smallAngVelThresholdSq)
                            {
                                motionVelocity.AngularVelocity = 0f;
                            }
                        }
                    }

                    // Write back
                    motionVelocities[i] = motionVelocity;
                }
            }
        }

        #endregion

        #region Implementation

        private static void GetMotions(
            PhysicsBody.IndexPair pair,
            ref NativeArray<PhysicsBody.MotionData> motionDatas,
            ref NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            out PhysicsBody.MotionVelocity velocityA,
            out PhysicsBody.MotionVelocity velocityB,
            out PhysicsTransform worldFromA,
            out PhysicsTransform worldFromB)
        {
            bool bodyAIsStatic = pair.PhysicsBodyIndexA >= motionVelocities.Length;
            bool bodyBIsStatic = pair.PhysicsBodyIndexB >= motionVelocities.Length;

            if (bodyAIsStatic)
            {
                if (bodyBIsStatic)
                {
                    Assert.IsTrue(false); // static-static pairs should have been filtered during broadphase overlap test
                    velocityA = PhysicsBody.MotionVelocity.Zero;
                    velocityB = PhysicsBody.MotionVelocity.Zero;
                    worldFromA = PhysicsTransform.Identity;
                    worldFromB = PhysicsTransform.Identity;
                    return;
                }

                velocityA = PhysicsBody.MotionVelocity.Zero;
                velocityB = motionVelocities[pair.PhysicsBodyIndexB];

                worldFromA = PhysicsTransform.Identity;
                worldFromB = motionDatas[pair.PhysicsBodyIndexB].WorldTransform;
            }
            else if (bodyBIsStatic)
            {
                velocityA = motionVelocities[pair.PhysicsBodyIndexA];
                velocityB = PhysicsBody.MotionVelocity.Zero;

                worldFromA = motionDatas[pair.PhysicsBodyIndexA].WorldTransform;
                worldFromB = PhysicsTransform.Identity;
            }
            else
            {
                velocityA = motionVelocities[pair.PhysicsBodyIndexA];
                velocityB = motionVelocities[pair.PhysicsBodyIndexB];

                worldFromA = motionDatas[pair.PhysicsBodyIndexA].WorldTransform;
                worldFromB = motionDatas[pair.PhysicsBodyIndexB].WorldTransform;
            }
        }

        private static unsafe void BuildJacobians(
            ref PhysicsWorld world,
            float timestep,
            float invTimestep,
            float gravityAcceleration,
            int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            int firstDispatchPairIndex,
            int dispatchPairCount,
            ref NativeStream.Reader contactReader,
            ref NativeStream.Writer jacobianWriter)
        {
            // Contact resting velocity for restitution
            float negContactRestingVelocity = -gravityAcceleration * timestep;

            for (int i = 0; i < dispatchPairCount; i++)
            {
                var pair = dispatchPairs[i + firstDispatchPairIndex];
                if (!pair.IsValid)
                {
                    continue;
                }

                var motionDatas = world.BodyMotionData;
                var motionVelocities = world.BodyMotionVelocity;
                var bodies = world.AllBodies;

                if (pair.IsContact)
                {
                    while (contactReader.RemainingItemCount > 0)
                    {
                        // Check if this is the matching contact
                        {
                            var header = contactReader.Peek<ContactHeader>();
                            if (pair.PhysicsBodyIndexA != header.BodyPair.PhysicsBodyIndexA ||
                                pair.PhysicsBodyIndexB != header.BodyPair.PhysicsBodyIndexB)
                            {
                                break;
                            }
                        }

                        ref ContactHeader contactHeader = ref contactReader.Read<ContactHeader>();
                        GetMotions(contactHeader.BodyPair, ref motionDatas, ref motionVelocities,
                            out PhysicsBody.MotionVelocity velocityA,
                            out PhysicsBody.MotionVelocity velocityB,
                            out PhysicsTransform worldFromA,
                            out PhysicsTransform worldFromB);

                        float sumInvMass = velocityA.InverseMass + velocityB.InverseMass;
                        bool bodiesHaveInfiniteMass = velocityA.HasInfiniteInertiaAndMass && velocityB.HasInfiniteInertiaAndMass;

                        // Skip contact between infinite mass bodies which don't want to raise events. These cannot have any effect during solving.
                        // These should not normally appear, because the collision detector doesn't generate such contacts.
                        if (bodiesHaveInfiniteMass)
                        {
                            if ((contactHeader.JacobianFlags & (JacobianFlags.IsTrigger | JacobianFlags.EnableCollisionEvents)) == 0)
                            {
                                for (int j = 0; j < contactHeader.NumContacts; j++)
                                {
                                    contactReader.Read<ContactPoint>();
                                }
                                continue;
                            }
                        }

                        JacobianType jacType = ((int)(contactHeader.JacobianFlags) & (int)(JacobianFlags.IsTrigger)) != 0 ?
                            JacobianType.Trigger : JacobianType.Contact;

                        // Write size before every jacobian and allocate all necessary data for this jacobian
                        int jacobianSize = JacobianHeader.CalculateSize(jacType, contactHeader.JacobianFlags, contactHeader.NumContacts);
                        jacobianWriter.Write(jacobianSize);
                        byte* jacobianPtr = jacobianWriter.Allocate(jacobianSize);

                        ref JacobianHeader jacobianHeader = ref UnsafeUtility.AsRef<JacobianHeader>(jacobianPtr);
                        jacobianHeader.BodyPair = contactHeader.BodyPair;
                        jacobianHeader.Type = jacType;
                        jacobianHeader.Flags = contactHeader.JacobianFlags;

                        var baseJac = new BaseContactJacobian
                        {
                            NumContacts = contactHeader.NumContacts,
                            Normal = contactHeader.Normal
                        };

                        // Body A must be dynamic
                        Assert.IsTrue(contactHeader.BodyPair.PhysicsBodyIndexA < motionVelocities.Length);
                        bool isDynamicStaticPair = contactHeader.BodyPair.PhysicsBodyIndexB >= motionVelocities.Length;

                        // If contact distance is negative, use an artificially reduced penetration depth to prevent the dynamic-dynamic contacts from depenetrating too quickly
                        float maxDepenetrationVelocity = isDynamicStaticPair ? float.MaxValue : 3.0f; // meter/seconds time step independent

                        if (jacobianHeader.Type == JacobianType.Contact)
                        {
                            ref ContactJacobian contactJacobian = ref jacobianHeader.AccessBaseJacobian<ContactJacobian>();
                            contactJacobian.BaseJacobian = baseJac;
                            contactJacobian.CoefficientOfFriction = contactHeader.CoefficientOfFriction;

                            // Indicator whether restitution will be applied,
                            // used to scale down friction on bounce.
                            bool applyRestitution = false;

                            // Initialize modifier data (in order from JacobianModifierFlags) before angular jacobians
                            InitModifierData(ref jacobianHeader, contactHeader.ColliderKeys,
                                new EntityPair { EntityA = bodies[contactHeader.BodyPair.PhysicsBodyIndexA].Entity, EntityB = bodies[contactHeader.BodyPair.PhysicsBodyIndexB].Entity });

                            // Build normal jacobians
                            var centerA = new float2(0.0f);
                            var centerB = new float2(0.0f);
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                // Build the jacobian
                                BuildContactJacobian(
                                    j, contactJacobian.BaseJacobian.Normal, worldFromA, worldFromB, invTimestep, velocityA, velocityB, sumInvMass, maxDepenetrationVelocity,
                                    ref jacobianHeader, ref centerA, ref centerB, ref contactReader);

                                // Restitution (optional)
                                if (contactHeader.CoefficientOfRestitution > 0.0f)
                                {
                                    ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(j);
                                    float relativeVelocity = BaseContactJacobian.GetJacVelocity(baseJac.Normal, jacAngular.Jac,
                                        velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                                    float dv = jacAngular.VelToReachCp - relativeVelocity;
                                    if (dv > 0.0f && relativeVelocity < negContactRestingVelocity)
                                    {
                                        // Restitution impulse is applied as if contact point is on the contact plane.
                                        // However, it can (and will) be slightly away from contact plane at the moment restitution is applied.
                                        // So we have to apply vertical shot equation to make sure we don't gain energy:
                                        // effectiveRestitutionVelocity^2 = restitutionVelocity^2 - 2.0f * gravityAcceleration * distanceToGround
                                        // From this formula we calculate the effective restitution velocity, which is the velocity
                                        // that the contact point needs to reach the same height from current position
                                        // as if it was shot with the restitutionVelocity from the contact plane.
                                        // ------------------------------------------------------------
                                        // This is still an approximation for 2 reasons:
                                        // - We are assuming the contact point will hit the contact plane with its current velocity,
                                        // while actually it would have a portion of gravity applied before the actual hit. However,
                                        // that velocity increase is quite small (less than gravity in one step), so it's safe
                                        // to use current velocity instead.
                                        // - gravityAcceleration is the actual value of gravity applied only when contact plane is
                                        // directly opposite to gravity direction. Otherwise, this value will only be smaller.
                                        // However, since this can only result in smaller bounce than the "correct" one, we can
                                        // safely go with the default gravity value in all cases.
                                        float restitutionVelocity = (relativeVelocity - negContactRestingVelocity) * contactHeader.CoefficientOfRestitution;
                                        float distanceToGround = math.max(-jacAngular.VelToReachCp * timestep, 0.0f);
                                        float effectiveRestitutionVelocity =
                                            math.sqrt(math.max(restitutionVelocity * restitutionVelocity - 2.0f * gravityAcceleration * distanceToGround, 0.0f));

                                        jacAngular.VelToReachCp =
                                            math.max(jacAngular.VelToReachCp - effectiveRestitutionVelocity, 0.0f) +
                                            effectiveRestitutionVelocity;

                                        // Remember that restitution should be applied
                                        applyRestitution = true;
                                    }
                                }
                            }

                            // Build friction jacobians
                            // (skip friction between two infinite-mass objects)
                            if (!bodiesHaveInfiniteMass)
                            {
                                // Clear accumulated impulse
                                contactJacobian.Friction0.Impulse = 0.0f;
                                contactJacobian.Friction1.Impulse = 0.0f;
                                contactJacobian.AngularFriction.Impulse = 0.0f;

                                // Calculate average position
                                float invNumContacts = math.rcp(contactJacobian.BaseJacobian.NumContacts);
                                centerA *= invNumContacts;
                                centerB *= invNumContacts;
                                float3 normal3d = math.float3(contactJacobian.BaseJacobian.Normal, 0);

                                // Choose friction axes
                                PhysicsMath.CalculatePerpendicularNormalized(normal3d, out float3 frictionDir0, out float3 frictionDir1);

                                // Build linear jacobian
                                float invEffectiveMass0, invEffectiveMass1;
                                {
                                    float3 armA = math.float3(centerA, 0);
                                    float3 armB = math.float3(centerB, 0);
                                    BuildJacobian(worldFromA, worldFromB, frictionDir0, armA, armB, velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                                        out contactJacobian.Friction0.AngularA, out contactJacobian.Friction0.AngularB, out invEffectiveMass0);
                                    BuildJacobian(worldFromA, worldFromB, frictionDir1, armA, armB, velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                                        out contactJacobian.Friction1.AngularA, out contactJacobian.Friction1.AngularB, out invEffectiveMass1);
                                }

                                // Build angular jacobian   CAN STAY IN 2D
                                float invEffectiveMassAngular;
                                {
                                    contactJacobian.AngularFriction.AngularA = math.mul(worldFromA.InverseRotation3x3, normal3d);
                                    contactJacobian.AngularFriction.AngularB = math.mul(worldFromB.InverseRotation3x3, -normal3d);
                                    float3 temp = contactJacobian.AngularFriction.AngularA * contactJacobian.AngularFriction.AngularA * velocityA.InverseInertia;
                                    temp += contactJacobian.AngularFriction.AngularB * contactJacobian.AngularFriction.AngularB * velocityB.InverseInertia;
                                    invEffectiveMassAngular = math.csum(temp);
                                }

                                // Build effective mass
                                {
                                    // Build the inverse effective mass matrix
                                    var invEffectiveMassDiag = new float3(invEffectiveMass0, invEffectiveMass1, invEffectiveMassAngular);
                                    var invEffectiveMassOffDiag = new float3( // (0, 1), (0, 2), (1, 2)
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.Friction1.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction0.AngularB, contactJacobian.Friction1.AngularB, velocityB.InverseInertia),
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction0.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia),
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction1.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction1.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia));

                                    // Invert the matrix and store it to the jacobians
                                    if (!JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out float3 effectiveMassDiag, out float3 effectiveMassOffDiag))
                                    {
                                        // invEffectiveMass can be singular if the bodies have infinite inertia about the normal.
                                        // In that case angular friction does nothing so we can regularize the matrix, set col2 = row2 = (0, 0, 1)
                                        invEffectiveMassOffDiag.y = 0.0f;
                                        invEffectiveMassOffDiag.z = 0.0f;
                                        invEffectiveMassDiag.z = 1.0f;
                                        bool success = JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out effectiveMassDiag, out effectiveMassOffDiag);
                                        Assert.IsTrue(success); // it should never fail, if it does then friction will be disabled
                                    }
                                    contactJacobian.Friction0.EffectiveMass = effectiveMassDiag.x;
                                    contactJacobian.Friction1.EffectiveMass = effectiveMassDiag.y;
                                    contactJacobian.AngularFriction.EffectiveMass = effectiveMassDiag.z;
                                    contactJacobian.FrictionEffectiveMassOffDiag = effectiveMassOffDiag;
                                }

                                // Reduce friction to 1/4 of the impulse if there will be restitution
                                if (applyRestitution)
                                {
                                    contactJacobian.Friction0.EffectiveMass *= 0.25f;
                                    contactJacobian.Friction1.EffectiveMass *= 0.25f;
                                    contactJacobian.AngularFriction.EffectiveMass *= 0.25f;
                                    contactJacobian.FrictionEffectiveMassOffDiag *= 0.25f;
                                }
                            }
                        }
                        // Much less data needed for triggers
                        else
                        {
                            ref TriggerJacobian triggerJacobian = ref jacobianHeader.AccessBaseJacobian<TriggerJacobian>();

                            triggerJacobian.BaseJacobian = baseJac;
                            triggerJacobian.ColliderKeys = contactHeader.ColliderKeys;
                            triggerJacobian.Entities = new EntityPair
                            {
                                EntityA = bodies[contactHeader.BodyPair.PhysicsBodyIndexA].Entity,
                                EntityB = bodies[contactHeader.BodyPair.PhysicsBodyIndexB].Entity
                            };

                            // Build normal jacobians
                            var centerA = new float2(0.0f); // fbessette: unused here
                            var centerB = new float2(0.0f);
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                // Build the jacobian
                                BuildContactJacobian(
                                    j, triggerJacobian.BaseJacobian.Normal, worldFromA, worldFromB, invTimestep, velocityA, velocityB, sumInvMass, maxDepenetrationVelocity,
                                    ref jacobianHeader, ref centerA, ref centerB, ref contactReader);
                            }
                        }
                    }
                }
                else
                {
                    //Joint joint = world.Joints[pair.JointIndex];
                    //// Need to fetch the real body indices from the joint, as the scheduler may have reordered them
                    //int bodyIndexA = joint.BodyPair.PhysicsBodyIndexA;
                    //int bodyIndexB = joint.BodyPair.PhysicsBodyIndexB;

                    //GetMotion(ref world, bodyIndexA, out MotionVelocity velocityA, out MotionData motionA);
                    //GetMotion(ref world, bodyIndexB, out MotionVelocity velocityB, out MotionData motionB);

                    //BuildJointJacobian(joint, velocityA, velocityB,
                    //    motionA, motionB, timestep, numSolverIterations, ref jacobianWriter);
                }
            }

            contactReader.EndForEachIndex();
            jacobianWriter.EndForEachIndex();
        }

        private static void InitModifierData(ref JacobianHeader jacobianHeader, ColliderKeyPair colliderKeys, EntityPair entities)
        {
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessColliderKeys() = colliderKeys;
                jacobianHeader.AccessEntities() = entities;
            }
            if (jacobianHeader.HasSurfaceVelocity)
            {
                jacobianHeader.AccessSurfaceVelocity() = new SurfaceVelocity();
            }
            if (jacobianHeader.HasMassFactors)
            {
                jacobianHeader.AccessMassFactors() = MassFactors.Default;
            }
        }

        private static void BuildJacobian(PhysicsTransform worldFromA, PhysicsTransform worldFromB, float3 normal, float3 armA, float3 armB,
            float3 invInertiaA, float3 invInertiaB, float sumInvMass, out float3 angularA, out float3 angularB, out float invEffectiveMass)
        {
            /* Was simplified from this:
             * float3 crossA = math.cross(armA, normal);
             * angularA = math.mul(worldFromA.InverseRotation, crossA).xyz;
             * 
             * float3 crossB = math.cross(normal, armB);
             * angularB = math.mul(worldFromB.InverseRotation, crossB).xyz;
             * 
             * to
             * 
             * angularA = math.cross(math.float3(armA, 0), math.float3(normal, 0)).z;
             * angularB = math.cross(math.float3(normal, 0), math.float3(armB, 0)).z;
             * 
             * since crossA and crossB will always be in the Z axis in 2D, multiplying with the inverse rotation will have no effect
             */

            float3 crossA = math.cross(armA, normal);
            angularA = math.mul(worldFromA.InverseRotation3x3, crossA).xyz;

            float3 crossB = math.cross(normal, armB);
            angularB = math.mul(worldFromB.InverseRotation3x3, crossB).xyz;

            float3 temp = angularA * angularA * invInertiaA + angularB * angularB * invInertiaB;
            invEffectiveMass = temp.x + temp.y + temp.z + sumInvMass;
        }

        private static void BuildContactJacobian(
            int contactPointIndex,
            float2 normal,
            PhysicsTransform worldFromA,
            PhysicsTransform worldFromB,
            float invTimestep,
            PhysicsBody.MotionVelocity velocityA,
            PhysicsBody.MotionVelocity velocityB,
            float sumInvMass,
            float maxDepenetrationVelocity,
            ref JacobianHeader jacobianHeader,
            ref float2 centerA,
            ref float2 centerB,
            ref NativeStream.Reader contactReader)
        {
            ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(contactPointIndex);
            ContactPoint contact = contactReader.Read<ContactPoint>();
            float2 pointOnB = contact.Position;
            float2 pointOnA = contact.Position + normal * contact.Distance;
            float2 armA = pointOnA - worldFromA.Translation;
            float2 armB = pointOnB - worldFromB.Translation;
            BuildJacobian(worldFromA, worldFromB, math.float3(normal, 0), math.float3(armA, 0), math.float3(armB, 0), velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                out jacAngular.Jac.AngularA, out jacAngular.Jac.AngularB, out float invEffectiveMass);
            jacAngular.Jac.EffectiveMass = 1.0f / invEffectiveMass;
            jacAngular.Jac.Impulse = 0.0f;

            float solveDistance = contact.Distance;
            float solveVelocity = solveDistance * invTimestep;

            solveVelocity = math.max(-maxDepenetrationVelocity, solveVelocity);

            jacAngular.VelToReachCp = -solveVelocity;

            // Calculate average position for friction
            centerA += armA;
            centerB += armB;

            // Write the contact point to the jacobian stream if requested
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessContactPoint(contactPointIndex) = contact;
            }
        }

        // Updates data for solver stabilization heuristic.
        // Updates number of pairs for dynamic bodies and resets inverse inertia scale in first iteration.
        // Also prepares motion stabilization solver data for current Jacobian to solve.
        private static void SolverStabilizationUpdate(
            ref JacobianHeader header, bool isFirstIteration,
            PhysicsBody.MotionVelocity velocityA, PhysicsBody.MotionVelocity velocityB,
            StabilizationData solverStabilizationData,
            ref MotionStabilizationInput motionStabilizationSolverInputA,
            ref MotionStabilizationInput motionStabilizationSolverInputB)
        {
            // Solver stabilization heuristic, count pairs and reset inverse inertia scale only in first iteration
            var inputVelocities = solverStabilizationData.InputVelocities;
            var motionData = solverStabilizationData.MotionData;
            if (isFirstIteration)
            {
                // Only count heavier (or up to 2 times lighter) bodies as pairs
                // Also reset inverse inertia scale
                if (header.BodyPair.PhysicsBodyIndexA < motionData.Length)
                {
                    var data = motionData[header.BodyPair.PhysicsBodyIndexA];
                    if (0.5f * velocityB.InverseMass <= velocityA.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = 1.0f;
                    motionData[header.BodyPair.PhysicsBodyIndexA] = data;
                }
                if (header.BodyPair.PhysicsBodyIndexB < motionData.Length)
                {
                    var data = motionData[header.BodyPair.PhysicsBodyIndexB];
                    if (0.5f * velocityA.InverseMass <= velocityB.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = 1.0f;
                    motionData[header.BodyPair.PhysicsBodyIndexB] = data;
                }
            }

            // Motion solver input stabilization data
            {
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities)
                {
                    motionStabilizationSolverInputA.InputVelocity = header.BodyPair.PhysicsBodyIndexA < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.PhysicsBodyIndexA] : PhysicsVelocity.Zero;
                    motionStabilizationSolverInputB.InputVelocity = header.BodyPair.PhysicsBodyIndexB < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.PhysicsBodyIndexB] : PhysicsVelocity.Zero;
                }

                motionStabilizationSolverInputA.InverseInertiaScale = header.BodyPair.PhysicsBodyIndexA < motionData.Length ?
                    motionData[header.BodyPair.PhysicsBodyIndexA].InverseInertiaScale : 1.0f;
                motionStabilizationSolverInputB.InverseInertiaScale = header.BodyPair.PhysicsBodyIndexB < motionData.Length ?
                    motionData[header.BodyPair.PhysicsBodyIndexB].InverseInertiaScale : 1.0f;
            }
        }

        private static void Solve(
            NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            [NoAlias] ref NativeStream.Reader jacobianReader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter,
            [NoAlias] ref NativeStream.Writer triggerEventsWriter,
            int workItemIndex, StepInput stepInput,
            StabilizationData solverStabilizationData)
        {
            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.BeginForEachIndex(workItemIndex);
                triggerEventsWriter.BeginForEachIndex(workItemIndex);
            }

            MotionStabilizationInput motionStabilizationSolverInputA = MotionStabilizationInput.Default;
            MotionStabilizationInput motionStabilizationSolverInputB = MotionStabilizationInput.Default;

            var jacIterator = new JacobianIterator(jacobianReader, workItemIndex);
            while (jacIterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();

                // Static-static pairs should have been filtered during broadphase overlap test
                Assert.IsTrue(header.BodyPair.PhysicsBodyIndexA < motionVelocities.Length || header.BodyPair.PhysicsBodyIndexB < motionVelocities.Length);

                // Get the motion pair
                PhysicsBody.MotionVelocity velocityA = header.BodyPair.PhysicsBodyIndexA < motionVelocities.Length ? motionVelocities[header.BodyPair.PhysicsBodyIndexA] : PhysicsBody.MotionVelocity.Zero;
                PhysicsBody.MotionVelocity velocityB = header.BodyPair.PhysicsBodyIndexB < motionVelocities.Length ? motionVelocities[header.BodyPair.PhysicsBodyIndexB] : PhysicsBody.MotionVelocity.Zero;

                // Solver stabilization
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization && header.Type == JacobianType.Contact)
                {
                    SolverStabilizationUpdate(ref header, stepInput.IsFirstIteration, velocityA, velocityB, solverStabilizationData,
                        ref motionStabilizationSolverInputA, ref motionStabilizationSolverInputB);
                }

                // Solve the jacobian
                header.Solve(ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter, ref triggerEventsWriter,
                    solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization &&
                    solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities,
                    motionStabilizationSolverInputA, motionStabilizationSolverInputB);

                // Write back velocity for dynamic bodies
                if (header.BodyPair.PhysicsBodyIndexA < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.PhysicsBodyIndexA] = velocityA;
                }
                if (header.BodyPair.PhysicsBodyIndexB < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.PhysicsBodyIndexB] = velocityB;
                }
            }

            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.EndForEachIndex();
                triggerEventsWriter.EndForEachIndex();
            }
        }

        private static void StabilizeVelocities(NativeArray<PhysicsBody.MotionVelocity> motionVelocities,
            bool isFirstIteration, float timeStep, StabilizationData solverStabilizationData)
        {
            float2 gravityPerStep = solverStabilizationData.Gravity * timeStep;
            float2 gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);
            for (int i = 0; i < motionVelocities.Length; i++)
            {
                StabilizeVelocitiesJob.ExecuteImpl(i, motionVelocities, isFirstIteration,
                    gravityPerStep, gravityNormalized, solverStabilizationData);
            }
        }
        #endregion

    }
}
