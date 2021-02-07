using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityEngine;

namespace CCC.Fix2D
{
    // Default simulation implementation
    public class DefaultSimulation : ISimulation
    {
        public SimulationType Type => SimulationType.Default;
        public JobHandle FinalSimulationJobHandle => m_StepHandles.FinalExecutionHandle;
        public JobHandle FinalJobHandle => JobHandle.CombineDependencies(FinalSimulationJobHandle, m_StepHandles.FinalDisposeHandle);

        internal SimulationContext SimulationContext;
        internal StepContext StepContext = new StepContext();

        SimulationJobHandles m_StepHandles = new SimulationJobHandles(new JobHandle());
        private readonly DispatchPairSequencer m_Scheduler = new DispatchPairSequencer();

        // Schedule all the jobs for the simulation step.
        public unsafe SimulationJobHandles ScheduleStepJobs(PhysicsWorld world, PhysicsCallbacks physicsCallbacks, JobHandle inputDeps)
        {
            PhysicsSettings settings = world.Settings;
            bool singleThreadedSim = (settings.NumberOfThreadsHint <= 0);

            SafetyChecks.IsFalse(world.TimeStep < 0f);

            StepContext = new StepContext();

            SimulationContext.Reset(ref world);
            SimulationContext.TimeStep = world.TimeStep;

            if (world.DynamicBodyCount == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(inputDeps);
                return m_StepHandles;
            }


            // Execute phase callback.
            JobHandle handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PreStepSimulation, ref world, inputDeps);


            ////////////////////////////////////////////////////////////////////////////////////////
            //      Find Overlaps
            ////////////////////////////////////////////////////////////////////////////////////////
            // Find all body pairs that overlap in the broadphase
            SimulationJobHandles handles = world.CollisionWorld.ScheduleFindOverlapsJobs(
                out NativeStream dynamicVsDynamicBodyPairs, out NativeStream dynamicVsStaticBodyPairs, handle);

            handle = handles.FinalExecutionHandle;
            var disposeHandle1 = handles.FinalDisposeHandle;
            var postOverlapsHandle = handle;


            ////////////////////////////////////////////////////////////////////////////////////////
            //      Sort Overlaps
            ////////////////////////////////////////////////////////////////////////////////////////
            // Sort all overlapping and jointed body pairs into phases
            handles = m_Scheduler.ScheduleCreatePhasedDispatchPairsJob(ref world, ref dynamicVsDynamicBodyPairs, ref dynamicVsStaticBodyPairs, handle,
                ref StepContext.PhasedDispatchPairs, out StepContext.SolverSchedulerInfo, settings.NumberOfThreadsHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle2 = handles.FinalDisposeHandle;

            disposeHandle2.Complete();


            ////////////////////////////////////////////////////////////////////////////////////////
            //      Apply Gravity
            ////////////////////////////////////////////////////////////////////////////////////////
            // Apply gravity and copy input velocities at this point (in parallel with the scheduler, but before the callbacks)
            handle = Solver.ScheduleApplyGravityAndCopyInputVelocitiesJob(
                ref world.DynamicsWorld, SimulationContext.InputVelocities, world.TimeStep * settings.Gravity, handle, settings.NumberOfThreadsHint);
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateOverlapBodies, ref world, handle);


            ////////////////////////////////////////////////////////////////////////////////////////
            //      Create Contact Points & Joint Jacobians
            ////////////////////////////////////////////////////////////////////////////////////////

            // Create contact points & joint Jacobians
            handles = NarrowPhase.ScheduleCreateContactsJobs(ref world, world.TimeStep,
                ref StepContext.Contacts, ref StepContext.Jacobians, ref StepContext.PhasedDispatchPairs, handle,
                ref StepContext.SolverSchedulerInfo, settings.NumberOfThreadsHint);
            handle = handles.FinalExecutionHandle;

            var disposeHandle3 = handles.FinalDisposeHandle;
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateContacts, ref world, handle);

            //handle = new LogContactsJob()
            //{
            //    Contacts = StepContext.Contacts.AsReader()
            //}.Schedule(handle);


            ////////////////////////////////////////////////////////////////////////////////////////
            //      Create Contact Jacobians
            ////////////////////////////////////////////////////////////////////////////////////////
            handles = Solver.ScheduleBuildJacobiansJobs(ref world, world.TimeStep, settings.Gravity, settings.NumSolverIterations,
                handle, ref StepContext.PhasedDispatchPairs, ref StepContext.SolverSchedulerInfo,
                ref StepContext.Contacts, ref StepContext.Jacobians, settings.NumberOfThreadsHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle4 = handles.FinalDisposeHandle;
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostCreateConstraints, ref world, handle);

            //handle = new LogJacobiansJob()
            //{
            //    Jacobians = StepContext.Jacobians.AsReader()
            //}.Schedule(handle);

            ////////////////////////////////////////////////////////////////////////////////////////
            //      Solve all Jacobians
            ////////////////////////////////////////////////////////////////////////////////////////
            Solver.StabilizationData solverStabilizationData = new Solver.StabilizationData(settings.SolverStabilizationHeuristicSettings, settings.Gravity, SimulationContext);
            handles = Solver.ScheduleSolveJacobiansJobs(ref world.DynamicsWorld, world.TimeStep, settings.NumSolverIterations,
                ref StepContext.Jacobians, ref SimulationContext.CollisionEventDataStream, ref SimulationContext.TriggerEventDataStream,
                ref StepContext.SolverSchedulerInfo, solverStabilizationData, handle, settings.NumberOfThreadsHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle5 = handles.FinalDisposeHandle;
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostSolve, ref world, handle);

            //handle = new LogTriggerEvents()
            //{
            //    TriggerEvents = SimulationContext.TriggerEvents
            //}.Schedule(handle);

            ////////////////////////////////////////////////////////////////////////////////////////
            //      Integrate Motions
            ////////////////////////////////////////////////////////////////////////////////////////
            handle = Integrator.ScheduleIntegrateJobs(ref world, handle, settings.NumberOfThreadsHint);

            ////////////////////////////////////////////////////////////////////////////////////////
            //      Re-update Collision World (optional)
            ////////////////////////////////////////////////////////////////////////////////////////
            if (settings.SynchronizeCollisionWorld)
            {
                handle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, handle);  // TODO: timeStep = 0?
            }

            // Schedule phase callback.
            handle = physicsCallbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PostIntegrate, ref world, handle);

            m_StepHandles.FinalExecutionHandle = handle;
            //m_StepHandles.FinalDisposeHandle = handle;

            // Different dispose logic for single threaded simulation compared to "standard" threading (multi threaded)
            if (singleThreadedSim)
            {
                handle = dynamicVsDynamicBodyPairs.Dispose(handle);
                handle = dynamicVsStaticBodyPairs.Dispose(handle);
                handle = StepContext.PhasedDispatchPairs.Dispose(handle);
                handle = StepContext.Contacts.Dispose(handle);
                handle = StepContext.Jacobians.Dispose(handle);
                handle = StepContext.SolverSchedulerInfo.Dispose(handle);

                m_StepHandles.FinalDisposeHandle = handle;
            }
            else
            {
                // Return the final handle, which includes disposing temporary arrays
                const int DISPOSE_COUNT = 5;
                JobHandle* disposeDeps = stackalloc JobHandle[DISPOSE_COUNT]
                {
                    disposeHandle1,
                    disposeHandle2,
                    disposeHandle3,
                    disposeHandle4,
                    disposeHandle5,
                    //StepContext.Contacts.Dispose(handle),
                    //StepContext.PhasedDispatchPairs.Dispose(handle),
                    //StepContext.Jacobians.Dispose(handle),
                    //StepContext.SolverSchedulerInfo.Dispose(handle)
                };
                m_StepHandles.FinalDisposeHandle = JobHandleUnsafeUtility.CombineDependencies(disposeDeps, DISPOSE_COUNT);
            }

            return m_StepHandles;
        }

        public void Dispose()
        {
            m_Scheduler.Dispose();
            SimulationContext.Dispose();
        }
    }

    // Temporary data created and destroyed during the step
    internal struct StepContext
    {
        // Built by the scheduler. Groups body pairs into phases in which each
        // body appears at most once, so that the interactions within each phase can be solved
        // in parallel with each other but not with other phases. This is consumed by the
        // ProcessBodyPairsJob, which outputs contact and joint Jacobians.
        public NativeList<DispatchPairSequencer.DispatchPair> PhasedDispatchPairs;

        // Built by the scheduler. Describes the grouping of PhasedBodyPairs
        // which informs how we can schedule the solver jobs and where they read info from.
        public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

        public NativeStream Contacts;
        public NativeStream Jacobians;
    }

    // Holds temporary data in a storage that lives as long as simulation lives
    // and is only re-allocated if necessary.
    public struct SimulationContext : IDisposable
    {
        private int _dynamicBodyCount;
        private NativeArray<PhysicsVelocity> m_InputVelocities;
        internal NativeArray<PhysicsVelocity> InputVelocities => m_InputVelocities.GetSubArray(0, _dynamicBodyCount);

        // Solver stabilization data (it's completely ok to be unallocated)
        [NativeDisableContainerSafetyRestriction]
        private NativeArray<Solver.StabilizationMotionData> m_SolverStabilizationMotionData;
        internal NativeArray<Solver.StabilizationMotionData> SolverStabilizationMotionData => m_SolverStabilizationMotionData.GetSubArray(0, _dynamicBodyCount);

        internal NativeStream ContactEventStream;
        internal NativeStream CollisionEventDataStream;
        internal NativeStream TriggerEventDataStream;

        internal float TimeStep;

        public CollisionEvents CollisionEvents => new CollisionEvents(CollisionEventDataStream, InputVelocities, TimeStep);
        public TriggerEvents TriggerEvents => new TriggerEvents(TriggerEventDataStream);
        //public ContactEvents ContactEvents => new ContactEvents(ContactEventStream);

        private NativeArray<int> _workItemCount;

        internal void Reset(ref PhysicsWorld world)
        {
            _dynamicBodyCount = world.DynamicBodyCount;
            if (!m_InputVelocities.IsCreated || m_InputVelocities.Length < _dynamicBodyCount)
            {
                if (m_InputVelocities.IsCreated)
                {
                    m_InputVelocities.Dispose();
                }
                m_InputVelocities = new NativeArray<PhysicsVelocity>(_dynamicBodyCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            // Solver stabilization data
            if (world.Settings.SolverStabilizationHeuristicSettings.EnableSolverStabilization)
            {
                if (!m_SolverStabilizationMotionData.IsCreated || m_SolverStabilizationMotionData.Length < _dynamicBodyCount)
                {
                    if (m_SolverStabilizationMotionData.IsCreated)
                    {
                        m_SolverStabilizationMotionData.Dispose();
                    }
                    m_SolverStabilizationMotionData = new NativeArray<Solver.StabilizationMotionData>(_dynamicBodyCount, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                }
                else if (_dynamicBodyCount > 0)
                {
                    unsafe
                    {
                        UnsafeUtility.MemClear(m_SolverStabilizationMotionData.GetUnsafePtr(), _dynamicBodyCount * UnsafeUtility.SizeOf<Solver.StabilizationMotionData>());
                    }
                }
            }

            if (ContactEventStream.IsCreated)
            {
                ContactEventStream.Dispose();
            }
            ContactEventStream = new NativeStream(1, Allocator.Persistent);

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }
            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
            }

            if (!_workItemCount.IsCreated)
            {
                _workItemCount = new NativeArray<int>(new int[] { 1 }, Allocator.Persistent);
            }
        }

        public void Dispose()
        {
            if (m_InputVelocities.IsCreated)
            {
                m_InputVelocities.Dispose();
            }
            if (ContactEventStream.IsCreated)
            {
                ContactEventStream.Dispose();
            }

            if (m_SolverStabilizationMotionData.IsCreated)
            {
                m_SolverStabilizationMotionData.Dispose();
            }

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }

            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
            }

            if (_workItemCount.IsCreated)
            {
                _workItemCount.Dispose();
            }
        }
    }

    public struct LogContactsJob : IJob
    {
        public NativeStream.Reader Contacts;

        void IJob.Execute()
        {
            Contacts.BeginForEachIndex(0);
            while (Contacts.RemainingItemCount > 0)
            {
                ref ContactHeader header = ref Contacts.Read<ContactHeader>();
                Debug.Log("collision header");

                for (int i = 0; i < header.NumContacts; i++)
                {
                    var point = Contacts.Read<ContactPoint>();
                    Debug.Log("collision point");
                }
            }
            Contacts.EndForEachIndex();
        }
    }

    public struct LogJacobiansJob : IJob
    {
        public NativeStream.Reader Jacobians;

        void IJob.Execute()
        {
            var iterator = new JacobianIterator(Jacobians, 0);
            while (iterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref iterator.ReadJacobianHeader();

                switch (header.Type)
                {
                    case JacobianType.Contact:
                        Debug.Log($"jacobian: {header.Type}");
                        break;

                    case JacobianType.Trigger:
                        ref TriggerJacobian trigger = ref header.AccessBaseJacobian<TriggerJacobian>();
                        Debug.Log($"jacobian: {header.Type} numContact: {trigger.BaseJacobian.NumContacts}  normal: {trigger.BaseJacobian.Normal}");
                        break;
                }

            }
            Jacobians.EndForEachIndex();
        }
    }

    public struct LogTriggerEvents : IJob
    {
        public TriggerEvents TriggerEvents;

        void IJob.Execute()
        {
            foreach (var item in TriggerEvents)
            {
                Debug.Log($"Trigger between {item.EntityA} and {item.EntityB}");
            }
        }
    }

    //public struct CreateContactEventsJob : IJob
    //{
    //    public NativeStream.Reader Contacts;
    //    public NativeStream.Writer Events;

    //    [ReadOnly]
    //    public NativeArray<PhysicsBody> Bodies;

    //    void IJob.Execute()
    //    {
    //        Contacts.BeginForEachIndex(0);
    //        Events.BeginForEachIndex(0);
    //        while (Contacts.RemainingItemCount > 0)
    //        {
    //            ref ContactHeader header = ref Contacts.Read<ContactHeader>();

    //            ContactPoint point = default;
    //            bool validPointFound = false;

    //            // take note of first point only
    //            for (int i = 0; i < header.NumContacts; i++)
    //            {
    //                var p = Contacts.Read<ContactPoint>(); ;
    //                if (!validPointFound && p.Distance < 0)
    //                {
    //                    point = p;
    //                    validPointFound = true;
    //                }
    //            }

    //            if (validPointFound)
    //            {
    //                var evt = new ContactEvent()
    //                {
    //                    BodyIndexA = header.BodyPair.PhysicsBodyIndexA,
    //                    BodyIndexB = header.BodyPair.PhysicsBodyIndexB,
    //                    EntityA = Bodies[header.BodyPair.PhysicsBodyIndexA].Entity,
    //                    EntityB = Bodies[header.BodyPair.PhysicsBodyIndexB].Entity,
    //                    Normal = header.Normal,
    //                    Position = point.Position
    //                };
    //                Events.Write(evt);
    //            }
    //        }
    //        Events.EndForEachIndex();
    //        Contacts.EndForEachIndex();
    //    }
    //}
}
