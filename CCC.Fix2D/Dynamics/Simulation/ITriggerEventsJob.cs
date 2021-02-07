using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace CCC.Fix2D
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of trigger events produced by the solver.
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement ITriggerEventsJob.
    [JobProducerType(typeof(ITriggerEventJobExtensions.TriggerEventJobProcess<>))]
    public interface ITriggerEventsJobBase
    {
        void Execute(TriggerEvent triggerEvent);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of trigger events produced by the solver.
    public interface ITriggerEventsJob : ITriggerEventsJobBase
    {
    }

#endif

    public static class ITriggerEventJobExtensions
    {
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.Default)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsTriggerEventsJob(jobData, simulation, ref world, inputDeps);
        }

        // Schedules a trigger events job only for UnityPhysics simulation
        internal static unsafe JobHandle ScheduleUnityPhysicsTriggerEventsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJobBase
        {
            SafetyChecks.IsTrue(SimulationType.Default == simulation.Type);

            var data = new TriggerEventJobData<T>
            {
                UserJobData = jobData,
                EventReader = ((DefaultSimulation)simulation).SimulationContext.TriggerEvents
            };

            // Ensure the input dependencies include the end-of-simulation job, so events will have been generated
            inputDeps = JobHandle.CombineDependencies(inputDeps, simulation.FinalSimulationJobHandle);

            var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), TriggerEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
            return JobsUtility.Schedule(ref parameters);
        }

        internal unsafe struct TriggerEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public TriggerEvents EventReader;
        }

        internal struct TriggerEventJobProcess<T> where T : struct, ITriggerEventsJobBase
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(TriggerEventJobData<T>),
                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                foreach (TriggerEvent triggerEvent in jobData.EventReader)
                {
                    jobData.UserJobData.Execute(triggerEvent);
                }
            }
        }
    }
}
