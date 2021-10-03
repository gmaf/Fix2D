using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace CCC.Fix2D
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of collision events produced by the solver.
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement ICollisionEventsJob.
    [JobProducerType(typeof(ICollisionEventsJobExtensions.CollisionEventJobProcess<>))]
    public interface ICollisionEventsJob
    {
        void Execute(CollisionEvent collisionEvent);
    }

    public static class ICollisionEventsJobExtensions
    {
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ICollisionEventsJob
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.Default)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsCollisionEventsJob(jobData, simulation, ref world, inputDeps);
        }

        // Schedules a collision events job only for UnityPhysics simulation
        internal static unsafe JobHandle ScheduleUnityPhysicsCollisionEventsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ICollisionEventsJob
        {
            SafetyChecks.IsTrue(SimulationType.Default == simulation.Type);

            var data = new CollisionEventJobData<T>
            {
                UserJobData = jobData,
                EventReader = ((DefaultSimulation)simulation).SimulationContext.CollisionEvents
            };

            // Ensure the input dependencies include the end-of-simulation job, so events will have been generated
            inputDeps = JobHandle.CombineDependencies(inputDeps, simulation.FinalSimulationJobHandle);

            var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), CollisionEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Parallel);
            return JobsUtility.Schedule(ref parameters);
        }

        internal unsafe struct CollisionEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public CollisionEvents EventReader;
        }

        internal struct CollisionEventJobProcess<T> where T : struct, ICollisionEventsJob
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(CollisionEventJobData<T>),
                        typeof(T), (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                foreach (CollisionEvent evt in jobData.EventReader)
                {
                    jobData.UserJobData.Execute(evt);
                }
            }
        }
    }
}