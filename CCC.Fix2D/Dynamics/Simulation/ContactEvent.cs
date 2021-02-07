//using System;
//using Unity.Collections;
//using Unity.Collections.LowLevel.Unsafe;
//using Unity.Entities;
//using Unity.Jobs;
//using Unity.Jobs.LowLevel.Unsafe;
//using Unity.Mathematics;

//namespace CCC.Fix2D
//{

//    // An event raised when a pair of bodies have collided during solving.
//    public struct ContactEvent
//    {
//        public Entity EntityB { get; set; }
//        public Entity EntityA { get; set; }
//        public int BodyIndexB { get; set; }
//        public int BodyIndexA { get; set; }
//        public float2 Position { get; set; }
//        public float2 Normal { get; set; }
//    }

//    // A stream of trigger events.
//    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<TriggerEvent>).
//    public struct ContactEvents /* : IEnumerable<TriggerEvent> */
//    {
//        //@TODO: Unity should have a Allow null safety restriction
//        [NativeDisableContainerSafetyRestriction]
//        private readonly NativeStream m_EventDataStream;

//        internal ContactEvents(NativeStream eventDataStream)
//        {
//            m_EventDataStream = eventDataStream;
//        }

//        public Enumerator GetEnumerator()
//        {
//            return new Enumerator(m_EventDataStream);
//        }

//        public struct Enumerator /* : IEnumerator<TriggerEvent> */
//        {
//            private NativeStream.Reader m_Reader;
//            private int m_CurrentWorkItem;
//            private readonly int m_NumWorkItems;

//            public ContactEvent Current { get; private set; }

//            internal Enumerator(NativeStream stream)
//            {
//                m_Reader = stream.IsCreated ? stream.AsReader() : new NativeStream.Reader();
//                m_CurrentWorkItem = 0;
//                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;
//                Current = default;

//                AdvanceReader();
//            }

//            public bool MoveNext()
//            {
//                if (m_Reader.RemainingItemCount > 0)
//                {
//                    Current = m_Reader.Read<ContactEvent>();
//                    AdvanceReader();
//                    return true;
//                }
//                return false;
//            }

//            private void AdvanceReader()
//            {
//                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
//                {
//                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
//                    m_CurrentWorkItem++;
//                }
//            }
//        }
//    }

//    // INTERNAL UnityPhysics interface for jobs that iterate through the list of collision events produced by the solver.
//    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement ICollisionEventsJob.
//    [JobProducerType(typeof(IContactEventsJobExtensions.CollisionEventJobProcess<>))]
//    public interface IContactEventsJob
//    {
//        void Execute(ContactEvent collisionEvent);
//    }

//    public static class IContactEventsJobExtensions
//    {
//        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
//            where T : struct, IContactEventsJob
//        {
//            // Should work only for UnityPhysics
//            if (simulation.Type != SimulationType.Default)
//            {
//                return inputDeps;
//            }

//            return ScheduleUnityPhysicsCollisionEventsJob(jobData, simulation, ref world, inputDeps);
//        }

//        // Schedules a collision events job only for UnityPhysics simulation
//        internal static unsafe JobHandle ScheduleUnityPhysicsCollisionEventsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
//            where T : struct, IContactEventsJob
//        {
//            SafetyChecks.IsTrue(SimulationType.Default == simulation.Type);

//            var data = new CollisionEventJobData<T>
//            {
//                UserJobData = jobData,
//                EventReader = ((DefaultSimulation)simulation).SimulationContext.ContactEvents
//            };

//            // Ensure the input dependencies include the end-of-simulation job, so events will have been generated
//            inputDeps = JobHandle.CombineDependencies(inputDeps, simulation.FinalSimulationJobHandle);

//            var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), CollisionEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
//            return JobsUtility.Schedule(ref parameters);
//        }

//        internal unsafe struct CollisionEventJobData<T> where T : struct
//        {
//            public T UserJobData;
//            [NativeDisableContainerSafetyRestriction] public ContactEvents EventReader;
//        }

//        internal struct CollisionEventJobProcess<T> where T : struct, IContactEventsJob
//        {
//            static IntPtr jobReflectionData;

//            public static IntPtr Initialize()
//            {
//                if (jobReflectionData == IntPtr.Zero)
//                {
//                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(CollisionEventJobData<T>),
//                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
//                }
//                return jobReflectionData;
//            }

//            public delegate void ExecuteJobFunction(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
//                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

//            public unsafe static void Execute(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
//                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
//            {
//                foreach (ContactEvent evt in jobData.EventReader)
//                {
//                    jobData.UserJobData.Execute(evt);
//                }
//            }
//        }
//    }

//}
