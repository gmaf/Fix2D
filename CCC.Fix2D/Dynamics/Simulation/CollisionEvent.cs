using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    // An event raised when a pair of bodies have collided during solving.
    public struct CollisionEvent
    {
        internal CollisionEventDataRef EventData;
        internal float TimeStep;
        internal PhysicsVelocity InputVelocityA;
        internal PhysicsVelocity InputVelocityB;

        public Entity EntityB => EventData.Value.Entities.EntityB;
        public Entity EntityA => EventData.Value.Entities.EntityA;
        public int BodyIndexB => EventData.Value.BodyIndices.PhysicsBodyIndexB;
        public int BodyIndexA => EventData.Value.BodyIndices.PhysicsBodyIndexA;
        public ColliderKey ColliderKeyB => EventData.Value.ColliderKeys.ColliderKeyB;
        public ColliderKey ColliderKeyA => EventData.Value.ColliderKeys.ColliderKeyA;

        public float2 Normal => EventData.Value.Normal;

        // Calculate extra details about the collision.
        // Note: Since the solver does not naturally produce this data, it requires some computation.
        public Details CalculateDetails(ref PhysicsWorld physicsWorld)
        {
            int numContactPoints = EventData.Value.NumNarrowPhaseContactPoints;
            var contactPoints = new NativeArray<ContactPoint>(numContactPoints, Allocator.Temp);
            for (int i = 0; i < numContactPoints; i++)
            {
                contactPoints[i] = EventData.Value.AccessContactPoint(i);
            }

            return EventData.Value.CalculateDetails(ref physicsWorld, TimeStep, InputVelocityA, InputVelocityB, contactPoints);
        }

        // Extra details about a collision
        public struct Details
        {
            // Estimated contact point positions (in world space).
            // Use this information about individual contact point positions
            // to apply custom logic, for example looking at the Length
            // to differentiate between vertex (1 point), edge (2 point)
            // or face (3 or more points) collision.
            public NativeArray<float2> EstimatedContactPointPositions;

            // Estimated total impulse applied
            public float EstimatedImpulse;

            // Calculate the average contact point position
            public float2 AverageContactPointPosition
            {
                get
                {
                    var pos = float2.zero;
                    for (int i = 0; i < EstimatedContactPointPositions.Length; i++)
                    {
                        pos += EstimatedContactPointPositions[i];
                    }
                    return pos / EstimatedContactPointPositions.Length;
                }
            }

            public fix2 AverageContactPointPositionFix => (fix2)AverageContactPointPosition;
        }
    }

    // A stream of collision events.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<CollisionEvent>).
    public struct CollisionEvents /* : IEnumerable<CollisionEvent> */
    {
        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly NativeStream m_EventDataStream;

        private readonly NativeArray<PhysicsVelocity> m_InputVelocities;
        private readonly float m_TimeStep;

        internal CollisionEvents(NativeStream eventDataStream, NativeArray<PhysicsVelocity> inputVelocities, float timeStep)
        {
            m_EventDataStream = eventDataStream;
            m_InputVelocities = inputVelocities;
            m_TimeStep = timeStep;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventDataStream, m_InputVelocities, m_TimeStep);
        }

        public struct Enumerator /* : IEnumerator<CollisionEvent> */
        {
            private NativeStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;
            private CollisionEventDataRef m_Current;

            private readonly NativeArray<PhysicsVelocity> m_InputVelocities;
            private readonly float m_TimeStep;

            public CollisionEvent Current
            {
                get => m_Current.Value.CreateCollisionEvent(m_TimeStep, m_InputVelocities);
            }

            internal Enumerator(NativeStream stream, NativeArray<PhysicsVelocity> inputVelocities, float timeStep)
            {
                m_Reader = stream.IsCreated ? stream.AsReader() : new NativeStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;

                m_InputVelocities = inputVelocities;
                m_TimeStep = timeStep;

                unsafe
                {
                    m_Current = default;
                }

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    int currentSize = m_Reader.Read<int>();
                    AdvanceReader();

                    unsafe
                    {
                        m_Current = new CollisionEventDataRef((CollisionEventData*)(m_Reader.ReadUnsafePtr(currentSize)));
                    }

                    AdvanceReader();
                    return true;
                }
                return false;
            }

            private void AdvanceReader()
            {
                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
                {
                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }

    // An event raised when a pair of bodies have collided during solving.
    struct CollisionEventData
    {
        public PhysicsBody.IndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
        public EntityPair Entities;
        public float2 Normal;

        // The total impulse applied by the solver for this pair
        internal float SolverImpulse;

        // Number of narrow phase contact points
        internal int NumNarrowPhaseContactPoints;

        internal unsafe CollisionEvent CreateCollisionEvent(float timeStep, NativeArray<PhysicsVelocity> inputVelocities)
        {
            int bodyIndexA = BodyIndices.PhysicsBodyIndexA;
            int bodyIndexB = BodyIndices.PhysicsBodyIndexB;
            return new CollisionEvent
            {
                EventData = new CollisionEventDataRef((CollisionEventData*)(UnsafeUtility.AddressOf(ref this))),
                TimeStep = timeStep,
                InputVelocityA = bodyIndexA < inputVelocities.Length ? inputVelocities[bodyIndexA] : PhysicsVelocity.Zero,
                InputVelocityB = bodyIndexB < inputVelocities.Length ? inputVelocities[bodyIndexB] : PhysicsVelocity.Zero
            };
        }

        internal static int CalculateSize(int numContactPoints)
        {
            return UnsafeUtility.SizeOf<CollisionEventData>() + numContactPoints * UnsafeUtility.SizeOf<ContactPoint>();
        }

        internal unsafe ref ContactPoint AccessContactPoint(int pointIndex)
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<CollisionEventData>() + pointIndex * UnsafeUtility.SizeOf<ContactPoint>();
            return ref UnsafeUtility.AsRef<ContactPoint>(ptr);
        }

        // Calculate extra details about the collision, by re-integrating the leaf colliders to the time of collision
        internal unsafe CollisionEvent.Details CalculateDetails(
            ref PhysicsWorld physicsWorld, float timeStep, PhysicsVelocity inputVelocityA, PhysicsVelocity inputVelocityB, NativeArray<ContactPoint> narrowPhaseContactPoints)
        {
            int bodyIndexA = BodyIndices.PhysicsBodyIndexA;
            int bodyIndexB = BodyIndices.PhysicsBodyIndexB;
            bool bodyAIsDynamic = bodyIndexA < physicsWorld.BodyMotionVelocity.Length;
            bool bodyBIsDynamic = bodyIndexB < physicsWorld.BodyMotionVelocity.Length;

            PhysicsBody.MotionVelocity motionVelocityA = bodyAIsDynamic ? physicsWorld.BodyMotionVelocity[bodyIndexA] : PhysicsBody.MotionVelocity.Zero;
            PhysicsBody.MotionVelocity motionVelocityB = bodyBIsDynamic ? physicsWorld.BodyMotionVelocity[bodyIndexB] : PhysicsBody.MotionVelocity.Zero;
            PhysicsBody.MotionData motionDataA = bodyAIsDynamic ? physicsWorld.BodyMotionData[bodyIndexA] : PhysicsBody.MotionData.Zero;
            PhysicsBody.MotionData motionDataB = bodyBIsDynamic ? physicsWorld.BodyMotionData[bodyIndexB] : PhysicsBody.MotionData.Zero;

            float estimatedImpulse = SolverImpulse;

            // First calculate minimum time of impact and estimate the impulse
            float toi = timeStep;
            {
                float sumRemainingVelocities = 0.0f;
                float numRemainingVelocities = 0.0f;
                for (int i = 0; i < narrowPhaseContactPoints.Length; i++)
                {
                    var cp = narrowPhaseContactPoints[i];

                    //var worldA = motionDataA.WorldRigidTransform;
                    //var worldB = motionDataB.WorldRigidTransform;

                    // Collect data for impulse estimation
                    {
                        float2 pointVelA = GetPointVelocity(motionDataA.WorldPosition,
                            motionVelocityA.LinearVelocity, motionVelocityA.AngularVelocity, cp.Position + Normal * cp.Distance);
                        float2 pointVelB = GetPointVelocity(motionDataB.WorldPosition,
                            motionVelocityB.LinearVelocity, motionVelocityB.AngularVelocity, cp.Position);
                        float projRelVel = math.dot(pointVelB - pointVelA, Normal);
                        if (projRelVel > 0.0f)
                        {
                            sumRemainingVelocities += projRelVel;
                            numRemainingVelocities += 1.0f;
                        }
                    }

                    // Get minimum time of impact
                    {
                        float2 pointVelA = GetPointVelocity(motionDataA.WorldPosition,
                            inputVelocityA.LinearFloat, inputVelocityA.AngularFloat, cp.Position + Normal * cp.Distance);
                        float2 pointVelB = GetPointVelocity(motionDataB.WorldPosition,
                            inputVelocityB.LinearFloat, inputVelocityB.AngularFloat, cp.Position);
                        float projRelVel = math.dot(pointVelB - pointVelA, Normal);
                        if (projRelVel > 0.0f)
                        {
                            float newToi = math.max(0.0f, cp.Distance / projRelVel);
                            toi = math.min(toi, newToi);
                        }
                        else if (cp.Distance <= 0.0f)
                        {
                            // If in penetration, time of impact is 0 for sure
                            toi = 0.0f;
                        }
                    }
                }

                if (numRemainingVelocities > 0.0f)
                {
                    float sumInvMass = motionVelocityA.InverseMass + motionVelocityB.InverseMass;
                    estimatedImpulse += sumRemainingVelocities / (numRemainingVelocities * sumInvMass);
                }
            }

            // Then, sub-integrate for time of impact and keep contact points closer than hitDistanceThreshold
            int closestContactIndex = -1;
            float minDistance = float.MaxValue;
            {
                int estimatedContactPointCount = 0;
                for (int i = 0; i < narrowPhaseContactPoints.Length; i++)
                {
                    // Estimate new position
                    var cp = narrowPhaseContactPoints[i];
                    {
                        float2 pointVelA = GetPointVelocity(motionDataA.WorldPosition, inputVelocityA.LinearFloat, inputVelocityA.AngularFloat, cp.Position + Normal * cp.Distance);
                        float2 pointVelB = GetPointVelocity(motionDataB.WorldPosition, inputVelocityB.LinearFloat, inputVelocityB.AngularFloat, cp.Position);
                        float2 relVel = pointVelB - pointVelA;
                        float projRelVel = math.dot(relVel, Normal);

                        // Only sub integrate if approaching, otherwise leave it as is
                        // (it can happen that input velocity was separating but there
                        // still was a collision event - penetration recovery, or other
                        // body pushing in different direction).
                        if (projRelVel > 0.0f)
                        {
                            // Position the point on body A
                            cp.Position += Normal * cp.Distance;

                            // Sub integrate the point
                            cp.Position -= relVel * toi;

                            // Reduce the distance
                            cp.Distance -= projRelVel * toi;
                        }

                        // Filter out contacts that are still too far away
                        if (cp.Distance <= physicsWorld.CollisionWorld.CollisionTolerance)
                        {
                            narrowPhaseContactPoints[estimatedContactPointCount++] = cp;
                        }
                        else if (cp.Distance < minDistance)
                        {
                            minDistance = cp.Distance;
                            closestContactIndex = i;
                        }
                    }
                }

                // If due to estimation of relative velocity no contact points will
                // get closer than the tolerance, we need to export the closest one
                // to make sure at least one contact point is reported.
                if (estimatedContactPointCount == 0)
                {
                    narrowPhaseContactPoints[estimatedContactPointCount++] = narrowPhaseContactPoints[closestContactIndex];
                }

                // Instantiate collision details and allocate memory
                var details = new CollisionEvent.Details
                {
                    EstimatedContactPointPositions = new NativeArray<float2>(estimatedContactPointCount, Allocator.Temp),
                    EstimatedImpulse = estimatedImpulse
                };

                // Fill the contact point positions array
                for (int i = 0; i < estimatedContactPointCount; i++)
                {
                    details.EstimatedContactPointPositions[i] = narrowPhaseContactPoints[i].Position;
                }

                return details;
            }
        }

        private static float2 GetPointVelocity(float2 worldPos, float2 linVel, float angVel, float2 point)
        {
            /**
            Simplified from
            float3 angularVelocity = math.rotate(worldFromMotion, angVel);                      // will never change the angVel since in 2D
            float3 linearVelocity = math.cross(angularVelocity, point - worldFromMotion.pos);   // cross can be simplified since angularVelocity always vec3(0,0,angVel)
            return linVel + linearVelocity;

            to
            
            float3 b = math.float3(point, 0) - worldFromMotion.pos;
            float2 linearVelocity = math.float2(-angVel * b.y, angVel * b.x);
            return linVel + linearVelocity;

             */
            float2 b = point - worldPos;
            float2 linearVelocity = new float2(-angVel * b.y, angVel * b.x);
            return linVel + linearVelocity;
        }
    }

    // Wraps a pointer to CollisionEventData.
    // Used in enumerator for collision events.
    struct CollisionEventDataRef
    {
        private unsafe CollisionEventData* m_CollisionEventData;

        public unsafe CollisionEventDataRef(CollisionEventData* collisionEventData)
        {
            m_CollisionEventData = collisionEventData;
        }

        public unsafe ref CollisionEventData Value => ref UnsafeUtility.AsRef<CollisionEventData>(m_CollisionEventData);
    }
}