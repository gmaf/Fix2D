using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace CCC.Fix2D
{
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorldSystem))]
    [AlwaysUpdateSystem]
    public class PhysicsWorldSystem : SystemBase
    {
        public PhysicsWorld PhysicsWorld;

        public JobHandle FinalJobHandle { get; private set; }

        public EntityQuery StaticEntityGroup { get; private set; }
        public EntityQuery DynamicEntityGroup { get; private set; }

        // A look-up from an Entity to a Physics Body Index.
        public struct EntityToPhysicsBodyIndex : IDisposable
        {
            public NativeHashMap<Entity, int> Lookup;

            // Reset the look-up and increase its capacity if required.
            // NOTE: We'll create one if it's not around.
            internal void Reset(int totalBodyCount)
            {
                totalBodyCount = math.max(1, totalBodyCount);

                if (Lookup.IsCreated)
                {
                    Lookup.Clear();
                    if (Lookup.Capacity < totalBodyCount)
                    {
                        Lookup.Capacity = totalBodyCount;
                    }
                    return;
                }

                Lookup = new NativeHashMap<Entity, int>(totalBodyCount, Allocator.Persistent);
            }

            public void Dispose()
            {
                if (Lookup.IsCreated)
                    Lookup.Dispose();
            }
        }
        public EntityToPhysicsBodyIndex EntityToPhysicsBody => m_EntityToPhysicsBody;
        EntityToPhysicsBodyIndex m_EntityToPhysicsBody;

        internal PhysicsCallbacks Callbacks = new PhysicsCallbacks();

        EndFramePhysicsSystem m_EndFramePhysicsSystem;

        // Schedule a callback to run at the selected phase.
        public void ScheduleCallback(PhysicsCallbacks.Phase phase, PhysicsCallbacks.Callback callback, JobHandle dependency = default(JobHandle))
        {
            Callbacks.Enqueue(phase, callback, dependency);
        }

        // Get the PhysicBodyIndex via an Entity lookup.
        public int GetPhysicsBodyIndex(Entity entity)
        {
            if (entity != Entity.Null &&
                m_EntityToPhysicsBody.Lookup.IsCreated &&
                m_EntityToPhysicsBody.Lookup.TryGetValue(entity, out var physicsBodyIndex))
                return physicsBodyIndex;

            return PhysicsBody.Constants.InvalidBodyIndex;
        }

        // Get the PhysicBody via an Entity lookup.
        public PhysicsBody GetPhysicsBody(Entity entity)
        {
            var physicsBodyIndex = GetPhysicsBodyIndex(entity);
            if (physicsBodyIndex != PhysicsBody.Constants.InvalidBodyIndex)
            {
                var physicsBody = PhysicsWorld.AllBodies[physicsBodyIndex];
                SafetyChecks.IsTrue(entity == physicsBody.Entity);
                return physicsBody;
            }

            return default;
        }

        protected override void OnCreate()
        {
            base.OnCreate();
            
            // Definition of a static body entity.
            StaticEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new []
                {
                    ComponentType.ReadOnly<PhysicsColliderBlob>()
                },
                Any = new []
                {
                    ComponentType.ReadOnly<FixTranslation>(),
                    ComponentType.ReadOnly<FixRotation>()
                },
                None = new []
                {
                    ComponentType.ReadOnly<PhysicsVelocity>()
                }
            });

            // Definition of a dynamic body entity.
            DynamicEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new []
                {                    
                    ComponentType.ReadOnly<PhysicsVelocity>(),
                    ComponentType.ReadOnly<FixTranslation>(),
                    ComponentType.ReadOnly<FixRotation>()
                }
            });

            FinalJobHandle = default;

            PhysicsWorld = new PhysicsWorld(
                staticBodyCount: 0,
                dynamicBodyCount: 0,
                jointCount: 0
            );

            // Create the Entity to Physics Body Lookup.
            m_EntityToPhysicsBody = new EntityToPhysicsBodyIndex();
            m_EntityToPhysicsBody.Reset(0);

            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
       }

        protected override void OnDestroy()
        {
            PhysicsWorld.Dispose();
            m_EntityToPhysicsBody.Dispose();
            
            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            // Make sure last frame's physics jobs are complete
            m_EndFramePhysicsSystem.FinalJobHandle.Complete();

            // Update the physics world settings we have a component assigned.
            if (HasSingleton<PhysicsStepSettings>())
            {
                PhysicsWorld.StepSettings = GetSingleton<PhysicsStepSettings>();
                PhysicsWorld.StepSettings.Validate();
            }

            // Schedule phase callback.
            var inputDeps = Callbacks.ScheduleCallbacksForPhase(PhysicsCallbacks.Phase.PreBuild, ref PhysicsWorld, Dependency);

            var entityType = GetEntityTypeHandle();

            var parentType = GetComponentTypeHandle<Parent>(true);
            var translationType = GetComponentTypeHandle<FixTranslation>(true);
            var rotationType = GetComponentTypeHandle<FixRotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsColliderBlob>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);
            var physicsMassType = GetComponentTypeHandle<PhysicsMass>(true);
            var physicsDampingType = GetComponentTypeHandle<PhysicsDamping>(true);
            var physicsGravityType = GetComponentTypeHandle<PhysicsGravity>(true);

            var staticBodyCount = StaticEntityGroup.CalculateEntityCount();
            var dynamicBodyCount = DynamicEntityGroup.CalculateEntityCount();

            var previousStaticBodyCount = PhysicsWorld.StaticBodyCount;

            // Ensure we have adequate world simulation capacity for the bodies.
            // NOTE: Add an extra static "ground" body used by joints with no entity reference.
            PhysicsWorld.Reset(
                staticBodyCount: staticBodyCount + 1,
                dynamicBodyCount: dynamicBodyCount,
                jointCount: 0
            );

            // Reset the Entity to Body look-up.
            // NOTE: We don't need to add the static "ground" body here.
            m_EntityToPhysicsBody.Reset(dynamicBodyCount + staticBodyCount);

            // Determine if the static bodies have changed in any way that will require the static broadphase tree to be rebuilt.
            JobHandle staticBodiesCheckHandle = default;
            var haveStaticBodiesChanged = new NativeArray<int>(1, Allocator.TempJob) { [0] = 0 };
            {
                if (PhysicsWorld.StaticBodyCount != previousStaticBodyCount)
                {
                    haveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    staticBodiesCheckHandle = new CheckStaticBodyChangesJob
                    {
                        PhysicsColliderType = physicsColliderType,
                        TranslationType =  translationType,
                        RotationType = rotationType,
                        LastSystemVersion = LastSystemVersion,

                        Result = haveStaticBodiesChanged

                    }.ScheduleParallel(StaticEntityGroup, batchesPerChunk: 1, inputDeps);
                }
            }

            using (var jobHandles = new NativeList<JobHandle>(5, Allocator.Temp))
            {
                // Static body changes check jobs
                jobHandles.Add(staticBodiesCheckHandle);

                // Create the static "ground" body used by joints with no entity reference.
                // NOTE: This will always exist as the last body in the world simulation. Could skip this if no joints present
                jobHandles.Add(new CreateStaticGroundBody
                {
                    GroundBodyIndex = PhysicsWorld.GroundBodyIndex,
                    PhysicsBodies = PhysicsWorld.AllBodies,

                }.Schedule(inputDeps));

                // Create dynamic bodies.
                if (dynamicBodyCount > 0)
                {
                    jobHandles.Add(
                        new CreatePhysicsBodiesJob
                        {
                            EntityType = entityType,
                            ParentType = parentType,
                            TranslationType = translationType,
                            RotationType = rotationType,
                            ColliderType = physicsColliderType,

                            PhysicsBodies = PhysicsWorld.DynamicBodies,

                        }.ScheduleParallel(DynamicEntityGroup, batchesPerChunk: 1, inputDeps));

                    jobHandles.Add(
                        new CreatePhysicsBodyMotionsJob
                        {
                            TranslationType = translationType,
                            RotationType = rotationType,
                            ColliderType = physicsColliderType,
                            PhysicsVelocityType = physicsVelocityType,
                            PhysicsMassType = physicsMassType,
                            PhysicsDampingType = physicsDampingType,
                            PhysicsGravityType = physicsGravityType,

                            BodyMotionData = PhysicsWorld.BodyMotionData,
                            BodyMotionVelocity = PhysicsWorld.BodyMotionVelocity

                        }.ScheduleParallel(DynamicEntityGroup, batchesPerChunk: 1, inputDeps));
                }

                // Create static bodies.
                if (staticBodyCount > 0)
                {
                    jobHandles.Add(
                        new CreatePhysicsBodiesJob
                        {
                            EntityType = entityType,
                            ParentType = parentType,
                            TranslationType = translationType,
                            RotationType = rotationType,
                            ColliderType = physicsColliderType,

                            PhysicsBodies = PhysicsWorld.StaticBodies,

                        }.ScheduleParallel(StaticEntityGroup, batchesPerChunk: 1, inputDeps));
                }

                // Combine all scheduled jobs.
                var handle = JobHandle.CombineDependencies(jobHandles);
                jobHandles.Clear();

                // Build the Entity to PhysicsBody Look-ups.
                var totalBodyCount = staticBodyCount + dynamicBodyCount;
                if (totalBodyCount > 0)
                {
                    handle = new CreateEntityToPhysicsBodyLookupsJob
                    {
                        PhysicsBodies = PhysicsWorld.AllBodies,
                        IndexLookup = m_EntityToPhysicsBody.Lookup.AsParallelWriter()

                    }.Schedule(totalBodyCount, 128, handle);
                }

                // Build the broadphase.
                handle = PhysicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(
                    ref PhysicsWorld,
                    haveStaticBodiesChanged,
                    handle);

                FinalJobHandle = haveStaticBodiesChanged.Dispose(handle);
            }

            Dependency = JobHandle.CombineDependencies(FinalJobHandle, inputDeps);
        }

        #region Jobs

        [BurstCompile]
        struct CheckStaticBodyChangesJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<FixTranslation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<FixRotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsColliderBlob> PhysicsColliderType;
            public uint LastSystemVersion;
            
            [NativeDisableParallelForRestriction]
            public NativeArray<int> Result;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                var didBatchChange =
                    batchInChunk.DidChange(TranslationType, LastSystemVersion) ||
                    batchInChunk.DidChange(RotationType, LastSystemVersion) ||
                    batchInChunk.DidChange(PhysicsColliderType, LastSystemVersion) ||
                    batchInChunk.DidOrderChange(LastSystemVersion);

                if (didBatchChange)
                {
                    // Note that multiple worker threads may be running at the same time.
                    // They either write 1 to Result[0] or not write at all.  In case multiple
                    // threads are writing 1 to this variable, in C#, reads or writes of int
                    // data type are atomic, which guarantees that Result[0] is 1.
                    Result[0] = 1;
                }
            }
        }

        [BurstCompile]
        struct CreateStaticGroundBody : IJob
        {
            [ReadOnly] public int GroundBodyIndex;

            [NativeDisableContainerSafetyRestriction]
            public NativeArray<PhysicsBody> PhysicsBodies;

            public void Execute()
            {
                PhysicsBodies[GroundBodyIndex] = PhysicsBody.Zero;
            }
        }

        [BurstCompile]
        struct CreatePhysicsBodiesJob : IJobEntityBatchWithIndex
        {
            [ReadOnly] public EntityTypeHandle EntityType;
            [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
            [ReadOnly] public ComponentTypeHandle<FixTranslation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<FixRotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsColliderBlob> ColliderType;

            [NativeDisableContainerSafetyRestriction]
            public NativeArray<PhysicsBody> PhysicsBodies;

            //public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int indexOfFirstEntityInQuery)
            {
                NativeArray<Entity> entities = batchInChunk.GetNativeArray(EntityType);
                //var localToWorlds = batchInChunk.GetNativeArray(LocalToWorldType);
                NativeArray<FixTranslation> translations = batchInChunk.GetNativeArray(TranslationType);
                NativeArray<FixRotation> rotations = batchInChunk.GetNativeArray(RotationType);
                NativeArray<PhysicsColliderBlob> colliders = batchInChunk.GetNativeArray(ColliderType);

                bool hasParentType = batchInChunk.Has(ParentType);
                bool hasLocalToWorldType = false;
                bool hasTranslationType = batchInChunk.Has(TranslationType);
                bool hasRotationType = batchInChunk.Has(RotationType);
                bool hasColliderType = batchInChunk.Has(ColliderType);

                PhysicsTransform worldTransform = PhysicsTransform.Identity;

                int instanceCount = batchInChunk.Count;
                for(int i = 0, physicsBodyIndex = indexOfFirstEntityInQuery; i < instanceCount; ++i, ++physicsBodyIndex)
                {
                    if (hasParentType)
                    {
                        if (hasLocalToWorldType)
                        {
                            //var localToWorld = localToWorlds[i];
                            //var matrix = localToWorld.Value;
                            //var orientation = quaternion.LookRotationSafe(matrix.c2.xyz, matrix.c1.xyz);
                            //worldTransform = new PhysicsTransform(localToWorld.Position, orientation);
                        }
                    }
                    else
                    {
                        if (hasTranslationType)
                        {
                            worldTransform.Translation = new float2((float)translations[i].Value.x, (float)translations[i].Value.y);
                        }
                        //else if (hasLocalToWorldType)
                        //{
                        //    worldTransform.Translation = localToWorlds[i].Position.xy;
                        //}

                        if (hasRotationType)
                        {
                            worldTransform.SetAngleRotation((float)rotations[i].Value);
                        }
                        else if (hasLocalToWorldType)
                        {
                            //var localToWorld = localToWorlds[i];
                            //var matrix = localToWorld.Value;
                            //worldTransform.SetQuaternionRotation(quaternion.LookRotationSafe(matrix.c2.xyz, matrix.c1.xyz));
                        }
                    }

                    var entity = entities[i];

                    PhysicsBodies[physicsBodyIndex] = new PhysicsBody
                    {
                        Collider = hasColliderType ? colliders[i].Collider : default,
                        WorldTransform = worldTransform,
                        Entity = entity
                    };
                }
            }
        }

        [BurstCompile]
        struct CreatePhysicsBodyMotionsJob : IJobEntityBatchWithIndex
        {
            [ReadOnly] public ComponentTypeHandle<FixTranslation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<FixRotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsColliderBlob> ColliderType;
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
            [ReadOnly] public ComponentTypeHandle<PhysicsDamping> PhysicsDampingType;
            [ReadOnly] public ComponentTypeHandle<PhysicsGravity> PhysicsGravityType;

            [NativeDisableParallelForRestriction] public NativeArray<PhysicsBody.MotionData> BodyMotionData;
            [NativeDisableParallelForRestriction] public NativeArray<PhysicsBody.MotionVelocity> BodyMotionVelocity;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int indexOfFirstEntityInQuery)
            {
                var translations = batchInChunk.GetNativeArray(TranslationType);
                var rotations = batchInChunk.GetNativeArray(RotationType);
                var colliders = batchInChunk.GetNativeArray(ColliderType);
                var velocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                var masses = batchInChunk.GetNativeArray(PhysicsMassType);
                var dampings = batchInChunk.GetNativeArray(PhysicsDampingType);
                var gravities = batchInChunk.GetNativeArray(PhysicsGravityType);

                var hasColliderType = batchInChunk.Has(ColliderType);
                var hasMassType = batchInChunk.Has(PhysicsMassType);
                var hasDampingType = batchInChunk.Has(PhysicsDampingType);
                var hasGravitiesType = batchInChunk.Has(PhysicsGravityType);

                for(int i = 0, physicsBodyIndex = indexOfFirstEntityInQuery; i < batchInChunk.Count; ++i, ++physicsBodyIndex)
                {                    
                    var worldTransform = new PhysicsTransform(translations[i].Value, rotations[i].Value);
                    var velocity = velocities[i];
                    var damping = hasDampingType ? dampings[i] : default;
                    var mass = hasMassType ? masses[i] : default;
                    
                    var gravityScale = 1f;
                    if (hasGravitiesType)
                    {
                        gravityScale = gravities[i].Scale;
                    }
                    else
                    {
                        // If we've got infinite mass then no gravity should be applied.
                        if (mass.InverseMass < float.Epsilon)
                            gravityScale = 0f;
                    }

                    var angularExpansionFactor = 0f;
                    if (hasColliderType)
                    {
                        var colliderBlob = colliders[i].Collider;
                        if (colliderBlob.IsCreated)
                            angularExpansionFactor = colliderBlob.Value.MassProperties.AngularExpansionFactor;
                    }

                    BodyMotionData[physicsBodyIndex] = new PhysicsBody.MotionData
                    {                       
                        WorldPosition = worldTransform.Translation,
                        WorldAngle = PhysicsMath.angle(worldTransform.Rotation),

                        LocalCenterOfMass = mass.LocalCenterOfMass,

                        LinearDamping = damping.Linear,
                        AngularDamping = damping.Angular
                    };

                    BodyMotionVelocity[physicsBodyIndex] = new PhysicsBody.MotionVelocity
                    {
                        LinearVelocity = velocity.Linear,
                        AngularVelocity = velocity.Angular,
                        GravityFactor = gravityScale,

                        InverseMass = mass.InverseMass,
                        InverseInertia = mass.InverseInertia,

                        AngularExpansionFactor = angularExpansionFactor
                    };
                }
            }
        }

        [BurstCompile]
        struct CreateEntityToPhysicsBodyLookupsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<PhysicsBody> PhysicsBodies;
            public NativeHashMap<Entity, int>.ParallelWriter IndexLookup;

            public void Execute(int index)
            {
                // Add the Entity to the lookup.
                IndexLookup.TryAdd(PhysicsBodies[index].Entity, index);
            }
        }

        #endregion
    }
}
