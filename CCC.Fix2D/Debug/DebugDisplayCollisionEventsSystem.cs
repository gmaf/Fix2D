using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace CCC.Fix2D.Debugging
{
    // A system which draws any collision events produced by the physics step system
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorldSystem)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DebugDisplayCollisionEventsSystem : SystemBase
    {
        PhysicsWorldSystem m_BuildPhysicsWorldSystem;
        StepPhysicsWorldSystem m_StepPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        PhysicsDebugStreamSystem m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorldSystem>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<PhysicsDebugStreamSystem>();

            RequireSingletonForUpdate<PhysicsDebugDisplay>();
        }

        protected override void OnUpdate()
        {
            var settings = GetSingleton<PhysicsDebugDisplay>();
            if (!settings.DrawCollisions)
                return;

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                PhysicsDebugStreamSystem.Context* sharedOutput = (PhysicsDebugStreamSystem.Context*)UnsafeUtility.Malloc(sizeof(PhysicsDebugStreamSystem.Context), 16, Allocator.TempJob);
                *sharedOutput = m_DebugStreamSystem.GetContext(1);
                sharedOutput->Begin(0);

                // This will call the extension method defined in Unity.Physics
                JobHandle handle = new DisplayCollisionEventsJob
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput,
                    Color = (Vector4)settings.CollisionsColor
                }.Schedule(m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, Dependency);

                JobHandle finishHandle = new FinishDisplayCollisionEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);

                m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

                Dependency = handle;
            }
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        //[BurstCompile] // cannot burst because of text display
        private unsafe struct DisplayCollisionEventsJob : ICollisionEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            public PhysicsDebugStreamSystem.Context* OutputStreamContext;
            public Color Color;

            public unsafe void Execute(CollisionEvent e)
            {
                CollisionEvent.Details details = e.CalculateDetails(ref World);
                const float WIDTH = 0.2f;

                float2 point = default;
                
                for (int i = 0; i < details.EstimatedContactPointPositions.Length; i++)
                {
                    if(i > 0)
                    {
                        OutputStreamContext->Line(point, details.EstimatedContactPointPositions[i], Color);
                    }

                    point = details.EstimatedContactPointPositions[i];

                    OutputStreamContext->Point(point, WIDTH / 4, Color);
                    OutputStreamContext->Circle(point, WIDTH / 2, Color);
                }

                point = details.AverageContactPointPosition;

                float displayedImpulse = math.min(math.remap(0, 20, 1.5f, 10f, details.EstimatedImpulse), 10);
                OutputStreamContext->Line(point, point + (e.Normal * WIDTH * displayedImpulse), Color);
                OutputStreamContext->Text(details.EstimatedImpulse.ToString("0.00").ToCharArray(), point + new float2(WIDTH, WIDTH), Color);
            }
        }

        [BurstCompile]
        private unsafe struct FinishDisplayCollisionEventsJob : IJob
        {
            [NativeDisableUnsafePtrRestriction]
            internal PhysicsDebugStreamSystem.Context* OutputStreamContext;

            public void Execute()
            {
                OutputStreamContext->End();
                UnsafeUtility.Free(OutputStreamContext, Allocator.TempJob);
            }
        }
    }
}