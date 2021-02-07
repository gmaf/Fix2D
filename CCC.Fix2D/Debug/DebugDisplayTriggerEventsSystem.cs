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
    public class DebugDisplayTriggerEventsSystem : SystemBase
    {
        PhysicsWorldSystem _buildPhysicsWorldSystem;
        StepPhysicsWorldSystem _stepPhysicsWorldSystem;
        EndFramePhysicsSystem _endFramePhysicsSystem;
        PhysicsDebugStreamSystem _debugStreamSystem;

        protected override void OnCreate()
        {
            _buildPhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            _stepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorldSystem>();
            _endFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            _debugStreamSystem = World.GetOrCreateSystem<PhysicsDebugStreamSystem>();

            RequireSingletonForUpdate<PhysicsDebugDisplay>();
        }

        protected override void OnUpdate()
        {
            var settings = GetSingleton<PhysicsDebugDisplay>();
            if (!settings.DrawTriggers)
                return;

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                PhysicsDebugStreamSystem.Context* sharedOutput = (PhysicsDebugStreamSystem.Context*)UnsafeUtility.Malloc(sizeof(PhysicsDebugStreamSystem.Context), 16, Allocator.TempJob);
                *sharedOutput = _debugStreamSystem.GetContext(1);
                sharedOutput->Begin(0);

                // This will call the extension method defined in Unity.Physics
                JobHandle handle = new DisplayTriggerEventsJob
                {
                    World = _buildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput,
                    Color = (Vector4)settings.TriggersColor
                }.Schedule(_stepPhysicsWorldSystem.Simulation, ref _buildPhysicsWorldSystem.PhysicsWorld, Dependency);

                JobHandle finishHandle = new FinishDisplayCollisionEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);

                _endFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

                Dependency = handle;
            }
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        [BurstCompile]
        private unsafe struct DisplayTriggerEventsJob : ITriggerEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            public PhysicsDebugStreamSystem.Context* OutputStreamContext;
            public Color Color;

            public unsafe void Execute(TriggerEvent e)
            {
                var p1 = World.AllBodies[e.BodyIndexA].WorldTransform.Translation;
                var p2 = World.AllBodies[e.BodyIndexB].WorldTransform.Translation;
                OutputStreamContext->Line(p1, p2, Color);
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