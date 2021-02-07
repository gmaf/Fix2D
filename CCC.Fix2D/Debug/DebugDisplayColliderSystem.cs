using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Assertions;

namespace CCC.Fix2D.Debugging
{
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsDebugStreamSystem))]
    [UpdateBefore(typeof(PhysicsWorldSystem))]
    public class DebugDisplayColliderSystem : SystemBase
    {
        PhysicsWorldSystem m_PhysicsWorldSystem;
        PhysicsDebugStreamSystem m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_PhysicsWorldSystem = World.GetOrCreateSystem<PhysicsWorldSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<PhysicsDebugStreamSystem>();

            RequireSingletonForUpdate<PhysicsDebugDisplay>();
        }

        protected override void OnUpdate()
        {
            if (m_PhysicsWorldSystem.PhysicsWorld.BodyCount == 0)
                return;

            var debugDisplay = GetSingleton<PhysicsDebugDisplay>();

            if (debugDisplay.DrawStaticColliders)
            {
                JobHandle callback(ref PhysicsWorld world, JobHandle deps)
                {
                    return new DisplayColliderJob
                    {
                        OutputStream = m_DebugStreamSystem.GetContext(1),
                        ColliderColor = (Vector4)debugDisplay.StaticColliderColor,
                        PhysicsBodies = m_PhysicsWorldSystem.PhysicsWorld.StaticBodies,
                        Translations = GetComponentDataFromEntity<FixTranslation>(isReadOnly: true),
                        Rotations = GetComponentDataFromEntity<FixRotation>(isReadOnly: true),
                    }.Schedule(deps);
                }

                m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PostExport, callback);
            }

            if (debugDisplay.DrawDynamicColliders)
            {
                JobHandle callback(ref PhysicsWorld world, JobHandle deps)
                {
                    return new DisplayColliderJob
                    {
                        OutputStream = m_DebugStreamSystem.GetContext(1),
                        ColliderColor = (Vector4)debugDisplay.DynamicColliderColor,
                        PhysicsBodies = m_PhysicsWorldSystem.PhysicsWorld.DynamicBodies,
                        Translations = GetComponentDataFromEntity<FixTranslation>(isReadOnly: true),
                        Rotations = GetComponentDataFromEntity<FixRotation>(isReadOnly: true),
                    }.Schedule(deps);
                }

                m_PhysicsWorldSystem.ScheduleCallback(PhysicsCallbacks.Phase.PostExport, callback);
            }
        }

        // Job to iterate over all the bodies in a scene, for any
        // which have a collider, fetch the geometry and
        // write it to a debug stream.
        [BurstCompile]
        private unsafe struct DisplayColliderJob : IJob
        {
            public PhysicsDebugStreamSystem.Context OutputStream;
            public Color ColliderColor;

            [ReadOnly] public NativeSlice<Fix2D.PhysicsBody> PhysicsBodies;
            [ReadOnly] public ComponentDataFromEntity<FixTranslation> Translations;
            [ReadOnly] public ComponentDataFromEntity<FixRotation> Rotations;

            public void Execute()
            {
                OutputStream.Begin(0);

                for (var i = 0; i < PhysicsBodies.Length; ++i)
                {
                    var physicsBody = PhysicsBodies[i];
                    var colliderBlob = physicsBody.Collider;

                    if (colliderBlob.IsCreated)
                    {
                        var worldTransform = new PhysicsTransform(Translations[physicsBody.Entity].Value, Rotations[physicsBody.Entity].Value);
                        DrawConvex(colliderBlob.GetColliderPtr(), ref worldTransform, ref OutputStream, ColliderColor);

                    }
                }

                OutputStream.End();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void DrawConvex(
            Collider* collider,
            ref PhysicsTransform worldTransform,
            ref PhysicsDebugStreamSystem.Context outputStream,
            Color colliderColor)
        {
            var vertexCount = collider->VertexCount;
            var vertices = collider->Vertices;
            var convexRadius = collider->m_ConvexHull.ConvexRadius;

            switch (collider->ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                {
                    Assert.IsTrue(vertexCount >= 3, "ConvexCollider must have >= 3 vertices.");

                    outputStream.Polygon(vertices, vertexCount, worldTransform, colliderColor);
                    return;
                }

                case ColliderType.Circle:
                {
                    Assert.AreEqual(1, vertexCount, "CircleCollider must have 1 vertex.");

                    var position = PhysicsMath.mul(worldTransform, vertices[0]);
                    outputStream.Circle(position, convexRadius, colliderColor);
                    return;
                }

                case ColliderType.Capsule:
                {
                    Assert.AreEqual(2, vertexCount, "CapsuleCollider must have 2 vertices.");

                    var vertex0 = PhysicsMath.mul(worldTransform, vertices[0]);
                    var vertex1 = PhysicsMath.mul(worldTransform, vertices[1]);
                    var offset = PhysicsMath.perp(math.normalizesafe(vertex1 - vertex0)) * new float2(convexRadius);

                    // Side Edges.
                    {
                        outputStream.Line(vertex0 - offset, vertex1 - offset, colliderColor);
                        outputStream.Line(vertex0 + offset, vertex1 + offset, colliderColor);
                    }

                    // End Caps.
                    {
                        var startAngle = math.atan2(offset.y, offset.x);
                        var endAngle = startAngle + math.PI;
                        outputStream.Arc(vertex0, convexRadius, startAngle, endAngle, colliderColor);
                        outputStream.Arc(vertex1, convexRadius, startAngle + math.PI, endAngle + math.PI, colliderColor);
                    }

                    return;
                }

                default:
                    return;
            }
        }
    }
}