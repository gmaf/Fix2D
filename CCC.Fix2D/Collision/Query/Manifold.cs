using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities.UniversalDelegates;
using Unity.Mathematics;
using UnityEngine;
using static CCC.Fix2D.PhysicsMath;

namespace CCC.Fix2D
{
    // A header preceding a number of contact points in a stream.
    struct ContactHeader
    {
        public PhysicsBody.IndexPair BodyPair;
        public PhysicsBody.CustomTagsPair BodyCustomTags;
        public JacobianFlags JacobianFlags;
        public int NumContacts;
        public float2 Normal;
        public float CoefficientOfFriction;
        public float CoefficientOfRestitution;
        public ColliderKeyPair ColliderKeys;

        // followed by NumContacts * ContactPoint
    }

    // A contact point in a manifold. All contacts share the same normal.
    public struct ContactPoint
    {
        public float2 Position; // world space position on object B
        public float Distance;  // separating distance along the manifold normal
    }

    // Contact manifold stream generation functions
    static class ManifoldQueries
    {
        // A context passed through the manifold generation functions
        internal unsafe struct Context
        {
            public PhysicsBody.IndexPair BodyIndices;
            public PhysicsBody.CustomTagsPair BodyCustomTags;
            public bool BodiesHaveInfiniteMass;
            public NativeStream.Writer* ContactWriter;  // cannot be passed by value
        }

        // Write a set of contact manifolds for a pair of bodies to the given stream.
        public static unsafe void BodyBody(in PhysicsBody rigidBodyA, in PhysicsBody rigidBodyB, in PhysicsBody.MotionVelocity motionVelocityA, in PhysicsBody.MotionVelocity motionVelocityB,
            float collisionTolerance, float timeStep, PhysicsBody.IndexPair pair, ref NativeStream.Writer contactWriter)
        {
            var colliderA = (Collider*)rigidBodyA.Collider.GetUnsafePtr();
            var colliderB = (Collider*)rigidBodyB.Collider.GetUnsafePtr();

            if (colliderA == null || colliderB == null || !CollisionFilter.IsCollisionEnabled(colliderA->Filter, colliderB->Filter))
            {
                return;
            }

            // Build combined motion expansion
            MotionExpansion expansion;
            {
                MotionExpansion expansionA = motionVelocityA.CalculateExpansion(timeStep);
                MotionExpansion expansionB = motionVelocityB.CalculateExpansion(timeStep);
                expansion = new MotionExpansion
                {
                    Linear = expansionA.Linear - expansionB.Linear,
                    Uniform = expansionA.Uniform + expansionB.Uniform + collisionTolerance
                };
            }

            var context = new Context
            {
                BodyIndices = pair,
                BodyCustomTags = new PhysicsBody.CustomTagsPair { CustomTagsA = rigidBodyA.CustomTags, CustomTagsB = rigidBodyB.CustomTags },
                BodiesHaveInfiniteMass = motionVelocityA.HasInfiniteInertiaAndMass && motionVelocityB.HasInfiniteInertiaAndMass,
                ContactWriter = (NativeStream.Writer*)UnsafeUtility.AddressOf(ref contactWriter)
            };

            
            ConvexConvex(context, ColliderKeyPair.Empty, colliderA, colliderB, rigidBodyA.WorldTransform, rigidBodyB.WorldTransform, expansion.MaxDistance, false);
        }

        private static unsafe void WriteManifold(ConvexConvexManifoldQueries.Manifold manifold, Context context, ColliderKeyPair colliderKeys,
            PhysicsMaterial materialA, PhysicsMaterial materialB, bool flipped)
        {
            // Write results to stream
            if (manifold.NumContacts > 0)
            {
                if (flipped)
                {
                    manifold.Flip();
                }

                var header = new ContactHeader
                {
                    BodyPair = context.BodyIndices,
                    BodyCustomTags = context.BodyCustomTags,
                    NumContacts = manifold.NumContacts,
                    Normal = manifold.Normal,
                    ColliderKeys = colliderKeys
                };

                // Apply materials
                {
                    // Combined collision response of the two
                    CollisionResponsePolicy combinedCollisionResponse = PhysicsMaterial.GetCombinedCollisionResponse(materialA, materialB);
                    Unity.Assertions.Assert.IsFalse(combinedCollisionResponse == CollisionResponsePolicy.None,
                        "DisableCollisions pairs should have been filtered out earlier!");

                    if (combinedCollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents)
                    {
                        header.JacobianFlags |= JacobianFlags.IsTrigger;
                    }
                    else
                    {
                        if (combinedCollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableCollisionEvents;
                        }

                        PhysicsMaterial.MaterialFlags combinedFlags = materialA.Flags | materialB.Flags;
                        if ((combinedFlags & PhysicsMaterial.MaterialFlags.EnableMassFactors) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableMassFactors;
                        }
                        if ((combinedFlags & PhysicsMaterial.MaterialFlags.EnableSurfaceVelocity) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableSurfaceVelocity;
                        }

                        header.CoefficientOfFriction = PhysicsMaterial.GetCombinedFriction(materialA, materialB);
                        header.CoefficientOfRestitution = PhysicsMaterial.GetCombinedRestitution(materialA, materialB);
                    }
                }

                //string s = $"Manifold: n:{manifold.Normal}";

                context.ContactWriter->Write(header);

                // Group the contact points in 2s (when 4-6 contact points) and 3s (6 or more contact points)
                // to avoid the order forcing the magnitude of the impulse on one side of the face.
                // When less than 4 contact points access them in order.
                int startIndex = 0;
                int increment = header.NumContacts < 6 ?
                    math.max(header.NumContacts / 2, 1) : (header.NumContacts / 3 + ((header.NumContacts % 3 > 0) ? 1 : 0));
                for (int contactIndex = 0; ; contactIndex += increment)
                {
                    if (contactIndex >= header.NumContacts)
                    {
                        startIndex++;
                        if (startIndex == increment)
                        {
                            break;
                        }
                        contactIndex = startIndex;
                    }

                    //s += $"     p({manifold[contactIndex].Position}, depth{manifold[contactIndex].Distance})";

                    context.ContactWriter->Write(manifold[contactIndex]);
                }

                //Debug.LogWarning(s);
            }
        }

        private static unsafe void ConvexConvex(
            Context context, ColliderKeyPair colliderKeys,
            Collider* convexColliderA, Collider* convexColliderB, [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform worldFromB,
            float maxDistance, bool flipped)
        {
            PhysicsMaterial materialA = ((ConvexColliderHeader*)convexColliderA)->Material;
            PhysicsMaterial materialB = ((ConvexColliderHeader*)convexColliderB)->Material;

            CollisionResponsePolicy combinedCollisionResponse = PhysicsMaterial.GetCombinedCollisionResponse(materialA, materialB);

            // Skip the shapes if any of them is marked with a "None" collision response
            if (combinedCollisionResponse == CollisionResponsePolicy.None)
            {
                return;
            }

            // Skip if the bodies have infinite mass and the materials don't want to raise any solver events,
            // since the resulting contacts can't have any effect during solving
            if (context.BodiesHaveInfiniteMass)
            {
                if (combinedCollisionResponse != CollisionResponsePolicy.RaiseTriggerEvents &&
                    combinedCollisionResponse != CollisionResponsePolicy.CollideRaiseCollisionEvents)
                {
                    return;
                }
            }

            PhysicsTransform aFromB = mul(inverse(worldFromA), worldFromB);

            ConvexConvexManifoldQueries.Manifold contactManifold;
            switch (convexColliderA->ColliderType)
            {
                case ColliderType.Circle:
                    switch (convexColliderB->ColliderType)
                    {
                        case ColliderType.Circle:
                            ConvexConvexManifoldQueries.SphereSphere(
                                convexColliderA, convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                convexColliderB, convexColliderA,
                                worldFromB, inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxSphere(
                                convexColliderB, convexColliderA,
                                worldFromB, inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Polygon:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref convexColliderA->m_ConvexHull, ref convexColliderB->m_ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Box:
                    switch (convexColliderB->ColliderType)
                    {
                        case ColliderType.Circle:
                            ConvexConvexManifoldQueries.BoxSphere(
                                convexColliderA, convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxBox(
                                convexColliderA, convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                        case ColliderType.Polygon:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref convexColliderA->m_ConvexHull, ref convexColliderB->m_ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Capsule:
                    switch (convexColliderB->ColliderType)
                    {
                        case ColliderType.Circle:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                convexColliderA, convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleCapsule(
                                convexColliderA, convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Box:
                        case ColliderType.Polygon:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref convexColliderA->m_ConvexHull, ref convexColliderB->m_ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Polygon:
                    ConvexConvexManifoldQueries.ConvexConvex(
                        ref convexColliderA->m_ConvexHull, ref convexColliderB->m_ConvexHull,
                        worldFromA, aFromB, maxDistance, out contactManifold);
                    break;
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return;
            }

            WriteManifold(contactManifold, context, colliderKeys, materialA, materialB, flipped);
        }
    }
}
