using System;

using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    #region Query Input & Output

    public struct OverlapAabbInput : IQueryInput
    {
        public Aabb Aabb;
        public CollisionFilter Filter;
        public IgnoreHit Ignore { get; set; }

        public static OverlapAabbInput Default =>
            new OverlapAabbInput
            {
                Aabb = Aabb.Empty,
                Filter = CollisionFilter.Default
            };
    }

    public struct OverlapPointInput : IQueryInput
    {
        public float2 Position;
        public CollisionFilter Filter;
        public IgnoreHit Ignore { get; set; }

        internal QueryContext QueryContext;

        public static OverlapPointInput Default =>
            new OverlapPointInput
            {
                Filter = CollisionFilter.Default
            };
    }

    public struct OverlapPointHit : IQueryResult
    {
        #region IQueryResult

        public int PhysicsBodyIndex { get; internal set; }
        public ColliderKey ColliderKey { get; internal set; }
        public Entity Entity { get; internal set; }
        public float Fraction { get; internal set; }

        public bool IsValid => Entity != Entity.Null;

        #endregion

        public float2 Position { get; internal set; }
    }

    public struct OverlapColliderInput : IQueryInput
    {
        public BlobAssetReference<Collider> Collider;
        public PhysicsTransform Transform;
        public CollisionFilter Filter;
        public IgnoreHit Ignore { get; set; }

        internal QueryContext QueryContext;

        public static OverlapColliderInput Default =>
            new OverlapColliderInput
            {
                Transform = PhysicsTransform.Identity,
                Filter = CollisionFilter.Default
            };
    }

    public struct OverlapColliderHit : IQueryResult
    {
        #region IQueryResult

        public int PhysicsBodyIndex { get; internal set; }
        public ColliderKey ColliderKey { get; internal set; }
        public Entity Entity { get; internal set; }
        public float Fraction { get; internal set; }

        public bool IsValid => Entity != Entity.Null;

        #endregion
    }

    // Interface for collecting hits from overlap queries
    public interface IOverlapCollector
    {
        unsafe void AddPhysicsBodyIndices(int* indices, int count);
        unsafe void AddColliderKeys(ColliderKey* keys, int count);
        void PushCompositeCollider(ColliderKeyPath compositeKey);
        void PopCompositeCollider(uint numCompositeKeyBits);
    }

    #endregion

    // Overlap query implementations
    public static class OverlapQueries
    {
        #region Overlap Point

        // Point/Convex intersection.
        static unsafe bool PointConvex(float2 point, ref ConvexHull hull)
        {
            var vertices = hull.Vertices.GetUnsafePtr();
            var normals = hull.Normals.GetUnsafePtr();

            var length = hull.Length;
            for (var i = 0; i < length; ++i)
            {
                if (math.dot(*normals++, point - *vertices++) > 0f)
                    return false;
            }

            return true;
        }

        // Point/Capsule intersection.
        internal static bool PointCapsule(float2 point, float2 vertex0, float2 vertex1, float radius)
        {
            var localDistance = QueryUtility.NearestPointOnLineSegment(point, vertex0, vertex1) - point;
            return math.lengthsq(localDistance) <= (radius * radius);
        }

        // Point/Circle intersection.
        static bool PointCircle(float2 point, float2 center, float radius)
        {
            return math.lengthsq(point - center) <= (radius * radius);
        }

        internal static bool OverlapPoint<T>(OverlapPointInput input, ref Collider collider, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            // Nothing to do if:
            // - MaxFraction is zero.
            // - Filtered out.
            if (math.abs(collector.MaxFraction) < 0f ||
                !CollisionFilter.IsCollisionEnabled(input.Filter, collider.Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            bool hadHit;
            switch (collider.ColliderType)
            {
                case ColliderType.Box:
                {
                    hadHit = PointConvex(input.Position, ref collider.m_ConvexHull);
                    break;
                }

                case ColliderType.Polygon:
                {
                    hadHit = PointConvex(input.Position, ref collider.m_ConvexHull);
                    break;
                }

                case ColliderType.Capsule:
                {
                    hadHit = PointCapsule(input.Position, collider.Vertex0, collider.Vertex1, collider.Radius);
                    break;
                }

                case ColliderType.Circle:
                {
                    hadHit = PointCircle(input.Position, collider.Center, collider.Radius);
                    break;
                }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            if (hadHit)
            {
                var hit = new OverlapPointHit
                {
                    Fraction = 0f,
                    Position = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, input.Position),

                    PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex,
                    ColliderKey = input.QueryContext.ColliderKey,
                    Entity = input.QueryContext.Entity
                };

                return collector.AddHit(hit);
            }
            return false;
        }

        #region Overlap Point Compound

        //unsafe struct OverlapPointCompoundLeafProcessor : BoundingVolumeHierarchy.IPointOverlapLeafProcessor
        //{
        //    readonly PhysicsCompoundCollider* m_CompoundCollider;

        //    public OverlapPointCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
        //    {
        //        m_CompoundCollider = compoundCollider;
        //    }

        //    public bool PointLeaf<T>(OverlapPointInput input, int leafData, ref T collector) where T : struct, ICollector<OverlapPointHit>
        //    {
        //        ref var child = ref m_CompoundCollider->Children[leafData];

        //        if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
        //        {
        //            return false;
        //        }

        //        // Transform the point into child space.
        //        var inputLs = input;
        //        {
        //            var compoundFromChild = child.CompoundFromChild;
        //            var childFromCompound = PhysicsMath.inverse(compoundFromChild);

        //            inputLs.Position = PhysicsMath.mul(childFromCompound, input.Position);

        //            inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
        //            inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
        //            inputLs.QueryContext.LocalToWorldTransform = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, compoundFromChild);
        //        }

        //        return child.Collider->OverlapPoint(inputLs, ref collector);
        //    }
        //}

        //static unsafe bool PointCompound<T>(OverlapPointInput input, PhysicsCompoundCollider* compoundCollider, ref T collector) where T : struct, ICollector<OverlapPointHit>
        //{
        //    if (!CollisionFilter.IsCollisionEnabled(input.Filter, compoundCollider->Filter))
        //    {
        //        return false;
        //    }

        //    var leafProcessor = new OverlapPointCompoundLeafProcessor(compoundCollider);
        //    return compoundCollider->BoundingVolumeHierarchy.OverlapPoint(input, ref leafProcessor, ref collector);
        //}

        #endregion

        #endregion

        #region Overlap Collider

        internal static bool OverlapConvexConvex(ref PhysicsTransform transform, ref DistanceProxy proxySource, ref DistanceProxy proxyTarget)
        {
            var hit = DistanceQueries.ColliderDistance(transform, ref proxySource, ref proxyTarget);

            return hit.Distance < 10f * float.Epsilon;
        }

        static bool OverlapConvex(ref OverlapColliderInput input, ref DistanceProxy proxyTarget)
        {
            if (!input.Collider.IsCreated)
                return false;

            var inputColliderBlob = input.Collider;
            switch (inputColliderBlob.Value.ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                {
                    ref var convexHull = ref inputColliderBlob.Value.m_ConvexHull;
                    var proxySource = new DistanceProxy(ref convexHull);
                    return OverlapConvexConvex(ref input.Transform, ref proxySource, ref proxyTarget);
                }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }
        }

        internal static bool OverlapCollider<T>(OverlapColliderInput input, ref Collider collider, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            // Nothing to do if:
            // - MaxFraction is zero.
            // - The Collider isn't valid.
            // - Filtered out.
            if (collector.MaxFraction < float.Epsilon ||
                !input.Collider.IsCreated ||
                !CollisionFilter.IsCollisionEnabled(input.Filter, collider.Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            var proxyTarget = new DistanceProxy(ref collider.m_ConvexHull);
            bool hadHit = OverlapConvex(ref input, ref proxyTarget);

            if (hadHit)
            {
                var hit = new OverlapColliderHit
                {
                    Fraction = 0f,

                    PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex,
                    ColliderKey = input.QueryContext.ColliderKey,
                    Entity = input.QueryContext.Entity
                };

                return collector.AddHit(hit);
            }
            return false;
        }

        #region Overlap Collider Compound

        //unsafe struct OverlapCompoundLeafProcessor : BoundingVolumeHierarchy.IColliderOverlapLeafProcessor
        //{
        //    readonly PhysicsCompoundCollider* m_CompoundCollider;

        //    public OverlapCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
        //    {
        //        m_CompoundCollider = compoundCollider;
        //    }

        //    public bool ColliderLeaf<T>(OverlapColliderInput input, int leafData, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        //    {
        //        ref var child = ref m_CompoundCollider->Children[leafData];

        //        if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
        //        {
        //            return false;
        //        }

        //        // Transform the query into child space
        //        var inputLs = input;
        //        {
        //            var compoundFromChild = child.CompoundFromChild;
        //            var childFromCompound = PhysicsMath.inverse(child.CompoundFromChild);

        //            inputLs.Transform = PhysicsMath.mul(childFromCompound, input.Transform);

        //            inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
        //            inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
        //            inputLs.QueryContext.LocalToWorldTransform = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, compoundFromChild);
        //        }

        //        return child.Collider->OverlapCollider(inputLs, ref collector);
        //    }
        //}

        //static unsafe bool OverlapCompound<T>(OverlapColliderInput input, PhysicsCompoundCollider* compoundCollider, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        //{
        //    if (!CollisionFilter.IsCollisionEnabled(input.Filter, compoundCollider->Filter))
        //    {
        //        return false;
        //    }

        //    var leafProcessor = new OverlapCompoundLeafProcessor(compoundCollider);
        //    return compoundCollider->BoundingVolumeHierarchy.OverlapCollider(input, ref leafProcessor, ref collector);
        //}

        #endregion

        #endregion

        #region Overlap AABB

        internal static unsafe void AabbCollider<T>(OverlapAabbInput input, Collider* collider, ref T collector)
            where T : struct, IOverlapCollector
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider->Filter))
            {
                return;
            }

            switch (collider->ColliderType)
            {
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    break;
            }
        }

        //unsafe struct AabbOverlapCompoundLeafProcessor : BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        //{
        //    readonly PhysicsCompoundCollider* m_CompoundCollider;
        //    readonly uint m_NumColliderKeyBits;

        //    const int k_MaxKeys = 512;
        //    fixed uint m_Keys[k_MaxKeys];  // actually ColliderKeys, but C# doesn't allow fixed arrays of structs
        //    int m_NumKeys;

        //    public AabbOverlapCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
        //    {
        //        m_CompoundCollider = compoundCollider;
        //        m_NumColliderKeyBits = compoundCollider->NumColliderKeyBits;
        //        m_NumKeys = 0;
        //    }

        //    public void AabbLeaf<T>(OverlapAabbInput input, int leafData, ref T collector) where T : struct, IOverlapCollector
        //    {
        //        var childKey = new ColliderKey(m_NumColliderKeyBits, (uint)(leafData));

        //        // Recurse if child is a composite
        //        ref var child = ref m_CompoundCollider->Children[leafData];
        //        //if (child.Collider->CollisionType == CollisionType.Composite)
        //        //{
        //        //    var childInput = input;
        //        //    childInput.Aabb = PhysicsMath.mul(PhysicsMath.inverse(child.CompoundFromChild), input.Aabb);

        //        //    collector.PushCompositeCollider(new ColliderKeyPath(childKey, m_NumColliderKeyBits));
        //        //    AabbCollider(childInput, child.Collider, ref collector);
        //        //    collector.PopCompositeCollider(m_NumColliderKeyBits);
        //        //}
        //        //else
        //        {
        //            m_Keys[m_NumKeys++] = childKey.Value;
        //            if (m_NumKeys > k_MaxKeys - 8)
        //            {
        //                Flush(ref collector);
        //            }
        //        }
        //    }

        //    // Flush keys to collector
        //    internal void Flush<T>(ref T collector) where T : struct, IOverlapCollector
        //    {
        //        fixed (uint* keys = m_Keys)
        //        {
        //            collector.AddColliderKeys((ColliderKey*)keys, m_NumKeys);
        //        }
        //        m_NumKeys = 0;
        //    }
        //}

        //static unsafe void AabbCompound<T>(OverlapAabbInput input, PhysicsCompoundCollider* compound, ref T collector)
        //    where T : struct, IOverlapCollector
        //{
        //    var leafProcessor = new AabbOverlapCompoundLeafProcessor(compound);
        //    compound->BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref collector);
        //    leafProcessor.Flush(ref collector);
        //}

        #endregion
    }
}
