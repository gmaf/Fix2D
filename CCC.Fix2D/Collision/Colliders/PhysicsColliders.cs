using System;
using System.Runtime.InteropServices;

using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Entities;

namespace CCC.Fix2D
{
    public enum ColliderType
    {
        Invalid = 0,

        // Convex types.
        Box = 1,
        Polygon = 2,
        Circle = 3,
        Capsule = 4,
    }

    public enum CollisionType
    {
        Invalid = 0,

        Convex = 1,
    }

    interface ICollider : IQueryable
    {
        ColliderType ColliderType { get; }
        CollisionType CollisionType { get; }
        MassProperties MassProperties { get; }

        // The total size of the collider in memory
        int MemorySize { get; }
    }

    interface IConvexCollider : ICollider
    {
        CollisionFilter Filter { get; set; }
        PhysicsMaterial Material { get; set; }

        int VertexCount { get; }
        ConvexHull.ConvexArray.Accessor Vertices { get; }
        ConvexHull.ConvexArray.Accessor Normals { get; }
    }

    //interface ICompositeCollider : ICollider
    //{
    //    // The combined filter of all the child colliders.
    //    CollisionFilter Filter { get; }

    //    //// The maximum number of bits needed to identify a child of this collider.
    //    //uint NumColliderKeyBits { get; }

    //    //// Get a child of this collider.
    //    //// Return false if the key is not valid.
    //    //bool GetChild(ref ColliderKey key, out ChildCollider child);

    //    //// Get a leaf of this collider.
    //    //// Return false if the key is not valid.
    //    //bool GetLeaf(ColliderKey key, out ChildCollider leaf);

    //    //// Get all the leaves of this collider.
    //    //void GetLeaves<T>(ref T collector) where T : struct, ILeafColliderCollector;
    //}

    public interface ILeafColliderCollector
    {
        void AddLeaf(ColliderKey key, ref ChildCollider leaf);

        void PushCompositeCollider(ColliderKeyPath compositeKey, PhysicsTransform parentFromComposite, out PhysicsTransform worldFromParent);

        void PopCompositeCollider(uint numCompositeKeyBits, PhysicsTransform worldFromParent);
    }

    // Header common to all colliders.
    struct ColliderHeader
    {
        public ColliderType ColliderType;
        public CollisionType CollisionType;
        public uint UserData;   // User data. Not used by the physics system itself.
        public byte Version;    // increment whenever the collider data has changed
        public byte Magic;      // always = 0xff (for validation)

        public CollisionFilter Filter;

        public struct Constants
        {
            public const byte Magic = 0xff;
        }

        public void SetDirty()
        {
            Version += 1;
        }
    };

    // Convex colliders only.
    struct ConvexColliderHeader
    {
        public ColliderType ColliderType;
        public CollisionType CollisionType;
        public uint UserData;   // User data. Not used by the physics system itself.
        public byte Version;    // increment whenever the collider data has changed
        public byte Magic;      // always = 0xff (for validation)

        public CollisionFilter Filter;
        public PhysicsMaterial Material;

        public void SetDirty()
        {
            Version += 1;
        }
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct Collider : IConvexCollider
    {
        public struct BoxConstants
        {
            public const int MaxVertexCount = 4;
        }
        public struct PolygonConstants
        {
            public const int MaxVertexCount = 16;
        }

        ConvexColliderHeader m_Header;
        internal ConvexHull m_ConvexHull; // 24

        // Circle
        float2 m_DummyNormal;

        // Circle & Box
        float2 m_Center;

        // Capsule
        float2 m_CapsuleVertex0;
        float2 m_CapsuleVertex1;
        float2 m_CapsuleNormal0;
        float2 m_CapsuleNormal1;

        // Box
        float2 m_Size;
        float m_Angle;
        unsafe fixed byte m_BoxVertices[(sizeof(float) * 2) * BoxConstants.MaxVertexCount];
        unsafe fixed byte m_BoxNormals[(sizeof(float) * 2) * BoxConstants.MaxVertexCount];

        // Polygon
        unsafe fixed byte m_PolygonVertices[(sizeof(float) * 2) * PolygonConstants.MaxVertexCount];
        unsafe fixed byte m_PolygonNormals[(sizeof(float) * 2) * PolygonConstants.MaxVertexCount];

        public float Angle => m_Angle;
        public float2 Size => m_Size;
        public float2 Center => m_Center;
        public float2 Vertex0 => m_CapsuleVertex0;
        public float2 Vertex1 => m_CapsuleVertex1;
        public float Radius => m_ConvexHull.ConvexRadius;
        public float BevelRadius => m_ConvexHull.ConvexRadius;

        public BoxGeometry BoxGeometry
        {
            get => new BoxGeometry
            {
                Size = m_Size,
                Center = m_Center,
                Angle = m_Angle,
                BevelRadius = m_ConvexHull.ConvexRadius
            };
            set
            {
                if (!value.Equals(BoxGeometry))
                {
                    SetGeometry(value);
                }
            }
        }

        public CapsuleGeometry CapsuleGeometry
        {
            get => new CapsuleGeometry
            {
                Vertex0 = m_CapsuleVertex0,
                Vertex1 = m_CapsuleVertex1,
                Radius = Radius
            };
            set
            {
                if (!value.Equals(CapsuleGeometry))
                {
                    SetGeometry(value);
                }
            }
        }
        public CircleGeometry CircleGeometry
        {
            get => new CircleGeometry
            {
                Center = m_Center,
                Radius = m_ConvexHull.ConvexRadius
            };
            set
            {
                if (!value.Equals(CircleGeometry))
                {
                    SetGeometry(value);
                }
            }
        }

        public PolygonGeometry PolygonGeometry
        {
            set => SetGeometry(value);
        }

        #region ICollider

        public ColliderType ColliderType => m_Header.ColliderType;
        public CollisionType CollisionType => m_Header.CollisionType;

        public MassProperties MassProperties { get; set; }

        public unsafe int MemorySize => UnsafeUtility.SizeOf<Collider>();

        #endregion

        #region IConvexCollider
        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) m_Header.Filter = value; m_Header.SetDirty(); } }
        public PhysicsMaterial Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) m_Header.Material = value; m_Header.SetDirty(); } }

        public int VertexCount => m_ConvexHull.Length;
        public ConvexHull.ConvexArray.Accessor Vertices => m_ConvexHull.Vertices;
        public ConvexHull.ConvexArray.Accessor Normals => m_ConvexHull.Normals;
        #endregion

        #region IQueryable

        public Aabb CalculateAabb()
        {
            return CalculateAabb(PhysicsTransform.Identity);
        }

        public unsafe Aabb CalculateAabb(PhysicsTransform transform)
        {
            switch (ColliderType)
            {
                case ColliderType.Box:
                    return m_ConvexHull.CalculateAabb(transform);

                case ColliderType.Polygon:
                    return m_ConvexHull.CalculateAabb(transform);

                case ColliderType.Circle:
                    var center = PhysicsMath.mul(transform, m_Center);
                    return new Aabb()
                    {
                        Min = center - Radius,
                        Max = center + Radius
                    };

                case ColliderType.Capsule:
                    var vertex0 = PhysicsMath.mul(transform, m_CapsuleVertex0);
                    var vertex1 = PhysicsMath.mul(transform, m_CapsuleVertex1);
                    return new Aabb()
                    {
                        Min = math.min(vertex0, vertex1) - Radius,
                        Max = math.max(vertex0, vertex1) + Radius
                    };

                case ColliderType.Invalid:
                default:
                    SafetyChecks.ThrowInvalidOperationException("Unknown or invalid physics collider.");
                    return default;
            }
        }

        // Overlap point.
        public bool OverlapPoint(OverlapPointInput input) => QueryWrappers.OverlapPoint(ref this, input);
        public bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit) => QueryWrappers.OverlapPoint(ref this, input, out hit);
        public bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) => QueryWrappers.OverlapPoint(ref this, input, ref allHits);
        public bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>
        {
            return OverlapQueries.OverlapPoint(input, ref this, ref collector);
        }

        // Check a collider against this body.
        public bool OverlapCollider(OverlapColliderInput input) => QueryWrappers.OverlapCollider(ref this, input);
        public bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit) => QueryWrappers.OverlapCollider(ref this, input, out hit);
        public bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) => QueryWrappers.OverlapCollider(ref this, input, ref allHits);
        public bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>
        {
            return OverlapQueries.OverlapCollider(input, ref this, ref collector);
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return RaycastQueries.RayCollider(input, ref this, ref collector);
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return ColliderCastQueries.CastCollider(input, ref this, ref collector);
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return DistanceQueries.PointDistance(input, ref this, ref collector);
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return DistanceQueries.ColliderDistance(input, ref this, ref collector);
        }

        #endregion

        #region Construction
        public static BlobAssetReference<Collider> Create(BoxGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var collider = new Collider();
            collider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<Collider>());
        }

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var collider = new Collider();
            collider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<Collider>());
        }

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new Collider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<Collider>());
        }

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, PhysicsMaterial.Default);

        public static BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material) =>
            Create(geometry, filter, material, 0);

        public static unsafe BlobAssetReference<Collider> Create(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            var physicsCollider = new Collider();
            physicsCollider.Initialize(geometry, filter, material, userData);
            return BlobAssetReference<Collider>.Create(&physicsCollider, UnsafeUtility.SizeOf<Collider>());
        }

        void Initialize(BoxGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Box,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(BoxGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_Size = geometry.Size;
            m_Center = geometry.Center;
            m_Angle = geometry.Angle;

            fixed (Collider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, collider->m_BoxVertices, collider->m_BoxNormals, BoxConstants.MaxVertexCount, geometry.BevelRadius);
            }

            var extents = m_Size * 0.5f;

            var xf = new PhysicsTransform(m_Center, m_Angle);

            var vertices = m_ConvexHull.Vertices.GetUnsafePtr();
            vertices[0] = PhysicsMath.mul(xf, -extents);
            vertices[1] = PhysicsMath.mul(xf, new float2(extents.x, -extents.y));
            vertices[2] = PhysicsMath.mul(xf, extents);
            vertices[3] = PhysicsMath.mul(xf, new float2(-extents.x, extents.y));

            var normals = m_ConvexHull.Normals.GetUnsafePtr();
            normals[0] = math.mul(xf.Rotation, new float2(0.0f, -1.0f));
            normals[1] = math.mul(xf.Rotation, new float2(1.0f, 0.0f));
            normals[2] = math.mul(xf.Rotation, new float2(0.0f, 1.0f));
            normals[3] = math.mul(xf.Rotation, new float2(-1.0f, 0.0f));

            MassProperties = m_ConvexHull.GetMassProperties();
        }

        public void Initialize(CapsuleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Capsule,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            m_CapsuleVertex0 = geometry.Vertex0;
            m_CapsuleVertex1 = geometry.Vertex1;

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(CapsuleGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_CapsuleVertex0 = geometry.Vertex0;
            m_CapsuleVertex1 = geometry.Vertex1;

            fixed (Collider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, (byte*)&collider->m_CapsuleVertex0, (byte*)&collider->m_CapsuleNormal0, 2, geometry.Radius);
            }

            var radiusSqr = geometry.Radius * geometry.Radius;
            var bodyLength = math.distance(m_CapsuleVertex0, m_CapsuleVertex1);
            var bodyArea = bodyLength * geometry.Radius * 2.0f;
            var bodyMass = bodyArea;
            var bodyInertia = bodyMass * (bodyLength * bodyLength + radiusSqr) / 12.0f;

            var capsArea = math.PI * radiusSqr;
            var capsMass = capsArea;
            var capsInertia = capsMass * (0.5f * radiusSqr + bodyLength * bodyLength * 0.25f);

            var mass = bodyMass + capsArea;
            var localCenterOfMass = 0.5f * (m_CapsuleVertex0 + m_CapsuleVertex1);
            var area = bodyArea + capsArea;
            var inertia = bodyInertia + capsInertia + mass * math.dot(localCenterOfMass, localCenterOfMass);
            var angularExpansionFactor = math.length(m_CapsuleVertex1 - m_CapsuleVertex0) * 0.5f;

            m_CapsuleNormal0 = PhysicsMath.perp(math.normalize(m_CapsuleVertex1 - m_CapsuleVertex0));
            m_CapsuleNormal1 = -m_CapsuleNormal0;

            // Set mass properties.
            MassProperties = new MassProperties(
                localCenterOfMass: localCenterOfMass,
                inertia: inertia,
                area: area,
                angularExpansionFactor: angularExpansionFactor);
        }

        public void Initialize(CircleGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Center = geometry.Center;

            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Circle,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(CircleGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            m_Center = geometry.Center;

            fixed (Collider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, (byte*)&collider->m_Center, (byte*)&collider->m_DummyNormal, 1, geometry.Radius);
            }

            var radiusSqr = geometry.Radius * geometry.Radius;
            var area = math.PI * radiusSqr;
            var mass = area;
            var localCenterOfMass = m_Center;
            var inertia = mass * ((radiusSqr * 0.5f) + math.dot(localCenterOfMass, localCenterOfMass));

            // Set mass properties.
            MassProperties = new MassProperties(
                localCenterOfMass: localCenterOfMass,
                inertia: inertia,
                area: area,
                angularExpansionFactor: 0.0f);
        }

        public void Initialize(PolygonGeometry geometry, CollisionFilter filter, PhysicsMaterial material, uint userData)
        {
            m_Header = new ConvexColliderHeader
            {
                ColliderType = ColliderType.Polygon,
                CollisionType = CollisionType.Convex,
                UserData = userData,
                Version = 0,
                Magic = ColliderHeader.Constants.Magic,
                Filter = filter,
                Material = material
            };

            SetGeometry(geometry);
        }

        unsafe void SetGeometry(PolygonGeometry geometry)
        {
            geometry.Validate();

            m_Header.SetDirty();

            fixed (Collider* collider = &this)
            {
                m_ConvexHull = new ConvexHull(ref m_ConvexHull, collider->m_PolygonVertices, collider->m_PolygonNormals, geometry.Vertices.Length, geometry.BevelRadius);
                m_ConvexHull.SetAndGiftWrap(geometry.Vertices);
            }

            MassProperties = m_ConvexHull.GetMassProperties();
        }
        #endregion
    }

    // An opaque key which packs a path to a specific leaf of a collider hierarchy into a single integer.
    public struct ColliderKey : IEquatable<ColliderKey>
    {
        public uint Value { get; internal set; }

        public static readonly ColliderKey Empty = new ColliderKey { Value = uint.MaxValue };

        internal ColliderKey(uint numSubKeyBits, uint subKey)
        {
            Value = uint.MaxValue;
            PushSubKey(numSubKeyBits, subKey);
        }

        public bool Equals(ColliderKey other)
        {
            return Value == other.Value;
        }

        // Append a sub key to the front of the path
        // "numSubKeyBits" is the maximum number of bits required to store any value for this sub key.
        // Returns false if the key is empty.
        public void PushSubKey(uint numSubKeyBits, uint subKey)
        {
            var parentPart = (uint)((ulong)subKey << 32 - (int)numSubKeyBits);
            var childPart = Value >> (int)numSubKeyBits;
            Value = parentPart | childPart;
        }

        // Extract a sub key from the front of the path.
        // "numSubKeyBits" is the maximum number of bits required to store any value for this sub key.
        // Returns false if the key is empty.
        public bool PopSubKey(uint numSubKeyBits, out uint subKey)
        {
            if (Value != uint.MaxValue)
            {
                subKey = Value >> (32 - (int)numSubKeyBits);
                Value = ((1 + Value) << (int)numSubKeyBits) - 1;
                return true;
            }

            subKey = uint.MaxValue;
            return false;
        }
    }

    // Stores a ColliderKey along with the number of bits in it that are used.
    // This is useful for building keys from root to leaf, the bit count shows where to place the child key bits
    public struct ColliderKeyPath
    {
        ColliderKey m_Key;
        uint m_NumKeyBits;

        public ColliderKey Key => m_Key;

        public static ColliderKeyPath Empty => new ColliderKeyPath(ColliderKey.Empty, 0);

        public ColliderKeyPath(ColliderKey key, uint numKeyBits)
        {
            m_Key = key;
            m_NumKeyBits = numKeyBits;
        }

        // Append the local key for a child of the shape referenced by this path
        public void PushChildKey(ColliderKeyPath child)
        {
            m_Key.Value &= (uint)(child.m_Key.Value >> (int)m_NumKeyBits | (ulong)0xffffffff << (int)(32 - m_NumKeyBits));
            m_NumKeyBits += child.m_NumKeyBits;
        }

        // Remove the most leafward shape's key from this path
        public void PopChildKey(uint numChildKeyBits)
        {
            m_NumKeyBits -= numChildKeyBits;
            m_Key.Value |= (uint)((ulong)0xffffffff >> (int)m_NumKeyBits);
        }

        // Get the collider key for a leaf shape that is a child of the shape referenced by this path
        public ColliderKey GetLeafKey(ColliderKey leafKeyLocal)
        {
            var leafPath = this;
            leafPath.PushChildKey(new ColliderKeyPath(leafKeyLocal, 0));
            return leafPath.Key;
        }
    }

    // A pair of collider keys.
    public struct ColliderKeyPair
    {
        public ColliderKey ColliderKeyA;
        public ColliderKey ColliderKeyB;

        public static readonly ColliderKeyPair Empty = new ColliderKeyPair { ColliderKeyA = ColliderKey.Empty, ColliderKeyB = ColliderKey.Empty };

        internal ColliderKeyPair GetSwapped()
        {
            return new ColliderKeyPair
            {
                ColliderKeyA = ColliderKeyB,
                ColliderKeyB = ColliderKeyA
            };
        }
    }

    // A child/leaf collider.
    public unsafe struct ChildCollider
    {
        readonly Collider* m_Collider;

        // The transform of the child collider in whatever space it was queried from
        public PhysicsTransform TransformFromChild;

        public Collider* Collider
        {
            get
            {
                fixed (ChildCollider* self = &this)
                {
                    // Accessing uninitialized Collider.
                    SafetyChecks.IsTrue(m_Collider != null);
                    return self->m_Collider;
                }
            }
        }

        // Create from collider
        public ChildCollider(Collider* collider)
        {
            m_Collider = collider;
            TransformFromChild = PhysicsTransform.Identity;
        }

        // Create from body
        public ChildCollider(Collider* collider, PhysicsTransform transform)
        {
            m_Collider = collider;
            TransformFromChild = transform;
        }

        // Combine a parent ChildCollider with another ChildCollider describing one of its children
        public ChildCollider(ChildCollider parent, ChildCollider child)
        {
            m_Collider = child.m_Collider;
            TransformFromChild = PhysicsMath.mul(parent.TransformFromChild, child.TransformFromChild);
        }
    }

    // Extension method to make accessing a collider easier from an asset reference.
    public static unsafe class BlobAssetReferenceColliderExtension
    {
        public static Collider* GetColliderPtr(this Unity.Entities.BlobAssetReference<Collider> assetReference)
        {
            return (Collider*)assetReference.GetUnsafePtr();
        }
    }
}
