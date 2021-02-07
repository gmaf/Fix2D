using System;
using Unity.Assertions;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    /// <summary>
    /// Defines the collision response policy of a collider
    /// </summary>
    public enum CollisionResponsePolicy : byte
    {
        /// <summary>
        /// The collider will collide normally
        /// </summary>
        Collide = 0,
        /// <summary>
        /// The collider will collide normally and raise collision events
        /// </summary>
        CollideRaiseCollisionEvents = 1,
        /// <summary>
        /// The collider will raise trigger events when it overlaps another collider
        /// </summary>
        RaiseTriggerEvents = 3,
        /// <summary>
        /// The collider will skip collision, but can still move and intercept queries
        /// </summary>
        None = byte.MaxValue - 1,
    }

    // Describes how an object should respond to collisions with other objects.
    public struct PhysicsMaterial
    {
        public MaterialFlags Flags;
        public float Friction;
        public float Restitution;
        public CombinePolicy FrictionCombinePolicy;
        public CombinePolicy RestitutionCombinePolicy;

        // If true, the object does not collide but raises trigger events instead
        public bool IsTrigger => (Flags & MaterialFlags.IsTrigger) != 0;
        
        // If true, the object raises collision events if an impulse is applied during solving
        public bool EnableCollisionEvents => (Flags & MaterialFlags.EnableCollisionEvents) != 0;

        public CollisionResponsePolicy CollisionResponse
        {
            get => FlagsToCollisionResponse(Flags);
            set
            {
                switch (value)
                {
                    case CollisionResponsePolicy.None:
                        Flags |= MaterialFlags.DisableCollisions;
                        Flags &= ~MaterialFlags.IsTrigger & ~MaterialFlags.EnableCollisionEvents;
                        return;
                    case CollisionResponsePolicy.RaiseTriggerEvents:
                        Flags |= MaterialFlags.IsTrigger;
                        Flags &= ~MaterialFlags.DisableCollisions & ~MaterialFlags.EnableCollisionEvents;
                        return;
                    case CollisionResponsePolicy.CollideRaiseCollisionEvents:
                        Flags |= MaterialFlags.EnableCollisionEvents;
                        Flags &= ~MaterialFlags.IsTrigger & ~MaterialFlags.DisableCollisions;
                        return;
                    case CollisionResponsePolicy.Collide:
                        Flags &= ~MaterialFlags.DisableCollisions & ~MaterialFlags.EnableCollisionEvents & ~MaterialFlags.IsTrigger;
                        return;
                    default:
                        Assert.IsTrue(false, "Invalid collision response provided!");
                        return;
                }
            }
        }

        [Flags]
        public enum MaterialFlags
        {
            None = 0,
            IsTrigger = 1 << 0,
            EnableCollisionEvents = 1 << 1,
            EnableMassFactors = 1 << 2,
            EnableSurfaceVelocity = 1 << 3,
            DisableCollisions = 1 << 4
        }

        // Describes how to mix material properties.
        public enum CombinePolicy
        {
            // sqrt(a * b)
            GeometricMean,

            // min(a, b)
            Minimum,
                       
            // max(a, b)
            Maximum,

            // (a + b) / 2
            ArithmeticMean
        }

        public static readonly PhysicsMaterial Default = new PhysicsMaterial
        {
            Friction = 0.4f,
            Restitution = 0.0f,
            FrictionCombinePolicy = CombinePolicy.GeometricMean,
            RestitutionCombinePolicy = CombinePolicy.Maximum
        };

        // Get combined collision response of the 2 materials.
        // Used only internally by the manifold creation pipeline.
        internal static CollisionResponsePolicy GetCombinedCollisionResponse(PhysicsMaterial materialA, PhysicsMaterial materialB)
        {
            var flags = materialA.Flags | materialB.Flags;
            return FlagsToCollisionResponse(flags);
        }

        // Get a combined friction value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedFriction(in PhysicsMaterial materialA, in PhysicsMaterial materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.FrictionCombinePolicy, (int)materialB.FrictionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Friction * materialB.Friction);

                case CombinePolicy.Minimum:
                    return math.min(materialA.Friction, materialB.Friction);

                case CombinePolicy.Maximum:
                    return math.max(materialA.Friction, materialB.Friction);

                case CombinePolicy.ArithmeticMean:
                    return (materialA.Friction + materialB.Friction) * 0.5f;

                default:
                    return 0;
            }
        }

        // Get a combined restitution value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static float GetCombinedRestitution(in PhysicsMaterial materialA, in PhysicsMaterial materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.RestitutionCombinePolicy, (int)materialB.RestitutionCombinePolicy);

            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Restitution * materialB.Restitution);

                case CombinePolicy.Minimum:
                    return math.min(materialA.Restitution, materialB.Restitution);

                case CombinePolicy.Maximum:
                    return math.max(materialA.Restitution, materialB.Restitution);

                case CombinePolicy.ArithmeticMean:
                    return (materialA.Restitution + materialB.Restitution) * 0.5f;

                default:
                    return 0;
            }
        }

        public bool Equals(PhysicsMaterial other)
        {
            return
                Flags == other.Flags &&
                FrictionCombinePolicy == other.FrictionCombinePolicy &&
                RestitutionCombinePolicy == other.RestitutionCombinePolicy &&
                Friction.Equals(other.Friction) &&
                Restitution.Equals(other.Restitution);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                unchecked((uint)(
                    (byte)Flags
                    | ((byte)FrictionCombinePolicy << 4)
                    | ((byte)RestitutionCombinePolicy << 8))
                ),
                math.hash(new float2(Friction, Restitution))
            )));
        }

        private static CollisionResponsePolicy FlagsToCollisionResponse(MaterialFlags flags)
        {
            if ((flags & MaterialFlags.DisableCollisions) != 0)
            {
                return CollisionResponsePolicy.None;
            }
            else if ((flags & MaterialFlags.IsTrigger) != 0)
            {
                return CollisionResponsePolicy.RaiseTriggerEvents;
            }
            else if ((flags & MaterialFlags.EnableCollisionEvents) != 0)
            {
                return CollisionResponsePolicy.CollideRaiseCollisionEvents;
            }
            else
            {
                return CollisionResponsePolicy.Collide;
            }
        }
    }
}
