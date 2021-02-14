using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    // The linear and angular velocities of a physics body.
    // If not present, then the physics body becomes Static.
    public struct PhysicsVelocity : IComponentData
    {
        // The world-space linear velocity in the XY plane.
        public fix2 Linear;

        // The world-space angular velocity around the Z axis.
        public fix Angular;

        public PhysicsVelocity(fix2 linear) : this()
        {
            Linear = linear;
        }

        public PhysicsVelocity(fix2 linear, fix angular) : this(linear)
        {
            Angular = angular;
        }

        public float2 LinearFloat
        {
            get => new float2((float)Linear.x, (float)Linear.y);
            set => Linear = new fix2((fix)value.x, (fix)value.y);
        }
        public float AngularFloat
        {
            get => (float)Angular;
            set => Angular = (fix)value;
        }

        public static readonly PhysicsVelocity Zero = new PhysicsVelocity
        {
            Linear = new fix2(0),
            Angular = 0
        };
    }
}
