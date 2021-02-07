using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    // The linear and angular velocities of a physics body.
    // If not present, then the physics body becomes Static.
    [GenerateAuthoringComponent]
    public struct PhysicsVelocity : IComponentData
    {
        // The world-space linear velocity in the XY plane.
        public float2 Linear;

        // The world-space angular velocity around the Z axis.
        public float Angular;

        public static readonly PhysicsVelocity Zero = new PhysicsVelocity
        {
            Linear = new float2(0),
            Angular = 0f
        };
    }
}
