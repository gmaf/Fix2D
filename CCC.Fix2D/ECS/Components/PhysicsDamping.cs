using Unity.Entities;

namespace CCC.Fix2D
{
    // Optional damping applied to the physics body velocities during each simulation step.
    // This scales the velocities using: math.clamp(1 - damping * Timestep, 0, 1)
    [GenerateAuthoringComponent]
    public struct PhysicsDamping : IComponentData
    {
        public float Linear;
        public float Angular;
    }
}
