using Unity.Entities;

namespace CCC.Fix2D
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class PhysicsSystemGroup : ComponentSystemGroup
    {
    }
}
