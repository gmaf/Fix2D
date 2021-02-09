using Unity.Entities;

namespace CCC.Fix2D
{
    // A component that encapsulates the PhysicsSettings.
    public struct PhysicsSettingsComponent : IComponentData
    {
        public PhysicsStepSettings Value;
    }
}
