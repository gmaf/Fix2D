using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D.Debugging
{
    public struct PhysicsDebugDisplay : IComponentData
    {
        public bool DrawStaticColliders;
        public bool DrawDynamicColliders;
        public bool DrawColliderAabbs;
        public bool DrawBroadphase;
        public bool DrawCollisions;
        public bool DrawTriggers;

        public float4 StaticColliderColor;
        public float4 DynamicColliderColor;
        public float4 ColliderAabbColor;
        public float4 StaticBroadphaseColor;
        public float4 DynamicBroadphaseColor;
        public float4 CollisionsColor;
        public float4 TriggersColor;
    }
}
