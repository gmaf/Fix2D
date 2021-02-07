using Unity.Entities;
using UnityEngine;
using CCC.Fix2D.Debugging;

namespace CCC.Fix2D.Authoring
{
    [DisallowMultipleComponent]
    internal sealed class Fix2DGizmosSettings : MonoBehaviour, IConvertGameObjectToEntity
    {
        Fix2DGizmosSettings() { }

        [Header("Collider Geometry")]
        public bool DrawStaticColliders = false;
        public bool DrawDynamicColliders = false;
        public Color StaticColliderColor = new Color(0f, 0.5f, 0f, 0.5f);
        public Color DynamicColliderColor = Color.green;

        [Header("Collider AABB")]
        public bool DrawColliderAabbs = false;
        public Color ColliderAabbColor = Color.yellow;

        [Header("Broadphase")]
        public bool DrawBroadphase = false;
        public Color StaticBroadphaseColor = Color.blue;
        public Color DynamicBroadphaseColor = Color.gray;

        [Header("Contacts")]
        public bool DrawCollisions = false;
        public Color CollisionsColor = Color.red;
        public bool DrawTriggers = false;
        public Color TriggersColor = Color.red;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(
                entity,
                new PhysicsDebugDisplay
                {
                    DrawStaticColliders = DrawStaticColliders,
                    DrawDynamicColliders = DrawDynamicColliders,
                    DrawColliderAabbs = DrawColliderAabbs,
                    DrawBroadphase = DrawBroadphase,
                    DrawCollisions = DrawCollisions,

                    StaticColliderColor = (Vector4)StaticColliderColor,
                    DynamicColliderColor = (Vector4)DynamicColliderColor,
                    ColliderAabbColor = (Vector4)ColliderAabbColor,
                    StaticBroadphaseColor = (Vector4)StaticBroadphaseColor,
                    DynamicBroadphaseColor = (Vector4)DynamicBroadphaseColor,
                    CollisionsColor = (Vector4)CollisionsColor,
                }
            );
        }
    }
}