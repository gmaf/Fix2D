using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace CCC.Fix2D.Authoring
{
    [UpdateInGroup(typeof(GameObjectAfterConversionGroup))]
    [ConverterVersion("fred", 1)]
    public class ConvertToFixTransformSystem : GameObjectConversionSystem
    {
        public readonly List<Transform> ToConvert = new List<Transform>();

        protected override void OnUpdate()
        {
            foreach (var transform in ToConvert)
            {
                Convert(transform);
            }

            Entities.ForEach((FixTransformAuth transform) =>
            {
                Convert(transform.transform);
            });

            Entities.ForEach((PhysicsBodyAuth transform) =>
            {
                Convert(transform.transform);
            });
        }

        private void Convert(Transform transform)
        {
            // It's important to use local-space and not world-space to maintain determinism. 
            // (using world-space does calculations while local-space is just data-access

            Entity entity = GetPrimaryEntity(transform);
            float3 localPos = transform.localPosition;
            fix localAngle = fixMath.angle2d(ToFix(transform.localRotation));

            if (!DstEntityManager.HasComponent<FixTranslation>(entity))
                DstEntityManager.AddComponentData(entity, new FixTranslation { Value = ToFix(localPos).xy });

            if (!DstEntityManager.HasComponent<FixRotation>(entity))
                DstEntityManager.AddComponentData(entity, new FixRotation() { Value = localAngle });
            // we don't do anything for the scale at the moment

            RemoveClassicTransform(entity);
        }

        private void RemoveClassicTransform(Entity entity)
        {
            Remove<LocalToWorld>();
            Remove<WorldToLocal>();
            Remove<Parent>();
            Remove<Translation>();
            Remove<Rotation>();
            Remove<Scale>();
            RemoveBuffer<LinkedEntityGroup>();

            void Remove<T>() where T : IComponentData
            {
                if (DstEntityManager.HasComponent<T>(entity))
                    DstEntityManager.RemoveComponent<T>(entity);
            }
            void RemoveBuffer<T>() where T : IBufferElementData
            {
                if (DstEntityManager.HasComponent<T>(entity))
                    DstEntityManager.RemoveComponent<T>(entity);
            }
        }

        private static fixQuaternion ToFix(Quaternion quat)
        {
            return new fixQuaternion((fix)quat.x, (fix)quat.y, (fix)quat.z, (fix)quat.w);
        }

        private static fix3 ToFix(Vector3 vec)
        {
            return new fix3((fix)vec.x, (fix)vec.y, (fix)vec.z);
        }
    }
}