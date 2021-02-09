using UnityEngine;

using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D.Authoring
{
    [DisallowMultipleComponent]
    [ConverterVersion("2d", 2)]
    sealed class PhysicsSettingsAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        [Tooltip("The Gravity applied to any Dynamic Physics Body.")]
        public float2 Gravity = new float2(0f, -9.81f);

        [Range(0, 128)]
        [Tooltip("The number of available threads that the Physics System can use.")]
        public int NumberOfThreadsHint = 8;

        [Tooltip("The type of simulation to be used when simulating a physics world.")]
        public SimulationType SimulationType = SimulationType.Default;

        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity,
                new PhysicsStepSettings
                {
                    Gravity = Gravity,
                    NumberOfThreadsHint = NumberOfThreadsHint,
                    SimulationType = SimulationType
                }
            );
        }
    }
}
