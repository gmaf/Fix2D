using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace CCC.Fix2D.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class BoxColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.BoxCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        var lossyScale = new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);
                        var size = collider.size;
                    
                        var geometry = new BoxGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.offset)).xy,
                            Size = new float2(size.x * lossyScale.x, size.y * lossyScale.y),
                            Angle = PhysicsMath.ZRotationFromQuaternion(localToWorld.rotation),
                            BevelRadius = math.max(collider.edgeRadius, PhysicsSettings.Constants.MinimumConvexRadius),
                        };

                        geometry.Validate();

                        var colliderBlob = Collider.Create(
                                geometry,
                                ConversionUtilities.GetCollisionFilterFromCollider(collider),
                                ConversionUtilities.GetPhysicsMaterialFromCollider(collider)
                                );

                        // Submit the collider for conversion.
                        m_ColliderConversionSystem.SubmitCollider(collider, ref colliderBlob);
                    }
                    catch(ArgumentException exception)
                    {
                        UnityEngine.Debug.LogWarning($"{collider.name}: {exception.Message}", collider);
                    }
                }
            });

            Entities.ForEach((PhysicsBodyAuth collider) =>
            {
                // Convert the collider if it's valid.
                if(ConversionUtilities.CanConvertCollider(collider) && collider.Shape == PhysicsBodyAuth.ColliderShape.Box)
                {
                    try
                    {
                        var localScale = new float3(collider.transform.localScale).xy;
                        if (math.any(!math.isfinite(localScale)) || math.any(localScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = Matrix4x4.identity; //ConversionUtilities.GetColliderLocalToWorld(collider);
                        var size = collider.BoxSize;

                        var geometry = new BoxGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.BoxAndCircleOffset)).xy,
                            Size = new float2(size.x * localScale.x, size.y * localScale.y),
                            Angle = PhysicsMath.ZRotationFromQuaternion(localToWorld.rotation),
                            BevelRadius = PhysicsSettings.Constants.MinimumConvexRadius /*math.max(collider.edgeRadius, PhysicsSettings.Constants.MinimumConvexRadius)*/,
                        };

                        geometry.Validate();

                        var colliderBlob = Collider.Create(
                                geometry,
                                ConversionUtilities.GetCollisionFilterFromCollider(collider),
                                ConversionUtilities.GetPhysicsMaterialFromCollider(collider)
                                );

                        // Submit the collider for conversion.
                        m_ColliderConversionSystem.SubmitCollider(collider, ref colliderBlob);
                    }
                    catch (ArgumentException exception)
                    {
                        UnityEngine.Debug.LogWarning($"{collider.name}: {exception.Message}", collider);
                    }
                }
            });
        }


    }
}
