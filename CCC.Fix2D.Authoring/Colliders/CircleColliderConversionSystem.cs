using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace CCC.Fix2D.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class CircleColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.CircleCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        var lossyScale =new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);

                        var geometry = new CircleGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.offset)).xy,
                            Radius = math.clamp(collider.radius *  math.cmax(lossyScale), ConversionUtilities.MinRangeClamp, ConversionUtilities.MaxRangeClamp),
                        };

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
                if (ConversionUtilities.CanConvertCollider(collider) && collider.Shape == PhysicsBodyAuth.ColliderShape.Circle)
                {
                    try
                    {
                        var localScale = new float3(collider.transform.localScale).xy;
                        if (math.any(!math.isfinite(localScale)) || math.any(localScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = Matrix4x4.identity; //ConversionUtilities.GetColliderLocalToWorld(collider);

                        var geometry = new CircleGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.BoxAndCircleOffset)).xy,
                            Radius = math.clamp(collider.CircleRadius * math.cmax(localScale), ConversionUtilities.MinRangeClamp, ConversionUtilities.MaxRangeClamp),
                        };

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
