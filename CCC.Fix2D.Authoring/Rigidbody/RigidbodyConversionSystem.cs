using Unity.Entities;

namespace CCC.Fix2D.Authoring
{
    [UpdateInGroup(typeof(GameObjectConversionGroup))]
    [ConverterVersion("2d", 1)]
    internal class RigidbodyConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.Rigidbody2D rigidbody) =>
            {
                // We don't convert a Rigidbody2D if it's not Simulated.
                if (!rigidbody.simulated)
                    return;

                var entity = GetPrimaryEntity(rigidbody);

                // Create any colliders associated with this rigidbody entity.
                m_ColliderConversionSystem.CreateCollider(entity);

                var bodyType = rigidbody.bodyType;

                // There's no components to define a Static rigidbody or its properties.
                if (bodyType != UnityEngine.RigidbodyType2D.Static)
                {
                    // Velocity.
                    if (!DstEntityManager.HasComponent<PhysicsVelocity>(entity))
                    {
                        DstEntityManager.AddComponentData(entity,
                            new PhysicsVelocity
                            {
                                LinearFloat = rigidbody.velocity,
                                AngularFloat = rigidbody.angularVelocity
                            });
                    }

                    var massProperties = MassProperties.Default;

                    // Fetch mass properties from any available collider.
                    if (DstEntityManager.HasComponent<PhysicsColliderBlob>(entity))
                    {
                        var collider = DstEntityManager.GetComponentData<PhysicsColliderBlob>(entity).Collider;
                        massProperties = collider.IsCreated ? collider.Value.MassProperties : MassProperties.Default;
                    }

                    // Dynamic.
                    if (bodyType == UnityEngine.RigidbodyType2D.Dynamic)
                    {
                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateDynamic(massProperties, rigidbody.mass));

                        if (!DstEntityManager.HasComponent<PhysicsGravity>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsGravity { Scale = rigidbody.gravityScale });

                        if (!DstEntityManager.HasComponent<PhysicsDamping>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsDamping
                                {
                                    Linear = rigidbody.drag,
                                    Angular = rigidbody.angularDrag
                                });
                    }
                    // Kinematic.
                    else
                    {
                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateKinematic(massProperties));
                    }
                }
            });

            Entities.ForEach((PhysicsBodyAuth rigidbody) =>
            {
                if (rigidbody.transform.parent != null)
                    throw new System.Exception("PhysicsBodies cannot be have parent gameobjects");

                var entity = GetPrimaryEntity(rigidbody);

                // Create any colliders associated with this rigidbody entity.
                m_ColliderConversionSystem.CreateCollider(entity);

                PhysicsBodyAuth.BodyType bodyType = rigidbody.Type;

                // There's no components to define a Static rigidbody or its properties.
                if (bodyType != PhysicsBodyAuth.BodyType.Static)
                {
                    // Velocity.
                    if (!DstEntityManager.HasComponent<PhysicsVelocity>(entity))
                    {
                        DstEntityManager.AddComponentData(entity, PhysicsVelocity.Zero);
                    }

                    var massProperties = MassProperties.Default;

                    // Fetch mass properties from any available collider.
                    if (DstEntityManager.HasComponent<PhysicsColliderBlob>(entity))
                    {
                        var collider = DstEntityManager.GetComponentData<PhysicsColliderBlob>(entity).Collider;
                        massProperties = collider.IsCreated ? collider.Value.MassProperties : MassProperties.Default;
                    }

                    // Dynamic.
                    if (bodyType == PhysicsBodyAuth.BodyType.Dynamic)
                    {
                        var materialAuth = rigidbody.Material;
                        float colliderArea = rigidbody.CalculateColliderArea();
                        float massDensity = materialAuth != null ? materialAuth.MassDensity : 1f;

                        if (rigidbody.FreezeRotation)
                        {
                            massProperties.MassDistribution.InertiaTensor = 0;
                        }

                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateDynamic(massProperties, mass: massDensity * colliderArea));

                        if (!DstEntityManager.HasComponent<PhysicsGravity>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsGravity { Scale = rigidbody.GravityScale });

                        if (!DstEntityManager.HasComponent<PhysicsDamping>(entity))
                            DstEntityManager.AddComponentData(entity,
                                new PhysicsDamping
                                {
                                    Linear = rigidbody.LinearDrag,
                                    Angular = rigidbody.AngularDrag
                                });
                    }
                    // Kinematic.
                    else
                    {
                        DstEntityManager.AddOrSetComponent(entity,
                            PhysicsMass.CreateKinematic(massProperties));
                    }
                }
            });
        }
    }
}
