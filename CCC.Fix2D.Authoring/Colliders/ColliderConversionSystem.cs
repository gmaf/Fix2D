using Unity.Collections;
using Unity.Entities;

namespace CCC.Fix2D.Authoring
{
    [AlwaysUpdateSystem]
    [UpdateInGroup(typeof(GameObjectBeforeConversionGroup))]
    [ConverterVersion("2d", 1)]
    internal sealed class ColliderConversionSystem : GameObjectConversionSystem
    {
        private NativeMultiHashMap<Entity, BlobAssetReference<Collider>> RigidbodyToColliderMapping;

        protected override void OnCreate()
        {
            base.OnCreate();
            RigidbodyToColliderMapping = new NativeMultiHashMap<Entity, BlobAssetReference<Collider>>(256, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            RigidbodyToColliderMapping.Dispose();
            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            RigidbodyToColliderMapping.Clear();
        }

        internal void SubmitCollider(
            UnityEngine.Collider2D collider,
            ref BlobAssetReference<Collider> colliderBlob)
        {
            // Fetch any rigidbody we're attached to.
            var rigidbody = collider.attachedRigidbody;

            // Are we attached to a rigidbody?
            if (rigidbody != null)
            {
                // Yes, so add as a mapping to the rigidbody entity for now in-case we're a compound collider.
                RigidbodyToColliderMapping.Add(GetPrimaryEntity(rigidbody), colliderBlob);

                // Declare a dependency between the rigidbody and the collider components.
                DeclareDependency(rigidbody, collider);
                return;
            }

            // No attached Rigidbody2D so add the collider blob onto this Entity.
            // NOTE: This is the implicit static collider case.
            DstEntityManager.AddComponentData(GetPrimaryEntity(collider), new PhysicsColliderBlob { Collider = colliderBlob });
        }

        internal void SubmitCollider(
            PhysicsBodyAuth collider,
            ref BlobAssetReference<Collider> colliderBlob)
        {
            /*
            // Fetch any rigidbody we're attached to.
            var rigidbody = collider.attachedRigidbody;

            // Are we attached to a rigidbody?
            if (rigidbody != null)
            {
                // Yes, so add as a mapping to the rigidbody entity for now in-case we're a compound collider.
                RigidbodyToColliderMapping.Add(GetPrimaryEntity(rigidbody), colliderBlob);

                // Declare a dependency between the rigidbody and the collider components.
                DeclareDependency(rigidbody, collider);
                return;
            }*/

            // No attached Rigidbody2D so add the collider blob onto this Entity.
            // NOTE: This is the implicit static collider case.
            DstEntityManager.AddComponentData(GetPrimaryEntity(collider), new PhysicsColliderBlob { Collider = colliderBlob });
        }

        internal void CreateCollider(Entity rigidbodyEntity)
        {
            var colliderCount = RigidbodyToColliderMapping.CountValuesForKey(rigidbodyEntity);
            if (colliderCount == 0)
                return;

            // Single collider doesn't require a compound collider.
            if (colliderCount == 1)
            {
                var foundColliderBlob = RigidbodyToColliderMapping.TryGetFirstValue(rigidbodyEntity, out BlobAssetReference<Collider> colliderBlob, out NativeMultiHashMapIterator<Entity> ignore);
                SafetyChecks.IsTrue(foundColliderBlob);

                // Add the single collider to the rigidbody entity.
                DstEntityManager.AddComponentData(rigidbodyEntity, new PhysicsColliderBlob { Collider = colliderBlob });

                return;
            }

            throw new System.Exception("Compound colliders not supported");
        }
    }
}
