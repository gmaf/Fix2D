using System;

using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using NUnit.Framework;
using static CCC.Fix2D.PhysicsMath;
using UnityEngine;
using Unity.Entities;

namespace CCC.Fix2D.Tests
{
    class ManifoldTests
    {
        [Test]
        public void TestFindMaxSeparation()
        {
            var geometryA = new BoxGeometry
            {
                Size = new float2(1, 1),
                Center = new float2(0, 0),
                Angle = 0f,
                BevelRadius = 0f
            };
            var geometryB = new BoxGeometry
            {
                Size = new float2(1, 1),
                Center = new float2(0, 0),
                Angle = 0,
                BevelRadius = 0f
            };

            PhysicsTransform transformA = new PhysicsTransform(translation: new float2(10, 0), rotation: 0);
            PhysicsTransform transformB = new PhysicsTransform(translation: new float2(15, 0.5f), rotation: 0);


            BlobAssetReference<Collider> colliderBlobA = Collider.Create(geometryA);
            BlobAssetReference<Collider> colliderBlobB = Collider.Create(geometryB);
            ref ConvexHull hullA = ref colliderBlobA.Value.m_ConvexHull;
            ref ConvexHull hullB = ref colliderBlobB.Value.m_ConvexHull;
            int edgeIndex = 0;

            PhysicsTransform aFromB = mul(inverse(transformA), transformB);

            float separation = ConvexConvexManifoldQueries.b2FindMaxSeparation(ref edgeIndex, ref hullB, ref hullA, aFromB);

            Debug.Log($"Max separation: {separation} edge:{edgeIndex}");

            colliderBlobA.Dispose();
            colliderBlobB.Dispose();
        }
        
        [Test]
        public void TestConvexConvex()
        {
            var geometryA = new BoxGeometry
            {
                Size = new float2(1, 1),
                Center = new float2(0, 0),
                Angle = 0f,
                BevelRadius = 0f
            };
            var geometryB = new BoxGeometry
            {
                Size = new float2(1, 1),
                Center = new float2(0, 0),
                Angle = 0,
                BevelRadius = 0f
            };

            PhysicsTransform transformA = new PhysicsTransform(translation: new float2(10, 0), rotation: 0);
            PhysicsTransform transformB = new PhysicsTransform(translation: new float2(11.2f, 0.5f), rotation: 0);
            BlobAssetReference<Collider> colliderBlobA = Collider.Create(geometryA);
            BlobAssetReference<Collider> colliderBlobB = Collider.Create(geometryB);
            ref ConvexHull hullA = ref colliderBlobA.Value.m_ConvexHull;
            ref ConvexHull hullB = ref colliderBlobB.Value.m_ConvexHull;

            PhysicsTransform aFromB = mul(inverse(transformA), transformB);

            ConvexConvexManifoldQueries.ConvexConvex(ref hullA, ref hullB, transformA, aFromB, 0, out ConvexConvexManifoldQueries.Manifold manifold);

            string s = $"Manifold normal: {manifold.Normal}   numContacts:{manifold.NumContacts}";
            for (int i = 0; i < manifold.NumContacts; i++)
            {
                s += $"\ncontact point[{i}]: {manifold[i].Position}  distance:{manifold[i].Distance}";
            }
            Debug.Log(s);

            colliderBlobA.Dispose();
            colliderBlobB.Dispose();
        }
    }
}
