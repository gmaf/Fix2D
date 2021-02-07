using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using CCC.Fix2D.Debugging;
using static CCC.Fix2D.PhysicsMath;
using static Unity.Mathematics.math;

[assembly: InternalsVisibleTo("Assembly-CSharp")]

namespace CCC.Fix2D
{
    // Low level convex-convex contact manifold query implementations
    internal static class ConvexConvexManifoldQueries
    {
        // The output of convex-convex manifold queries
        public unsafe struct Manifold
        {
            public int NumContacts;
            public float2 Normal;

            public const int k_MaxNumContacts = 32;
            private fixed float m_ContactPositions[k_MaxNumContacts * 2];
            private fixed float m_Distances[k_MaxNumContacts];

            // Create a single point manifold from a distance query result
            public Manifold(DistanceQueries.Result convexDistance, PhysicsTransform worldFromA)
            {
                NumContacts = 1;
                Normal = mul(worldFromA.Rotation, convexDistance.NormalInA);
                this[0] = new ContactPoint
                {
                    Distance = convexDistance.Distance,
                    Position = mul(worldFromA, convexDistance.PositionOnBinA)
                };
            }

            public ContactPoint this[int contactIndex]
            {
                get
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 2;
                    var contact = new ContactPoint();

                    fixed (float* positions = m_ContactPositions)
                    {
                        contact.Position = *(float2*)(positions + offset);
                    }

                    fixed (float* distances = m_Distances)
                    {
                        contact.Distance = distances[contactIndex];
                    }

                    return contact;
                }
                set
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 2;
                    fixed (float* positions = m_ContactPositions)
                    {
                        *(float2*)(positions + offset) = value.Position;
                    }

                    fixed (float* distances = m_Distances)
                    {
                        distances[contactIndex] = value.Distance;
                    }
                }
            }

            public void Flip()
            {
                for (int i = 0; i < NumContacts; i++)
                {
                    ContactPoint contact = this[i];
                    contact.Position += Normal * contact.Distance;
                    this[i] = contact;
                }
                Normal = -Normal;
            }
        }

        #region Convex vs convex

        // Create a contact point for a pair of spheres in world space.
        public static unsafe void SphereSphere(
            Collider* sphereA, Collider* sphereB,
            [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB, float maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.SphereSphere(sphereA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a box and a sphere in world space.
        public static unsafe void BoxSphere(
            [NoAlias] Collider* boxA, [NoAlias] Collider* sphereB,
            [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB, float maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.BoxSphere(boxA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of boxes in world space.
        public static unsafe void BoxBox(
            Collider* boxA, Collider* boxB,
            [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB, float maxDistance,
            [NoAlias] out Manifold manifold)
        {
            ConvexConvex(ref boxA->m_ConvexHull, ref boxB->m_ConvexHull, worldFromA, aFromB, maxDistance, out manifold);
            //manifold = new Manifold();

            /*
            // Get transforms with box center at origin
            PhysicsTransform bFromBoxB = new PhysicsTransform(boxB->Center, boxB->Angle);
            PhysicsTransform aFromBoxA = new PhysicsTransform(boxA->Center, boxA->Angle);
            PhysicsTransform boxAFromBoxB = mul(inverse(aFromBoxA), mul(aFromB, bFromBoxB));
            PhysicsTransform boxBFromBoxA = inverse(boxAFromBoxB);

            float2 halfExtentsA = boxA->Size * 0.5f;
            float2 halfExtentsB = boxB->Size * 0.5f;

            // Test planes of each box against the other's vertices
            float2 normal; // in BoxA-space
            float distance;
            {
                float2 normalA = new float2(1, 0);
                float2 normalB = new float2(1, 0);
                float distA = 0.0f;
                float distB = 0.0f;
                if (!PointPlanes(boxAFromBoxB, halfExtentsA, halfExtentsB, maxDistance, ref normalA, ref distA) ||
                    !PointPlanes(boxBFromBoxA, halfExtentsB, halfExtentsA, maxDistance, ref normalB, ref distB))
                {
                    return;
                }

                normalB = math.mul(boxAFromBoxB.Rotation, normalB);
                bool aGreater = distA > distB;
                normal = math.select(-normalB, normalA, (bool2)aGreater);
                distance = math.select(distB, distA, aGreater);
            }

            // Test edge pairs
            {
                float2 edgeA = new float2(1.0f, 0.0f);
                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        float2 edgeB;
                        switch (j)
                        {
                            case 0: edgeB = boxAFromBoxB.Rotation.c0; break;
                            case 1: edgeB = boxAFromBoxB.Rotation.c1; break;
                            case 2: edgeB = boxAFromBoxB.Rotation.c2; break;
                            default: edgeB = new float2(0.0f); break;
                        }
                        float2 dir = math.cross(edgeA, edgeB);

                        // hack around parallel edges
                        if (math.all(math.abs(dir) < new float2(1e-5f)))
                        {
                            continue;
                        }

                        float2 edgeNormal = math.normalize(dir);
                        float2 supportA = halfExtentsA;
                        float maxA = math.abs(math.dot(supportA, edgeNormal));
                        float minA = -maxA;
                        float2 supportBinB = halfExtentsB;
                        float2 supportB = math.mul(boxAFromBoxB.Rotation, supportBinB);
                        float offsetB = math.abs(math.dot(supportB, edgeNormal));
                        float centerB = math.dot(boxAFromBoxB.Translation, edgeNormal);
                        float maxB = centerB + offsetB;
                        float minB = centerB - offsetB;

                        float2 diffs = new float2(minB - maxA, minA - maxB); // positive normal, negative normal
                        if (math.all(diffs > new float2(maxDistance)))
                        {
                            return;
                        }

                        if (diffs.x > distance)
                        {
                            distance = diffs.x;
                            normal = -edgeNormal;
                        }

                        if (diffs.y > distance)
                        {
                            distance = diffs.y;
                            normal = edgeNormal;
                        }
                    }

                    edgeA = edgeA.zxy;
                }
            }

            if (distance < maxDistance)
            {
                // Get the normal and supporting faces
                float3 normalInA = math.mul(boxA->Orientation, normal);
                manifold.Normal = math.mul(worldFromA.Rotation, normalInA);
                int faceIndexA = boxA->ConvexHull.GetSupportingFace(-normalInA);
                int faceIndexB = boxB->ConvexHull.GetSupportingFace(math.mul(math.transpose(aFromB.Rotation), normalInA));

                // Build manifold
                if (!FaceFace(ref boxA->ConvexHull, ref boxB->ConvexHull, faceIndexA, faceIndexB, worldFromA, aFromB, normalInA, distance, ref manifold))
                {
                    // The closest points are vertices, we need GJK to find them
                    ConvexConvex(
                        ref ((ConvexCollider*)boxA)->ConvexHull, ref ((ConvexCollider*)boxB)->ConvexHull,
                        worldFromA, aFromB, maxDistance, out manifold);
                }
            }*/
        }

        // Create a single point manifold between a capsule and a sphere in world space.
        public static unsafe void CapsuleSphere(
            [NoAlias] Collider* capsuleA, [NoAlias] Collider* sphereB,
            [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB, float maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleSphere(
                capsuleA->Vertex0, capsuleA->Vertex1, capsuleA->Radius, sphereB->Center, sphereB->Radius, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a pair of capsules in world space.
        public static unsafe void CapsuleCapsule(
            Collider* capsuleA, Collider* capsuleB,
            [NoAlias] PhysicsTransform worldFromA, [NoAlias] PhysicsTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            // TODO: Should produce a multi-point manifold
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleCapsule(capsuleA, capsuleB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of generic convex hulls in world space.
        public static unsafe void ConvexConvex(
            ref ConvexHull hullA, ref ConvexHull hullB,
            [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB, float maxDistance,
            [NoAlias] out Manifold manifold)
        {

            manifold = new Manifold();
            float totalRadius = hullA.ConvexRadius + hullB.ConvexRadius;

            PhysicsTransform bFromA = inverse(aFromB);

            int edgeA = 0;
            float separationA = b2FindMaxSeparation(ref edgeA, ref hullA, ref hullB, bFromA); // potential flip
            if (separationA > totalRadius + maxDistance)
                return;

            int edgeB = 0;
            float separationB = b2FindMaxSeparation(ref edgeB, ref hullB, ref hullA, aFromB); // potential flip
            if (separationB > totalRadius + maxDistance)
                return;

            var worldFromB = mul(worldFromA, aFromB);

            const float k_tol = 0.1f * (0.005f /*b2_linearSlop*/);

            if (separationB > separationA + k_tol)
            {

                // type: faceB
                ConvexConvexPart2(
                    poly1: ref hullB,
                    poly2: ref hullA,
                    xf1: worldFromB,
                    xf2: worldFromA,
                    edge1: edgeB,
                    totalRadius: totalRadius,
                    maxDistance: maxDistance,
                    ref manifold);
            }
            else
            {
                // type: faceA
                ConvexConvexPart2(
                    poly1: ref hullA,
                    poly2: ref hullB,
                    xf1: worldFromA,
                    xf2: worldFromB,
                    edge1: edgeA,
                    totalRadius: totalRadius,
                    maxDistance: maxDistance,
                    ref manifold);


                manifold.Flip();
            }
            //DistanceQueries.BoxSphere

            /*
            // Get closest points on the hulls
            ConvexConvexDistanceQueries.Result result = ConvexConvexDistanceQueries.ConvexConvex(
                hullA.VerticesPtr, hullA.NumVertices, hullB.VerticesPtr, hullB.NumVertices, aFromB, ConvexConvexDistanceQueries.PenetrationHandling.Exact3D);

            float sumRadii = hullB.ConvexRadius + hullA.ConvexRadius;
            if (result.ClosestPoints.Distance < maxDistance + sumRadii)
            {
                float3 normal = result.ClosestPoints.NormalInA;

                manifold = new Manifold
                {
                    Normal = math.mul(worldFromA.Rotation, normal)
                };

                if (hullA.NumFaces > 0)
                {
                    int faceIndexA = hullA.GetSupportingFace(-normal, result.SimplexVertexA(0));
                    if (hullB.NumFaces > 0)
                    {
                        // Convex vs convex
                        int faceIndexB = hullB.GetSupportingFace(math.mul(math.transpose(aFromB.Rotation), normal), result.SimplexVertexB(0));
                        if (FaceFace(ref hullA, ref hullB, faceIndexA, faceIndexB, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    }
                    else if (hullB.NumVertices == 2)
                    {
                        // Convex vs capsule
                        if (FaceEdge(ref hullA, ref hullB, faceIndexA, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    } // Else convex vs sphere
                }
                else if (hullA.NumVertices == 2)
                {
                    if (hullB.NumFaces > 0)
                    {
                        // Capsule vs convex
                        manifold.Normal = math.mul(worldFromA.Rotation, -normal); // negate the normal because we are temporarily flipping to triangle A capsule B
                        PhysicsTransform worldFromB = Mul(worldFromA, aFromB);
                        PhysicsTransform bFromA = Inverse(aFromB);
                        float3 normalInB = math.mul(bFromA.Rotation, normal);
                        int faceIndexB = hullB.GetSupportingFace(normalInB, result.SimplexVertexB(0));
                        bool foundClosestPoint = FaceEdge(ref hullB, ref hullA, faceIndexB, worldFromB, bFromA, -normalInB, result.ClosestPoints.Distance, ref manifold);
                        manifold.Flip();
                        if (foundClosestPoint)
                        {
                            return;
                        }
                    } // Else capsule vs capsule or sphere
                } // Else sphere vs something

                // Either one of the shapes is a sphere, or both of the shapes are capsules, or both of the closest features are nearly perpendicular to the contact normal,
                // or FaceFace()/FaceEdge() missed the closest point due to numerical error.  In these cases, add the closest point directly to the manifold.
                if (manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    DistanceQueries.Result convexDistance = result.ClosestPoints;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, convexDistance.PositionOnAinA) - manifold.Normal * (convexDistance.Distance - hullB.ConvexRadius),
                        Distance = convexDistance.Distance - sumRadii
                    };
                }
            }
            else
            {
                manifold = new Manifold();
            } */
        }

        private unsafe static void ConvexConvexPart2(
            ref ConvexHull poly1,
            ref ConvexHull poly2,
            [NoAlias] in PhysicsTransform xf1,
            [NoAlias] in PhysicsTransform xf2,
            int edge1,
            float totalRadius,
            float maxDistance,
            ref Manifold manifold)
        {
            b2ClipVertex* incidentEdge = stackalloc b2ClipVertex[2];
            b2FindIncidentEdge(incidentEdge, ref poly1, xf1, edge1, ref poly2, xf2);

            int iv1 = edge1;
            int iv2 = edge1 + 1 < poly1.Length ? edge1 + 1 : 0;

            float2 v11 = poly1.Vertices[iv1];
            float2 v12 = poly1.Vertices[iv2];

            float2 localTangent = math.normalize(v12 - v11);

            float2 localNormal = cross(localTangent, 1.0f);
            float2 localPlanePoint = 0.5f * (v11 + v12);

            float2 tangent = mul(xf1.Rotation, localTangent);
            float2 normal = cross(tangent, 1.0f);

            v11 = mul(xf1, v11);
            v12 = mul(xf1, v12);

            // Face offset.
            float frontOffset = dot(normal, v11);

            // Side offsets, extended by polytope skin thickness.
            float sideOffset1 = -dot(tangent, v11) + totalRadius;
            float sideOffset2 = dot(tangent, v12) + totalRadius;

            // Clip incident edge against extruded edge1 side edges.
            b2ClipVertex* clipPoints1 = stackalloc b2ClipVertex[2];
            b2ClipVertex* clipPoints2 = stackalloc b2ClipVertex[2];
            int np;

            // Clip to box side 1
            np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

            if (np < 2)
                return;

            // Clip to negative box side 1
            np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

            if (np < 2)
            {
                return;
            }

            // Now clipPoints2 contains the clipped points.
            manifold.Normal = mul(xf1.Rotation, localNormal);
            float2 worldPlanePoint = mul(xf1, localPlanePoint);

            int pointCount = 0;
            for (int i = 0; i < 2; ++i)
            {
                float separation = dot(normal, clipPoints2[i].v) - frontOffset;

                if (separation <= totalRadius + maxDistance)
                {
                    float2 clipPoint = clipPoints2[i].v;

                    float2 cA = clipPoint + (/*poly1.ConvexRadius*/0f - dot(clipPoint - worldPlanePoint, normal)) * normal;
                    float2 cB = clipPoint - 0f/*poly2.ConvexRadius * normal*/;

                    ContactPoint cp = new ContactPoint()
                    {
                        Position = 0.5f * (cA + cB),
                        Distance = dot(cB - cA, normal)
                    };

                    manifold[pointCount] = cp;
                    ++pointCount;

                    /*
                    ContactPoint cp = new ContactPoint()
                    {
                        Position = clipPoints2[i].v,
                        Distance = separation - totalRadius
                    };

                    manifold[pointCount] = cp;

                    */
                    /*
                    b2ManifoldPoint* cp = manifold->points + pointCount;
                    cp->localPoint = b2MulT(xf2, clipPoints2[i].v);
                    cp->id = clipPoints2[i].id;
                    if (flip)
                    {
                        // Swap features
                        b2ContactFeature cf = cp->id.cf;
                        cp->id.cf.indexA = cf.indexB;
                        cp->id.cf.indexB = cf.indexA;
                        cp->id.cf.typeA = cf.typeB;
                        cp->id.cf.typeB = cf.typeA;
                    }*/
                }
            }

            manifold.NumContacts = pointCount;

            //for (int i = 0; i < manifold.NumContacts; i++)
            //{
            //    manifold[manifold.NumContacts + i] = manifold[i];
            //}
            //manifold.NumContacts *= 2;
        }

        private struct b2ClipVertex
        {
            public float2 v;
            public int indexA;
            public int indexB;
        }

        private unsafe static void b2FindIncidentEdge(b2ClipVertex* c,
                             ref ConvexHull poly1, [NoAlias] in PhysicsTransform xf1, int edge1,
                             ref ConvexHull poly2, [NoAlias] in PhysicsTransform xf2)
        {
            Debug.Assert(0 <= edge1 && edge1 < poly1.Length);

            // Get the normal of the reference edge in poly2's frame.
            float2 normal1 = mul(inverse(xf2.Rotation), mul(xf1.Rotation, poly1.Normals[edge1]));

            // Find the incident edge on poly2.
            int index = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < poly2.Length; ++i)
            {
                float dot = math.dot(normal1, poly2.Normals[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            // Build the clip vertices for the incident edge.
            int i1 = index;
            int i2 = i1 + 1 < poly2.Length ? i1 + 1 : 0;

            c[0].v = mul(xf2, poly2.Vertices[i1]);
            c[0].indexA = edge1;
            c[0].indexB = i1;
            //c0.typeA = b2ContactFeature::e_face;
            //c0.typeB = b2ContactFeature::e_vertex;

            c[1].v = mul(xf2, poly2.Vertices[i2]);
            c[1].indexA = edge1;
            c[1].indexB = i2;
            //c1.id.cf.typeA = b2ContactFeature::e_face;
            //c1.id.cf.typeB = b2ContactFeature::e_vertex;
        }


        // Sutherland-Hodgman clipping.
        unsafe static int b2ClipSegmentToLine(b2ClipVertex* vOut/*[2]*/,
                                b2ClipVertex* vIn/*[2]*/,
                                in float2 normal, float offset, int vertexIndexA)
        {
            // Start with no output points
            int count = 0;

            // Calculate the distance of end points to the line
            float distance0 = math.dot(normal, vIn[0].v) - offset;
            float distance1 = math.dot(normal, vIn[1].v) - offset;

            // If the points are behind the plane
            if (distance0 <= 0.0f)
                vOut[count++] = vIn[0];

            if (distance1 <= 0.0f)
                vOut[count++] = vIn[1];

            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f)
            {
                // Find intersection point of edge and plane
                float interp = distance0 / (distance0 - distance1);
                vOut[count].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

                // VertexA is hitting edgeB.
                vOut[count].indexA = vertexIndexA;
                vOut[count].indexB = vIn[0].indexB;
                //vOut[count].typeA = b2ContactFeature::e_vertex;
                //vOut[count].typeB = b2ContactFeature::e_face;
                ++count;

                Assert.IsTrue(count == 2);
            }

            return count;
        }

        /// <summary>
        /// Find the max separation between poly1 and poly2 using edge normals from poly1.
        /// </summary>
        internal static float b2FindMaxSeparation(ref int edgeIndex1, ref ConvexHull poly1, ref ConvexHull poly2, [NoAlias] in PhysicsTransform twoFromOne)
        {
            int count1 = poly1.Length;
            int count2 = poly2.Length;

            int bestIndex = 0;
            float maxSeparation = float.MinValue;
            for (int i = 0; i < count1; ++i)
            {
                // Get poly1 normal in frame2.
                float2 n = mul(twoFromOne.Rotation, poly1.Normals[i]);
                float2 v1 = mul(twoFromOne, poly1.Vertices[i]);

                // Find deepest point for normal i.
                float si = float.MaxValue;
                for (int j = 0; j < count2; ++j)
                {
                    float sij = dot(n, poly2.Vertices[j] - v1);
                    if (sij < si)
                    {
                        si = sij;
                    }
                }

                if (si > maxSeparation)
                {
                    maxSeparation = si;
                    bestIndex = i;
                }
            }

            edgeIndex1 = bestIndex;
            return maxSeparation;
        }

        #endregion
        /*
        #region Helpers

        // BoxBox() helper
        private static bool PointPlanes(PhysicsTransform aFromB, float2 halfExtA, float2 halfExtB, float maxDistance, ref float2 normalOut, ref float distanceOut)
        {
            // Calculate the AABB of box B in A-space
            Aabb aabbBinA;
            {
                Aabb aabbBinB = new Aabb { Min = -halfExtB, Max = halfExtB };
                aabbBinA = mul(aFromB, aabbBinB);
            }

            // Check for a miss
            float2 toleranceHalfExt = halfExtA + maxDistance;
            bool2 miss = (aabbBinA.Min > toleranceHalfExt) | (-toleranceHalfExt > aabbBinA.Max);
            if (math.any(miss))
            {
                return false;
            }

            // Return the normal with minimum separating distance
            float2 diff0 = aabbBinA.Min - halfExtA; // positive normal
            float2 diff1 = -aabbBinA.Max - halfExtA; // negative normal
            bool2 greater01 = diff0 > diff1;
            float2 max01 = math.select(diff1, diff0, greater01);
            distanceOut = math.cmax(max01);

            int axis = IndexOfMaxComponent(max01);
            if (axis == 0)
            {
                normalOut = new float2(1.0f, 0.0f);
            }
            else
            {
                normalOut = new float2(0.0f, 1.0f);
            }
            normalOut = math.select(normalOut, -normalOut, greater01);

            return true;
        }

        // returns the argument with greater w component
        private static float4 SelectMaxW(float4 a, float4 b)
        {
            return math.select(b, a, a.w > b.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CalcTrianglePlanes(float3 v0, float3 v1, float3 v2, float3 normalDirection,
            [NoAlias] out FourTransposedPoints verts, [NoAlias] out FourTransposedPoints edges, [NoAlias] out FourTransposedPoints perps)
        {
            verts = new FourTransposedPoints(v0, v1, v2, v0);
            edges = verts.V1230 - verts;
            perps = edges.Cross(new FourTransposedPoints(normalDirection));
        }

        #endregion

        #region Multiple contact generation

        // Iterates over the edges of a face
        private unsafe struct EdgeIterator
        {
            // Current edge
            public float3 Vertex0 { get; private set; }
            public float3 Vertex1 { get; private set; }
            public float3 Edge { get; private set; }
            public float3 Perp { get; private set; }
            public float Offset { get; private set; }
            public int Index { get; private set; }

            // Face description
            private float3* vertices;
            private byte* indices;
            private float3 normal;
            private int count;

            public static unsafe EdgeIterator Begin(float3* vertices, byte* indices, float3 normal, int count)
            {
                EdgeIterator iterator = new EdgeIterator();
                iterator.vertices = vertices;
                iterator.indices = indices;
                iterator.normal = normal;
                iterator.count = count;

                iterator.Vertex1 = (indices == null) ? vertices[count - 1] : vertices[indices[count - 1]];
                iterator.update();
                return iterator;
            }

            public bool Valid()
            {
                return Index < count;
            }

            public void Advance()
            {
                Index++;
                if (Valid())
                {
                    update();
                }
            }

            private void update()
            {
                Vertex0 = Vertex1;
                Vertex1 = (indices == null) ? vertices[Index] : vertices[indices[Index]];

                Edge = Vertex1 - Vertex0;
                Perp = math.cross(Edge, normal); // points outwards from face
                Offset = math.dot(Perp, Vertex1);
            }
        }

        // Cast ray originA, directionA against plane normalB, offsetB and update the ray hit fractions
        private static void castRayPlane(float3 originA, float3 directionA, float3 normalB, float offsetB, ref float fracEnter, ref float fracExit)
        {
            // Cast edge A against plane B
            float start = math.dot(originA, normalB) - offsetB;
            float diff = math.dot(directionA, normalB);
            float end = start + diff;
            float frac = math.select(-start / diff, 0.0f, diff == 0.0f);

            bool startInside = (start <= 0.0f);
            bool endInside = (end <= 0.0f);

            bool enter = !startInside & (frac > fracEnter);
            fracEnter = math.select(fracEnter, frac, enter);

            bool exit = !endInside & (frac < fracExit);
            fracExit = math.select(fracExit, frac, exit);

            bool hit = startInside | endInside;
            fracEnter = math.select(fracExit, fracEnter, hit); // mark invalid with enter <= exit in case of a miss
        }

        // If the rejections of the faces from the contact normal are just barely touching, then FaceFace() might miss the closest points because of numerical error.
        // FaceFace() and FaceEdge() check if they found a point as close as the closest, and if not they return false so that the caller can add it.
        private const float closestDistanceTolerance = 1e-4f;

        // Tries to generate a manifold between a pair of faces.  It can fail in some cases due to numerical accuracy:
        // 1) both faces are nearly perpendicular to the normal
        // 2) the closest features on the shapes are vertices, so that the intersection of the projection of the faces to the plane perpendicular to the normal contains only one point
        // In those cases, FaceFace() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceFace(
            ref ConvexHull convexA, ref ConvexHull convexB, int faceIndexA, int faceIndexB, [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB,
            float3 normal, float distance, [NoAlias] ref Manifold manifold)
        {
            // Get the plane of each face
            Plane planeA = convexA.Planes[faceIndexA];
            Plane planeB = TransformPlane(aFromB, convexB.Planes[faceIndexB]);

            // Handle cases where one of the faces is nearly perpendicular to the contact normal
            // This gets around divide by zero / numerical problems from dividing collider planes which often contain some error by a very small number, amplifying that error
            const float cosMaxAngle = 0.05f;
            float dotA = math.dot(planeA.Normal, normal);
            float dotB = math.dot(planeB.Normal, normal);
            bool acceptB = true; // true if vertices of B projected onto the face of A are accepted
            if (dotA > -cosMaxAngle)
            {
                // Handle cases where both faces are nearly perpendicular to the contact normal.
                if (dotB < cosMaxAngle)
                {
                    // Both faces are nearly perpendicular to the contact normal, let the caller generate a single contact
                    return false;
                }

                // Face of A is nearly perpendicular to the contact normal, don't try to project vertices onto it
                acceptB = false;
            }
            else if (dotB < cosMaxAngle)
            {
                // Face of B is nearly perpendicular to the normal, so we need to clip the edges of B against face A instead
                PhysicsTransform bFromA = Inverse(aFromB);
                float3 normalInB = math.mul(bFromA.Rotation, -normal);
                PhysicsTransform worldFromB = Mul(worldFromA, aFromB);
                bool result = FaceFace(ref convexB, ref convexA, faceIndexB, faceIndexA, worldFromB, bFromA, normalInB, distance, ref manifold);
                manifold.Normal = -manifold.Normal;
                manifold.Flip();
                return result;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Transform vertices of B into A-space
            // Initialize validB, which is true for each vertex of B that is inside face A
            ConvexHull.Face faceA = convexA.Faces[faceIndexA];
            ConvexHull.Face faceB = convexB.Faces[faceIndexB];
            bool* validB = stackalloc bool[faceB.NumVertices];
            float3* verticesBinA = stackalloc float3[faceB.NumVertices];
            {
                byte* indicesB = convexB.FaceVertexIndicesPtr + faceB.FirstIndex;
                float3* verticesB = convexB.VerticesPtr;
                for (int i = 0; i < faceB.NumVertices; i++)
                {
                    validB[i] = acceptB;
                    verticesBinA[i] = Mul(aFromB, verticesB[indicesB[i]]);
                }
            }

            // For each edge of A
            float invDotB = math.rcp(dotB);
            float sumRadii = convexA.ConvexRadius + convexB.ConvexRadius;
            byte* indicesA = convexA.FaceVertexIndicesPtr + faceA.FirstIndex;
            float3* verticesA = convexA.VerticesPtr;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                float fracEnterA = 0.0f;
                float fracExitA = 1.0f;

                // For each edge of B
                for (EdgeIterator edgeB = EdgeIterator.Begin(verticesBinA, null, normal, faceB.NumVertices); edgeB.Valid(); edgeB.Advance())
                {
                    // Cast edge A against plane B and test if vertex B is inside plane A
                    castRayPlane(edgeA.Vertex0, edgeA.Edge, edgeB.Perp, edgeB.Offset, ref fracEnterA, ref fracExitA);
                    validB[edgeB.Index] &= (math.dot(edgeB.Vertex1, edgeA.Perp) < edgeA.Offset);
                }

                // If edge A hits B, add a contact points
                if (fracEnterA < fracExitA)
                {
                    float distance0 = (math.dot(edgeA.Vertex0, planeB.Normal) + planeB.Distance) * invDotB;
                    float deltaDistance = math.dot(edgeA.Edge, planeB.Normal) * invDotB;
                    float3 vertexAOnB = edgeA.Vertex0 - normal * distance0;
                    float3 edgeAOnB = edgeA.Edge - normal * deltaDistance;
                    foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracEnterA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    if (fracExitA < 1.0f) // If the exit fraction is 1, then the next edge has the same contact point with enter fraction 0
                    {
                        foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracExitA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    }
                }
            }

            // For each vertex of B
            float invDotA = math.rcp(dotA);
            for (int i = 0; i < faceB.NumVertices; i++)
            {
                if (validB[i] && manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    float3 vertexB = verticesBinA[i];
                    float distanceB = (math.dot(vertexB, planeA.Normal) + planeA.Distance) * -invDotA;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, vertexB) + manifold.Normal * convexB.ConvexRadius,
                        Distance = distanceB - sumRadii
                    };
                    foundClosestPoint |= distanceB <= distance;
                }
            }

            return foundClosestPoint;
        }

        // Tries to generate a manifold between a face and an edge.  It can fail for the same reasons as FaceFace().
        // In those cases, FaceEdge() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceEdge(
            ref ConvexHull faceConvexA, ref ConvexHull edgeConvexB, int faceIndexA, [NoAlias] in PhysicsTransform worldFromA, [NoAlias] in PhysicsTransform aFromB,
            float3 normal, float distance, [NoAlias] ref Manifold manifold)
        {
            // Check if the face is nearly perpendicular to the normal
            const float cosMaxAngle = 0.05f;
            Plane planeA = faceConvexA.Planes[faceIndexA];
            float dotA = math.dot(planeA.Normal, normal);
            if (math.abs(dotA) < cosMaxAngle)
            {
                return false;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Get the supporting face on A
            ConvexHull.Face faceA = faceConvexA.Faces[faceIndexA];
            byte* indicesA = faceConvexA.FaceVertexIndicesPtr + faceA.FirstIndex;

            // Get edge in B
            float3 vertexB0 = Math.Mul(aFromB, edgeConvexB.Vertices[0]);
            float3 edgeB = math.mul(aFromB.Rotation, edgeConvexB.Vertices[1] - edgeConvexB.Vertices[0]);

            // For each edge of A
            float3* verticesA = faceConvexA.VerticesPtr;
            float fracEnterB = 0.0f;
            float fracExitB = 1.0f;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                // Cast edge B against plane A
                castRayPlane(vertexB0, edgeB, edgeA.Perp, edgeA.Offset, ref fracEnterB, ref fracExitB);
            }

            // If edge B hits A, add a contact points
            if (fracEnterB < fracExitB)
            {
                float invDotA = math.rcp(dotA);
                float sumRadii = faceConvexA.ConvexRadius + edgeConvexB.ConvexRadius;
                float distance0 = (math.dot(vertexB0, planeA.Normal) + planeA.Distance) * -invDotA;
                float deltaDistance = math.dot(edgeB, planeA.Normal) * -invDotA;
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracEnterB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracExitB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
            }

            return foundClosestPoint;
        }

        // Adds a contact to the manifold from an edge and fraction
        private static bool AddEdgeContact(float3 vertex0, float3 edge, float distance0, float deltaDistance, float fraction, float3 normalInA, float radiusB, float sumRadii,
            [NoAlias] in PhysicsTransform worldFromA, float distanceThreshold, [NoAlias] ref Manifold manifold)
        {
            if (manifold.NumContacts < Manifold.k_MaxNumContacts)
            {
                float3 position = vertex0 + fraction * edge;
                float distance = distance0 + fraction * deltaDistance;

                manifold[manifold.NumContacts++] = new ContactPoint
                {
                    Position = Mul(worldFromA, position + normalInA * radiusB),
                    Distance = distance - sumRadii
                };

                return distance <= distanceThreshold;
            }
            return false;
        }

        #endregion
        */
    }
}
