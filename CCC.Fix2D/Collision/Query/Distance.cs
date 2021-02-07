// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Miscellaneous parts of the source code below are an adaption of the Box2D library.

using System;
using Unity.Assertions;
using Unity.Entities;
using Unity.Mathematics;
using static CCC.Fix2D.PhysicsMath;

namespace CCC.Fix2D
{
    #region Query Input & Output

    public struct PointDistanceInput : IQueryInput
    {
        public float2 Position;
        public float MaxDistance;
        public CollisionFilter Filter;
        public IgnoreHit Ignore { get; set; }

        internal QueryContext QueryContext;

        public static PointDistanceInput Default =>
            new PointDistanceInput
            {
                MaxDistance = float.MaxValue,
                Filter = CollisionFilter.Default
            };
    }

    public struct ColliderDistanceInput : IQueryInput
    {
        public BlobAssetReference<Collider> Collider;
        public PhysicsTransform Transform;
        public float MaxDistance;
        public IgnoreHit Ignore { get; set; }

        internal QueryContext QueryContext;

        public static ColliderDistanceInput Default =>
            new ColliderDistanceInput
            {
                Transform = PhysicsTransform.Identity,
                MaxDistance = float.MaxValue
            };
    }

    public struct DistanceHit : IQueryResult
    {
        #region IQueryResult

        public int PhysicsBodyIndex { get; internal set; }
        public ColliderKey ColliderKey { get; internal set; }
        public Entity Entity { get; internal set; }
        public float Fraction { get; internal set; }

        public bool IsValid => Entity != Entity.Null;

        #endregion

        public float Distance => Fraction;
        public float2 Direction { get { return PointB - PointA; } }
        public float2 PointA { get; internal set; }
        public float2 PointB { get; internal set; }
    }

    #endregion

    // Distance query implementations
    static class DistanceQueries
    {
        // Distance queries have edge cases where distance = 0, eg. consider choosing the correct normal for a point that is exactly on a triangle surface.
        // Additionally, with floating point numbers there are often numerical accuracy problems near distance = 0.  Some routines handle this with special
        // cases where distance^2 < distanceEpsSq, which is expected to be rare in normal usage.  distanceEpsSq is not an exact value, but chosen to be small
        // enough that at typical simulation scale the difference between distance = distanceEps and distance = 0 is negligible.
        private const float distanceEpsSq = 1e-8f;

        public struct Result
        {
            public float2 PositionOnAinA;
            public float2 NormalInA;
            public float Distance;

            public float2 PositionOnBinA => PositionOnAinA - NormalInA * Distance;
        }

        public static unsafe Result SphereSphere(Collider* sphereA, Collider* sphereB, PhysicsTransform aFromB)
        {
            float2 posBinA = mul(aFromB, sphereB->Center);
            return PointPoint(sphereA->Center, posBinA, sphereA->Radius, sphereA->Radius + sphereB->Radius);
        }
        public static Result PointPoint(float2 pointA, float2 pointB, float radiusA, float sumRadii)
        {
            float2 diff = pointA - pointB;
            float coreDistanceSq = math.lengthsq(diff);
            return PointPoint(pointB, diff, coreDistanceSq, radiusA, sumRadii);
        }

        private static Result PointPoint(float2 pointB, float2 diff, float coreDistanceSq, float radiusA, float sumRadii)
        {
            bool distanceZero = coreDistanceSq == 0.0f;
            float invCoreDistance = math.select(math.rsqrt(coreDistanceSq), 0.0f, distanceZero);
            float2 normal = math.select(diff * invCoreDistance, new float2(0, 1), distanceZero); // choose an arbitrary normal when the distance is zero
            float distance = coreDistanceSq * invCoreDistance;
            return new Result
            {
                NormalInA = normal,
                PositionOnAinA = pointB + normal * (distance - radiusA),
                Distance = distance - sumRadii
            };
        }

        public static unsafe Result BoxSphere(Collider* boxA, Collider* sphereB, PhysicsTransform aFromB)
        {
            PhysicsTransform aFromBoxA = new PhysicsTransform(boxA->Center, boxA->Angle);
            float2 posBinA = mul(aFromB, sphereB->Center);
            float2 posBinBoxA = mul(inverse(aFromBoxA), posBinA);
            float2 innerHalfExtents = boxA->Size * 0.5f - boxA->BevelRadius;
            float2 normalInBoxA;
            float distance;
            {
                // from hkAabb::signedDistanceToPoint(), can optimize a lot
                float2 projection = math.min(posBinBoxA, innerHalfExtents);
                projection = math.max(projection, -innerHalfExtents);
                float2 difference = projection - posBinBoxA;
                float distanceSquared = math.lengthsq(difference);

                // Check if the sphere center is inside the box
                if (distanceSquared < 1e-6f)
                {
                    float2 projectionLocal = projection;
                    float2 absProjectionLocal = math.abs(projectionLocal);
                    float2 del = absProjectionLocal - innerHalfExtents;
                    int axis = IndexOfMaxComponent(new float3(del, -float.MaxValue));
                    switch (axis)
                    {
                        case 0: normalInBoxA = new float2(projectionLocal.x < 0.0f ? 1.0f : -1.0f, 0.0f); break;
                        case 1: normalInBoxA = new float2(0.0f, projectionLocal.y < 0.0f ? 1.0f : -1.0f); break;
                        default:
                            normalInBoxA = new float2(1, 0);
                            Assert.IsTrue(false);
                            break;
                    }
                    distance = math.max(del.x, del.y);
                }
                else
                {
                    float invDistance = math.rsqrt(distanceSquared);
                    normalInBoxA = difference * invDistance;
                    distance = distanceSquared * invDistance;
                }
            }

            float2 normalInA = math.mul(aFromBoxA.Rotation, normalInBoxA);
            return new Result
            {
                NormalInA = normalInA,
                PositionOnAinA = posBinA + normalInA * (distance - boxA->BevelRadius),
                Distance = distance - (sphereB->Radius + boxA->BevelRadius)
            };
        }

        public static Result CapsuleSphere(
            float2 capsuleVertex0, float2 capsuleVertex1, float capsuleRadius,
            float2 sphereCenter, float sphereRadius,
            PhysicsTransform aFromB)
        {
            // Transform the sphere into capsule space
            float2 centerB = mul(aFromB, sphereCenter);

            // Point-segment distance
            float2 edgeA = capsuleVertex1 - capsuleVertex0;
            float dot = math.dot(edgeA, centerB - capsuleVertex0);
            float edgeLengthSquared = math.lengthsq(edgeA);
            dot = math.max(dot, 0.0f);
            dot = math.min(dot, edgeLengthSquared);
            float invEdgeLengthSquared = 1.0f / edgeLengthSquared;
            float frac = dot * invEdgeLengthSquared;
            float2 pointOnA = capsuleVertex0 + edgeA * frac;
            return PointPoint(pointOnA, centerB, capsuleRadius, capsuleRadius + sphereRadius);
        }

        // Find the closest points on a pair of line segments
        private static void SegmentSegment(float2 pointA, float2 edgeA, float2 pointB, float2 edgeB, out float2 closestAOut, out float2 closestBOut)
        {
            // Find the closest point on edge A to the line containing edge B
            float2 diff = pointB - pointA;

            float r = math.dot(edgeA, edgeB);
            float s1 = math.dot(edgeA, diff);
            float s2 = math.dot(edgeB, diff);
            float lengthASq = math.lengthsq(edgeA);
            float lengthBSq = math.lengthsq(edgeB);

            float invDenom, invLengthASq, invLengthBSq;
            {
                float denom = lengthASq * lengthBSq - r * r;
                float3 inv = 1.0f / new float3(denom, lengthASq, lengthBSq);
                invDenom = inv.x;
                invLengthASq = inv.y;
                invLengthBSq = inv.z;
            }

            float fracA = (s1 * lengthBSq - s2 * r) * invDenom;
            fracA = math.clamp(fracA, 0.0f, 1.0f);

            // Find the closest point on edge B to the point on A just found
            float fracB = fracA * (invLengthBSq * r) - invLengthBSq * s2;
            fracB = math.clamp(fracB, 0.0f, 1.0f);

            // If the point on B was clamped then there may be a closer point on A to the edge
            fracA = fracB * (invLengthASq * r) + invLengthASq * s1;
            fracA = math.clamp(fracA, 0.0f, 1.0f);

            closestAOut = pointA + fracA * edgeA;
            closestBOut = pointB + fracB * edgeB;
        }

        public static unsafe Result CapsuleCapsule(Collider* capsuleA, Collider* capsuleB, PhysicsTransform aFromB)
        {
            // Transform capsule B into A-space
            float2 pointB = mul(aFromB, capsuleB->Vertex0);
            float2 edgeB = math.mul(aFromB.Rotation, capsuleB->Vertex1 - capsuleB->Vertex0);

            // Get point and edge of A
            float2 pointA = capsuleA->Vertex0;
            float2 edgeA = capsuleA->Vertex1 - capsuleA->Vertex0;

            // Get the closest points on the capsules
            SegmentSegment(pointA, edgeA, pointB, edgeB, out float2 closestA, out float2 closestB);
            //float2 diff = closestA - closestB;
            //float coreDistanceSq = math.lengthsq(diff);
            //if (coreDistanceSq < distanceEpsSq)
            //{
            //    // Special case for extremely small distances, should be rare
            //    float3 normal = math.cross(edgeA, edgeB);
            //    if (math.lengthsq(normal) < 1e-5f)
            //    {
            //        float3 edge = math.normalizesafe(edgeA, math.normalizesafe(edgeB, new float3(1, 0, 0))); // edges are parallel or one of the capsules is a sphere
            //        Math.CalculatePerpendicularNormalized(edge, out normal, out float3 _); // normal is anything perpendicular to edge
            //    }
            //    else
            //    {
            //        normal = math.normalize(normal); // normal is cross of edges, sign doesn't matter
            //    }

            //    return new Result
            //    {
            //        NormalInA = normal,
            //        PositionOnAinA = pointA - normal * capsuleA->Radius,
            //        Distance = -capsuleA->Radius - capsuleB->Radius
            //    };
            //}
            return PointPoint(closestA, closestB, capsuleA->Radius, capsuleA->Radius + capsuleB->Radius);
        }

        #region Intersection tests.

        internal static unsafe DistanceHit ColliderDistance(PhysicsTransform transformA, ref DistanceProxy proxyA, ref DistanceProxy proxyB)
        {
            var simplex = new Simplex();
            simplex.Reset(transformA, proxyA, proxyB);

            var inverseRotationA = math.transpose(transformA.Rotation);

            var vertices = &simplex.Vertex1;

            Simplex.VertexIndexTriple saveA;
            Simplex.VertexIndexTriple saveB;

            var iteration = 0;
            while (iteration < PhysicsSettings.Constants.MaxGJKInterations)
            {
                // Copy simplex so we can identify duplicates.
                var saveCount = simplex.Count;
                for (var i = 0; i < saveCount; ++i)
                {
                    saveA.Index[i] = vertices[i].IndexA;
                    saveB.Index[i] = vertices[i].IndexB;
                }

                switch (saveCount)
                {
                    case 1:
                        break;

                    case 2:
                        simplex.Solve2();
                        break;

                    case 3:
                        simplex.Solve3();
                        break;

                    default:
                        SafetyChecks.ThrowInvalidOperationException("Simplex has invalid count.");
                        return default;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    break;
                }

                // Get search direction.
                var direction = simplex.GetSearchDirection();

                // Ensure the search direction is numerically fit.
                if (math.lengthsq(direction) < float.Epsilon * float.Epsilon)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }

                // Compute a tentative new simplex vertex using support points.
                var vertex = vertices + simplex.Count;
                vertex->IndexA = proxyA.GetSupport(math.mul(inverseRotationA, -direction));
                vertex->SupportA = mul(transformA, proxyA.Vertices[vertex->IndexA]);
                vertex->IndexB = proxyB.GetSupport(direction);
                vertex->SupportB = proxyB.Vertices[vertex->IndexB];
                vertex->W = vertex->SupportB - vertex->SupportA;

                // Iteration count is equated to the number of support point calls.
                ++iteration;

                // Check for duplicate support points. This is the main termination criteria.
                var duplicate = false;
                for (var i = 0; i < saveCount; ++i)
                {
                    if (vertex->IndexA == saveA.Index[i] && vertex->IndexB == saveB.Index[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                // If we found a duplicate support point we must exit to avoid cycling.
                if (duplicate)
                    break;

                // New vertex is okay and needed.
                simplex.Count++;
            }

            // Prepare result.
            var pointA = float2.zero;
            var pointB = float2.zero;
            simplex.GetWitnessPoints(ref pointA, ref pointB);

            var distance = math.distance(pointA, pointB);
            var radiusA = proxyA.ConvexRadius;
            var radiusB = proxyB.ConvexRadius;
            if (distance > (radiusA + radiusB) && distance > float.Epsilon)
            {
                // Shapes not overlapped.
                // Move the witness points to the outer surface.
                distance -= radiusA + radiusB;
                var normal = math.normalize(pointB - pointA);
                pointA += radiusA * normal;
                pointB -= radiusB * normal;
            }
            else
            {
                // Shapes are overlapped.
                // Move the witness points to the middle.
                pointA = pointB = 0.5f * (pointA + pointB);
                distance = 0f;
            }

            return new DistanceHit
            {
                PointA = pointA,
                PointB = pointB,
                Fraction = distance
            };
        }

        #endregion

        #region Point Distance.

        internal unsafe static bool PointDistance<T>(PointDistanceInput input, ref Collider collider, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider.Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            var proxySource = new DistanceProxy(1, &input.Position, 0f);
            DistanceProxy proxyTarget;

            switch (collider.ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                {
                    proxyTarget = new DistanceProxy(ref collider.m_ConvexHull);
                    break;
                }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            var hit = ColliderDistance(PhysicsTransform.Identity, ref proxySource, ref proxyTarget);
            if (hit.Distance < collector.MaxFraction)
            {
                hit.PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex;
                hit.ColliderKey = input.QueryContext.ColliderKey;
                hit.Entity = input.QueryContext.Entity;

                hit.PointA = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, hit.PointA);
                hit.PointB = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, hit.PointB);

                return collector.AddHit(hit);
            }

            return false;
        }

        //#region Point Distance Compound

        //static unsafe bool PointDistanceCompound<T>(PointDistanceInput input, PhysicsCompoundCollider* compoundCollider, ref T collector)
        //    where T : struct, ICollector<DistanceHit>
        //{
        //    var leafProcessor = new PointDistanceCompoundLeafProcessor(compoundCollider);
        //    return compoundCollider->BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
        //}

        //unsafe struct PointDistanceCompoundLeafProcessor : BoundingVolumeHierarchy.IPointDistanceLeafProcessor
        //{
        //    readonly PhysicsCompoundCollider* m_CompoundCollider;

        //    public PointDistanceCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
        //    {
        //        m_CompoundCollider = compoundCollider;
        //    }

        //    public bool DistanceLeaf<T>(PointDistanceInput input, int leafData, ref T collector)
        //        where T : struct, ICollector<DistanceHit>
        //    {
        //        ref var child = ref m_CompoundCollider->Children[leafData];

        //        if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
        //        {
        //            return false;
        //        }

        //        // Transform the point into child space.
        //        var inputLs = input;
        //        {
        //            var compoundFromChild = child.CompoundFromChild;
        //            var childFromCompound = PhysicsMath.inverse(compoundFromChild);

        //            inputLs.Position = PhysicsMath.mul(childFromCompound, input.Position);

        //            inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
        //            inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
        //            inputLs.QueryContext.LocalToWorldTransform = PhysicsMath.mul(inputLs.QueryContext.LocalToWorldTransform, compoundFromChild);
        //        }

        //        return child.Collider->CalculateDistance(inputLs, ref collector);
        //    }
        //}

        //#endregion

        #endregion

        #region Collider Distance

        static DistanceHit DistanceConvex(ref ColliderDistanceInput input, ref DistanceProxy proxyTarget)
        {
            DistanceProxy proxySource;

            var inputColliderBlob = input.Collider;
            switch (inputColliderBlob.Value.ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                {
                    ref var convexHull = ref inputColliderBlob.Value.m_ConvexHull;
                    proxySource = new DistanceProxy(ref convexHull);
                    break;
                }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            return ColliderDistance(input.Transform, ref proxySource, ref proxyTarget);
        }

        internal static bool ColliderDistance<T>(ColliderDistanceInput input, ref Collider collider, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            if (!input.Collider.IsCreated ||
                !CollisionFilter.IsCollisionEnabled(input.Collider.Value.Filter, collider.Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            DistanceHit hit;
            switch (collider.ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                {
                    var proxyTarget = new DistanceProxy(ref collider.m_ConvexHull);
                    hit = DistanceConvex(ref input, ref proxyTarget);
                    break;
                }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            if (hit.Distance < collector.MaxFraction)
            {
                hit.PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex;
                hit.ColliderKey = input.QueryContext.ColliderKey;
                hit.Entity = input.QueryContext.Entity;

                hit.PointA = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, hit.PointA);
                hit.PointB = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, hit.PointB);

                return collector.AddHit(hit);
            }

            return false;
        }

        #endregion
    }

    // Simplex from Box2D.
    unsafe struct Simplex
    {
        public struct Vertex
        {
            public float2 SupportA;	// support point in proxyA
            public float2 SupportB;	// support point in proxyB
            public float2 W;	    // SupportB - SupportA
            public float A;		    // Barycentric coordinate for closest point
            public int IndexA;	    // SupportA index
            public int IndexB;	    // SupportB index
        }

        public struct VertexIndexTriple
        {
            public fixed int Index[3];
        }

        public void Reset(PhysicsTransform transformA, DistanceProxy proxyA, DistanceProxy proxyB)
        {
            var supportA = PhysicsMath.mul(transformA, *proxyA.Vertices);
            var supportB = *proxyB.Vertices;

            Vertex1 = new Vertex
            {
                SupportA = supportA,
                SupportB = supportB,
                W = supportB - supportA,
                A = 1f,
                IndexA = 0,
                IndexB = 0
            };

            Count = 1;
        }

        public void Solve2()
        {
            var w1 = Vertex1.W;
            var w2 = Vertex2.W;
            var e12 = w2 - w1;

            // w1 region
            var d12_2 = -math.mul(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                Vertex1.A = 1.0f;
                Count = 1;
                return;
            }

            // w2 region
            var d12_1 = math.mul(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                Vertex2.A = 1.0f;
                Count = 1;
                Vertex1 = Vertex2;
                return;
            }

            // Must be in e12 region.
            var inv_d12 = 1.0f / (d12_1 + d12_2);
            Vertex1.A = d12_1 * inv_d12;
            Vertex2.A = d12_2 * inv_d12;
            Count = 2;
        }

        public void Solve3()
        {
            var w1 = Vertex1.W;
            var w2 = Vertex2.W;
            var w3 = Vertex3.W;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            var e12 = w2 - w1;
            var w1e12 = math.mul(w1, e12);
            var w2e12 = math.mul(w2, e12);
            var d12_1 = w2e12;
            var d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            var e13 = w3 - w1;
            var w1e13 = math.mul(w1, e13);
            var w3e13 = math.mul(w3, e13);
            var d13_1 = w3e13;
            var d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            var e23 = w3 - w2;
            var w2e23 = math.mul(w2, e23);
            var w3e23 = math.mul(w3, e23);
            var d23_1 = w3e23;
            var d23_2 = -w2e23;

            // Triangle123
            var n123 = PhysicsMath.cross(e12, e13);

            var d123_1 = n123 * PhysicsMath.cross(w2, w3);
            var d123_2 = n123 * PhysicsMath.cross(w3, w1);
            var d123_3 = n123 * PhysicsMath.cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                Vertex1.A = 1.0f;
                Count = 1;
                return;
            }

            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                var inv_d12 = 1.0f / (d12_1 + d12_2);
                Vertex1.A = d12_1 * inv_d12;
                Vertex2.A = d12_2 * inv_d12;
                Count = 2;
                return;
            }

            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                var inv_d13 = 1.0f / (d13_1 + d13_2);
                Vertex1.A = d13_1 * inv_d13;
                Vertex3.A = d13_2 * inv_d13;
                Count = 2;
                Vertex2 = Vertex3;
                return;
            }

            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                Vertex2.A = 1.0f;
                Count = 1;
                Vertex1 = Vertex2;
                return;
            }

            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                Vertex3.A = 1.0f;
                Count = 1;
                Vertex1 = Vertex3;
                return;
            }

            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                var inv_d23 = 1.0f / (d23_1 + d23_2);
                Vertex2.A = d23_1 * inv_d23;
                Vertex3.A = d23_2 * inv_d23;
                Count = 2;
                Vertex1 = Vertex3;
                return;
            }

            // Must be in triangle123
            var inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            Vertex1.A = d123_1 * inv_d123;
            Vertex2.A = d123_2 * inv_d123;
            Vertex3.A = d123_3 * inv_d123;
            Count = 3;
        }

        public float2 GetSearchDirection()
        {
            switch (Count)
            {
                case 1:
                    return -Vertex1.W;

                case 2:
                {
                    var e12 = Vertex2.W - Vertex1.W;
                    var sgn = PhysicsMath.cross(e12, -Vertex1.W);
                    if (sgn > 0.0f)
                    {
                        // Origin is left of e12.
                        return PhysicsMath.cross(1.0f, e12);
                    }
                    else
                    {
                        // Origin is right of e12.
                        return PhysicsMath.cross(e12, 1.0f);
                    }
                }

                default:
                    SafetyChecks.ThrowInvalidOperationException("Invalid simplex search direction.");
                    return default;
            }
        }

        public float2 GetClosestPoint()
        {
            switch (Count)
            {
                case 1:
                    return Vertex1.W;

                case 2:
                    return Vertex1.A * Vertex1.W + Vertex2.A * Vertex2.W;

                case 3:
                    return float2.zero;

                case 0:
                default:
                    SafetyChecks.ThrowInvalidOperationException("Invalid simplex search direction.");
                    return default;
            }
        }

        public void GetWitnessPoints(ref float2 vertexA, ref float2 vertexB)
        {
            switch (Count)
            {
                case 1:
                    vertexA = Vertex1.SupportA;
                    vertexB = Vertex1.SupportB;
                    return;

                case 2:
                    vertexA = Vertex1.A * Vertex1.SupportA + Vertex2.A * Vertex2.SupportA;
                    vertexB = Vertex1.A * Vertex1.SupportB + Vertex2.A * Vertex2.SupportB;
                    return;

                case 3:
                    vertexA = Vertex1.A * Vertex1.SupportA + Vertex2.A * Vertex2.SupportA + Vertex3.A * Vertex3.SupportA;
                    vertexB = vertexA;
                    return;

                case 0:
                default:
                    SafetyChecks.ThrowInvalidOperationException("Invalid Simplex count.");
                    return;
            }
        }

        public int Count;
        public Vertex Vertex1;
        public Vertex Vertex2;
        public Vertex Vertex3;
    }

    // Proxy of a convex hull used for relative distance query.
    unsafe struct DistanceProxy
    {
        public DistanceProxy(ref ConvexHull hull)
        {
            Vertices = hull.Vertices.GetUnsafePtr();
            VertexCount = hull.Length;
            ConvexRadius = hull.ConvexRadius;
        }

        public DistanceProxy(int vertexCount, float2* vertices, float convexRadius)
        {
            Vertices = vertices;
            VertexCount = vertexCount;
            ConvexRadius = convexRadius;
        }

        public int GetSupport(float2 direction)
        {
            var bestIndex = 0;
            var bestValue = math.dot(Vertices[0], direction);
            for (var i = 1; i < VertexCount; ++i)
            {
                var value = math.dot(Vertices[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        public ref float2 GetSupportVertex(float2 direction)
        {
            var bestIndex = 0;
            var bestValue = math.dot(Vertices[0], direction);
            for (var i = 1; i < VertexCount; ++i)
            {
                var value = math.dot(Vertices[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return ref Vertices[bestIndex];
        }

        public float2* Vertices;
        public int VertexCount;
        public float ConvexRadius;
    }
}
