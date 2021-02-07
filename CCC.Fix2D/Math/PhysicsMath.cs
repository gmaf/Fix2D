using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public static partial class PhysicsMath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float cross(float2 a, float2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 cross(float2 a, float s)
        {
            return new float2(s * a.y, -s * a.x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 cross(float s, float2 a)
        {
            return new float2(-s * a.y, s * a.x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 perp(float2 v)
        {
            return new float2(-v.y, v.x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion ToPositiveQuaternion(quaternion q)
        {
            return q.value.w < 0.0f ? -q.value : q.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ZRotationFromQuaternion(quaternion q)
        {
            var positiveQ = ToPositiveQuaternion(q);
            return 2.0f * math.atan2(positiveQ.value.z, positiveQ.value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion ZQuaternionFromQuaternion(quaternion q)
        {
            return QuaternionFromZRotation(ZRotationFromQuaternion(q));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion QuaternionFromZRotation(float rotation)
        {
            return new quaternion(0.0f, 0.0f, math.sin(0.5f * rotation), math.cos(0.5f * rotation));
        }
    
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float2 v) => v.x < v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float2 v) => IndexOfMinComponent(-v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float3 v) => v.x > v.y ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf16(int input) => ((input + 15) >> 4) << 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf16(ulong input) => ((input + 15) >> 4) << 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float HorizontalMul(float3 v) => v.x * v.y * v.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float HorizontalMul(float4 v) => (v.x * v.y) * (v.z * v.w);

        // Return two normals perpendicular to the input vector
        public static void CalculatePerpendicularNormalized(float3 v, out float3 p, out float3 q)
        {
            float3 vSquared = v * v;
            float3 lengthsSquared = vSquared + vSquared.xxx; // y = ||j x v||^2, z = ||k x v||^2
            float3 invLengths = math.rsqrt(lengthsSquared);

            // select first direction, j x v or k x v, whichever has greater magnitude
            float3 dir0 = new float3(-v.y, v.x, 0.0f);
            float3 dir1 = new float3(-v.z, 0.0f, v.x);
            bool cmp = (lengthsSquared.y > lengthsSquared.z);
            float3 dir = math.select(dir1, dir0, cmp);

            // normalize and get the other direction
            float invLength = math.select(invLengths.z, invLengths.y, cmp);
            p = dir * invLength;
            float3 cross = math.cross(v, dir);
            q = cross * invLength;
        }
    }
}