using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public struct CapsuleGeometry : IEquatable<CapsuleGeometry>
    {
        public float2 Vertex0 { get => m_Vertex0; set => m_Vertex0 = value; }
        float2 m_Vertex0;

        public float2 Vertex1 { get => m_Vertex1; set => m_Vertex1 = value; }
        float2 m_Vertex1;

        public float Radius { get => m_Radius; set => m_Radius = value; }
        float m_Radius;

        public bool Equals(CapsuleGeometry other)
        {
            return m_Vertex0.Equals(other.m_Vertex0)
                && m_Vertex1.Equals(other.m_Vertex1)
                && m_Radius.Equals(other.m_Radius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                math.hash(m_Vertex0),
                math.hash(new float3(m_Vertex1, m_Radius))
                )));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Vertex0)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertex0");

            if (math.any(!math.isfinite(m_Vertex1)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertex1");

            if (!math.isfinite(m_Radius) || m_Radius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "Radius");
        }
    }
}
