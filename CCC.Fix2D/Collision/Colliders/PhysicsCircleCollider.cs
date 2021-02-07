using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public struct CircleGeometry : IEquatable<CircleGeometry>
    {
        public float2 Center { get => m_Center; set => m_Center = value; }
        float2 m_Center;

        public float Radius { get => m_Radius; set => m_Radius = value; }
        float m_Radius;

        public bool Equals(CircleGeometry other)
        {
            return m_Center.Equals(other.m_Center)
                && m_Radius.Equals(other.m_Radius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new float3(m_Center, m_Radius)));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Center)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Center");

            if (!math.isfinite(m_Radius) || m_Radius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "Radius");
        }
    }
}
