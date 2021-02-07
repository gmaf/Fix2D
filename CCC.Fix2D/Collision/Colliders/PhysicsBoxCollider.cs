using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public struct BoxGeometry : IEquatable<BoxGeometry>
    {
        public float2 Size { get => m_Size; set => m_Size = value; }
        float2 m_Size;

        public float2 Center { get => m_Center; set => m_Center = value; }
        float2 m_Center;

        public float Angle { get => m_Angle; set => m_Angle = value; }
        float m_Angle;

        public float BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        float m_BevelRadius;

        public bool Equals(BoxGeometry other)
        {
            return m_Size.Equals(other.m_Size)
                && m_Center.Equals(other.m_Center)
                && m_Angle.Equals(other.m_Angle)
                && m_BevelRadius.Equals(other.m_BevelRadius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint3(
                math.hash(m_Size),
                math.hash(m_Center),
                math.hash(new float2(m_Angle, m_BevelRadius))
                )));
        }

        internal void Validate()
        {
            if (math.any(!math.isfinite(m_Size)) || math.any(m_Size <= 0.0f))
                SafetyChecks.ThrowArgumentException("Cannot specify less than zero or Infinite/NaN.", "Size");

            if (math.any(!math.isfinite(m_Center)))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Center");

            if (!math.isfinite(Angle))
                SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Angle");

            if (!math.isfinite(m_BevelRadius) || m_BevelRadius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "BevelRadius");
        }
    }
}
