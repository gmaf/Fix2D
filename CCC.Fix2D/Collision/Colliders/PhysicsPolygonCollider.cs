using System;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    public struct PolygonGeometry : IEquatable<PolygonGeometry>
    {
        public NativeArray<float2> Vertices { get => m_Vertices; set => m_Vertices = value; }
        NativeArray<float2> m_Vertices;

        public float BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        float m_BevelRadius;

        public bool Equals(PolygonGeometry other)
        {
            if (m_Vertices.Length != other.m_Vertices.Length ||
                !m_BevelRadius.Equals(other.m_BevelRadius))
                return false;

            var length = m_Vertices.Length;
            for (var i = 0; i < length; ++i)
            {
                if (!m_Vertices[i].Equals(other.m_Vertices[i]))
                    return false;
            }

            return true;
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                (uint)m_Vertices.GetHashCode(),
                math.hash(new float2(BevelRadius))
                )));
        }

        internal void Validate()
        {
            if (m_Vertices.Length < 3 || m_Vertices.Length > Collider.PolygonConstants.MaxVertexCount)
                SafetyChecks.ThrowArgumentException("Invalid number of vertices specified.", "Vertices (length)");

            for (var i = 0; i < m_Vertices.Length; ++i)
            {
                if (math.any(!math.isfinite(m_Vertices[i])))
                    SafetyChecks.ThrowArgumentException("Cannot specify Infinite/NaN.", "Vertices");
            }

            if (!math.isfinite(m_BevelRadius) || m_BevelRadius < 0.0f)
                SafetyChecks.ThrowArgumentException("Cannot specify less than 0 or Infinite/NaN.", "BevelRadius");
        }
    }
}
