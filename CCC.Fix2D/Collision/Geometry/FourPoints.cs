using Unity.Mathematics;

namespace CCC.Fix2D
{
    public struct FourPoints
    {
        float4x2 m_Points;

        public float4 X => m_Points.c0;
        public float4 Y => m_Points.c1;

        public FourPoints(float2 point)
        {
            m_Points.c0 = new float4(point.x);
            m_Points.c1 = new float4(point.y);
        }
    }
}
