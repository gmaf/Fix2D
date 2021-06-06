using System;
using System.Diagnostics;
using Unity.Entities;

namespace CCC.Fix2D
{
    [Serializable]
    [DebuggerDisplay("{Value}")]
    public struct FixTranslation : IComponentData
    {
        public const int BYTE_SIZE = 2 * 8;

        public fix2 Value;

        public FixTranslation(fix2 value)
        {
            Value = value;
        }

        public static implicit operator fix2(FixTranslation val) => val.Value;
        public static implicit operator FixTranslation(fix2 val) => new FixTranslation() { Value = val };
    }
}