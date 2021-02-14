using System;
using System.Diagnostics;
using Unity.Entities;

namespace CCC.Fix2D
{
    [Serializable]
    [DebuggerDisplay("{Value}")]
    public struct FixRotation : IComponentData
    {
        public fix Value;

        public FixRotation(fix value)
        {
            Value = value;
        }

        public static implicit operator fix(FixRotation val) => val.Value;
        public static implicit operator FixRotation(fix val) => new FixRotation() { Value = val };
    }
}