using System;
using System.Diagnostics;
using Unity.Entities;

[Serializable]
[DebuggerDisplay("{Value}")]
public struct FixTranslation : IComponentData
{
    public fix2 Value;

    public static implicit operator fix2(FixTranslation val) => val.Value;
    public static implicit operator FixTranslation(fix2 val) => new FixTranslation() { Value = val };
}