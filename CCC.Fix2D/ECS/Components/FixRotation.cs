using System;
using Unity.Entities;

[Serializable]
public struct FixRotation : IComponentData
{
    public fix Value;

    public static implicit operator fix(FixRotation val) => val.Value;
    public static implicit operator FixRotation(fix val) => new FixRotation() { Value = val };
}