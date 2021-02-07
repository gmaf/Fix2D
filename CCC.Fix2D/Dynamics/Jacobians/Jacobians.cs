using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace CCC.Fix2D
{
    public enum JacobianType : byte
    {
        // Contact Jacobians
        Contact,
        Trigger
    }

    // Flags which enable optional Jacobian behaviors
    [Flags]
    public enum JacobianFlags : byte
    {
        // These flags apply to all Jacobians
        Disabled = 1 << 0,
        EnableMassFactors = 1 << 1,
        UserFlag0 = 1 << 2,
        UserFlag1 = 1 << 3,
        UserFlag2 = 1 << 4,

        // These flags apply only to contact Jacobians
        IsTrigger = 1 << 5,
        EnableCollisionEvents = 1 << 6,
        EnableSurfaceVelocity = 1 << 7
    }

    // Jacobian header, first part of each Jacobian in the stream
    struct JacobianHeader
    {
        public PhysicsBody.IndexPair BodyPair { get; internal set; }
        public JacobianType Type { get; internal set; }
        public JacobianFlags Flags { get; internal set; }

        // Whether the Jacobian should be solved or not
        public bool Enabled
        {
            get => ((Flags & JacobianFlags.Disabled) == 0);
            set => Flags = value ? (Flags & ~JacobianFlags.Disabled) : (Flags | JacobianFlags.Disabled);
        }

        // Whether the Jacobian contains manifold data for collision events or not
        public bool HasContactManifold => (Flags & JacobianFlags.EnableCollisionEvents) != 0;

        // Collider keys for the collision events
        public ColliderKeyPair ColliderKeys
        {
            get => HasContactManifold ? AccessColliderKeys() : ColliderKeyPair.Empty;
            set
            {
                if (HasContactManifold)
                    AccessColliderKeys() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have collision events enabled");
            }
        }

        // Overrides for the mass properties of the pair of bodies
        public bool HasMassFactors => (Flags & JacobianFlags.EnableMassFactors) != 0;
        public MassFactors MassFactors
        {
            get => HasMassFactors ? AccessMassFactors() : MassFactors.Default;
            set
            {
                if (HasMassFactors)
                    AccessMassFactors() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have mass factors enabled");
            }
        }

        // The surface velocity to apply to contact points
        public bool HasSurfaceVelocity => (Flags & JacobianFlags.EnableSurfaceVelocity) != 0;
        public SurfaceVelocity SurfaceVelocity
        {
            get => HasSurfaceVelocity ? AccessSurfaceVelocity() : new SurfaceVelocity();
            set
            {
                if (HasSurfaceVelocity)
                    AccessSurfaceVelocity() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have surface velocity enabled");
            }
        }

        // Solve the Jacobian
        public void Solve([NoAlias] ref PhysicsBody.MotionVelocity velocityA, [NoAlias] ref PhysicsBody.MotionVelocity velocityB, Solver.StepInput stepInput,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter, [NoAlias] ref NativeStream.Writer triggerEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            if (Enabled)
            {
                switch (Type)
                {
                    case JacobianType.Contact:
                        AccessBaseJacobian<ContactJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                            enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
                        break;
                    case JacobianType.Trigger:
                        AccessBaseJacobian<TriggerJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref triggerEventsWriter);
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        return;
                }
            }
        }

        #region Helpers

        public static int CalculateSize(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return UnsafeUtility.SizeOf<JacobianHeader>() +
                SizeOfBaseJacobian(type) + SizeOfModifierData(type, flags) +
                numContactPoints * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                SizeOfContactPointData(type, flags, numContactPoints);
        }

        private static int SizeOfColliderKeys(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<ColliderKeyPair>() : 0;
        }

        private static int SizeOfEntityPair(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<EntityPair>() : 0;
        }

        private static int SizeOfSurfaceVelocity(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableSurfaceVelocity) != 0) ?
                UnsafeUtility.SizeOf<SurfaceVelocity>() : 0;
        }

        private static int SizeOfMassFactors(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableMassFactors) != 0) ?
                UnsafeUtility.SizeOf<MassFactors>() : 0;
        }

        private static int SizeOfModifierData(JacobianType type, JacobianFlags flags)
        {
            return SizeOfColliderKeys(type, flags) + SizeOfEntityPair(type, flags) + SizeOfSurfaceVelocity(type, flags) +
                SizeOfMassFactors(type, flags);
        }

        private static int SizeOfContactPointData(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                numContactPoints * UnsafeUtility.SizeOf<ContactPoint>() : 0;
        }

        private static int SizeOfBaseJacobian(JacobianType type)
        {
            switch (type)
            {
                case JacobianType.Contact:
                    return UnsafeUtility.SizeOf<ContactJacobian>();
                case JacobianType.Trigger:
                    return UnsafeUtility.SizeOf<TriggerJacobian>();
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }
        }

        // Access to "base" jacobian - a jacobian that comes after the header
        public unsafe ref T AccessBaseJacobian<T>() where T : struct
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>();
            return ref UnsafeUtility.AsRef<T>(ptr);
        }

        public unsafe ref ColliderKeyPair AccessColliderKeys()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type);
            return ref UnsafeUtility.AsRef<ColliderKeyPair>(ptr);
        }

        public unsafe ref EntityPair AccessEntities()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfColliderKeys(Type, Flags);
            return ref UnsafeUtility.AsRef<EntityPair>(ptr);
        }

        public unsafe ref SurfaceVelocity AccessSurfaceVelocity()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableSurfaceVelocity) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags);
            return ref UnsafeUtility.AsRef<SurfaceVelocity>(ptr);
        }

        public unsafe ref MassFactors AccessMassFactors()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableMassFactors) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags) + SizeOfSurfaceVelocity(Type, Flags);
            return ref UnsafeUtility.AsRef<MassFactors>(ptr);
        }

        public unsafe ref ContactJacAngAndVelToReachCp AccessAngularJacobian(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact || Type == JacobianType.Trigger);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                pointIndex * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>();
            return ref UnsafeUtility.AsRef<ContactJacAngAndVelToReachCp>(ptr);
        }

        public unsafe ref ContactPoint AccessContactPoint(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact);

            var baseJac = AccessBaseJacobian<ContactJacobian>();
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                baseJac.BaseJacobian.NumContacts * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                pointIndex * UnsafeUtility.SizeOf<ContactPoint>();
            return ref UnsafeUtility.AsRef<ContactPoint>(ptr);
        }

        #endregion
    }

    // Iterator (and modifier) for jacobians
    unsafe struct JacobianIterator
    {
        NativeStream.Reader m_Reader;

        public JacobianIterator(NativeStream.Reader jacobianStreamReader, int workItemIndex)
        {
            m_Reader = jacobianStreamReader;
            m_Reader.BeginForEachIndex(workItemIndex);
        }

        public bool HasJacobiansLeft()
        {
            return m_Reader.RemainingItemCount > 0;
        }

        public ref JacobianHeader ReadJacobianHeader()
        {
            int readSize = Read<int>();
            return ref UnsafeUtility.AsRef<JacobianHeader>(Read(readSize));
        }

        private byte* Read(int size)
        {
            byte* dataPtr = m_Reader.ReadUnsafePtr(size);

            return dataPtr;
        }

        private ref T Read<T>() where T : struct
        {
            int size = UnsafeUtility.SizeOf<T>();
            return ref UnsafeUtility.AsRef<T>(Read(size));
        }
    }
    // Helper functions for working with Jacobians
    static class JacobianUtilities
    {
        // Calculate the inverse effective mass for a pair of jacobians with perpendicular linear parts
        public static float CalculateInvEffectiveMassOffDiag(
            float3 angA0, float3 angA1, float3 invInertiaA,
            float3 angB0, float3 angB1, float3 invInertiaB)
        {
            return math.csum(angA0 * angA1 * invInertiaA + angB0 * angB1 * invInertiaB);
        }

        // Inverts a symmetrix 3x3 matrix with diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static bool InvertSymmetricMatrix(float3 diag, float3 offDiag, out float3 invDiag, out float3 invOffDiag)
        {
            float3 offDiagSq = offDiag.zyx * offDiag.zyx;
            float determinant = PhysicsMath.HorizontalMul(diag) + 2.0f * PhysicsMath.HorizontalMul(offDiag) - math.csum(offDiagSq * diag);
            bool determinantOk = (determinant != 0);
            float invDeterminant = math.select(0.0f, 1.0f / determinant, determinantOk);
            invDiag = (diag.yxx * diag.zzy - offDiagSq) * invDeterminant;
            invOffDiag = (offDiag.yxx * offDiag.zzy - diag.zyx * offDiag) * invDeterminant;
            return determinantOk;
        }

        // Builds a symmetric 3x3 matrix from diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static float3x3 BuildSymmetricMatrix(float3 diag, float3 offDiag)
        {
            return new float3x3(
                new float3(diag.x, offDiag.x, offDiag.y),
                new float3(offDiag.x, diag.y, offDiag.z),
                new float3(offDiag.y, offDiag.z, diag.z)
            );
        }
    }
}
