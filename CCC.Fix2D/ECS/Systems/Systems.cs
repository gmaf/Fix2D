using CCC.Fix2D;
using System;
using System.Collections.Generic;
using Unity.Entities;

namespace CCC.Fix2D
{
    public static class Fix2DHelpers
    {
        public static void AddPhysicsSystemsToGroup(World world, ComponentSystemGroup systemGroup, bool addDebugSystems = true)
        {
            if (world is null)
                throw new ArgumentNullException(nameof(world));

            if (systemGroup is null)
                throw new ArgumentNullException(nameof(systemGroup));

            var physicsGroup = world.CreateSystem<PhysicsSystemGroup>();

            List<Type> sysTypes = new List<Type>
            {
                typeof(PhysicsColliderBlobDisposalSystem),
                typeof(PhysicsWorldSystem),
                typeof(StepPhysicsWorldSystem),
                typeof(ExportPhysicsWorldSystem),
                typeof(EndFramePhysicsSystem),
            };

            if (addDebugSystems)
            {
                sysTypes.Add(typeof(Debugging.DebugDisplayBroadphaseSystem));
                sysTypes.Add(typeof(Debugging.DebugDisplayColliderAabbSystem));
                sysTypes.Add(typeof(Debugging.DebugDisplayColliderSystem));
                sysTypes.Add(typeof(Debugging.DebugDisplayCollisionEventsSystem));
                sysTypes.Add(typeof(Debugging.DebugDisplayTriggerEventsSystem));
                sysTypes.Add(typeof(Debugging.PhysicsDebugStreamSystem));
            }

            for (int i = 0; i < sysTypes.Count; i++)
            {
                physicsGroup.AddSystemToUpdateList(world.CreateSystem(sysTypes[i]));
            }

            systemGroup.AddSystemToUpdateList(physicsGroup);
        }
    }
}
