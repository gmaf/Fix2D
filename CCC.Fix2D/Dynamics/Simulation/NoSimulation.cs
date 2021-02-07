using System;
using Unity.Jobs;

namespace CCC.Fix2D
{
    /// <summary>
    /// A simulation that performs no work.
    /// </summary>
    class NoSimulation : ISimulation
    {
        public SimulationType Type => SimulationType.None;
        
        public SimulationJobHandles ScheduleStepJobs(PhysicsWorld physicsWorld, PhysicsCallbacks callbacks, JobHandle inputDeps) => new SimulationJobHandles(inputDeps);
        
        public JobHandle FinalSimulationJobHandle => new JobHandle();
        public JobHandle FinalJobHandle => new JobHandle();
        
        public void Dispose() { }
    }
}