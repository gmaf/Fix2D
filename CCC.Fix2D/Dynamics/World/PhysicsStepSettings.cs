using System;
using Unity.Entities;
using Unity.Mathematics;

namespace CCC.Fix2D
{
    [Serializable]
    public struct PhysicsStepSettings : IComponentData
    {
        public static PhysicsStepSettings Default => new PhysicsStepSettings
        {
            Gravity = new float2(0.0f, -9.81f),
            NumberOfThreadsHint = 8,
            SimulationType = SimulationType.Default,
            NumSolverIterations = 4,
            SynchronizeCollisionWorld = false,
            TimeStep = 1 / 60f,
            SolverStabilizationHeuristicSettings = Solver.StabilizationHeuristicSettings.Default
        };

        // Settings for solver stabilization heuristic in Unity.Physics
        public Solver.StabilizationHeuristicSettings SolverStabilizationHeuristicSettings;

        // Number of iterations to perform while solving constraints
        public int NumSolverIterations;

        // World gravity.
        public float2 Gravity;

        public fix2 GravityFix { get => (fix2)Gravity; set => Gravity = (float2)value; }

        public float TimeStep;

        // The number of available threads.
        public int NumberOfThreadsHint;

        // The type of simulation to be used when simulating this physics world.
        public SimulationType SimulationType;

        // Whether to update the collision world after the step for more precise queries
        public bool SynchronizeCollisionWorld;

        // Constants.
        public struct Constants
        {
            // A small length used as a collision and constraint tolerance.
            public const float LinearSlop = 0.005f;

            // Default Convex Radius.
            public const float MinimumConvexRadius = LinearSlop * 2.0f;

            // Maximum iterations allowed for GJK.
            public const int MaxGJKInterations = 20;

            // Contacts are always created between PhysicsBody if they are closer than this distance threshold.
            public const float CollisionTolerance = 0.01f;
        }

        public void Validate()
        {
            var defaultSettings = Default;

            if (math.any(!math.isfinite(Gravity)))
                Gravity = defaultSettings.Gravity;

            if (NumberOfThreadsHint < 0)
                NumberOfThreadsHint = defaultSettings.NumberOfThreadsHint;

            if (TimeStep < 0)
                TimeStep = 0f;
        }
    }
}
