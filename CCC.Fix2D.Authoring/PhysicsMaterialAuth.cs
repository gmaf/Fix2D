using UnityEngine;

namespace CCC.Fix2D.Authoring
{
    [CreateAssetMenu(menuName = "Fix 2D/Physics Material")]
    public class PhysicsMaterialAuth : ScriptableObject
    {
        [Tooltip("Tons per m3. Water = 1, Wood = 0.6, Steel = 8, Flesh = 1.1, Air = 0.001")]
        public float MassDensity = 1;

        [Range(0, 1)]
        public float Bounciness = 0;

        public float Friction = 0.4f;
    }
}
