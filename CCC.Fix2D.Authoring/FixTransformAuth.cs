using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using UnityEngineX.InspectorDisplay;

namespace CCC.Fix2D.Authoring
{
    public class FixTransformAuth : MonoBehaviour
    {
        // fbessette: This could all be removed
        [System.Serializable]
        public struct SerializedData
        {
            public fix3 LocalPosition;
            public fixQuaternion LocalRotation;
            public fix3 LocalScale;
            public FixTransformAuth Parent;
            public int SiblingIndex;
        }

        public fix3 LocalScale { get => _data.LocalScale; set { _data.LocalScale = value; } }
        public fix3 LocalPosition { get => _data.LocalPosition; set { _data.LocalPosition = value; } }
        public fixQuaternion LocalRotation { get => _data.LocalRotation; set { _data.LocalRotation = value; } }

        [SerializeField]
        [AlwaysExpand]
        public SerializedData _data = new SerializedData() // needs to be public for Editor access
        {
            LocalScale = new fix3(1, 1, 1)
        };
    }
}