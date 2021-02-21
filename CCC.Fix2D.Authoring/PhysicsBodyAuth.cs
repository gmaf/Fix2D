using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace CCC.Fix2D.Authoring
{
    public class PhysicsBodyAuth : MonoBehaviour
    {
        public enum ColliderShape
        {
            Circle,
            Box,
            Polygon
        }

        public enum BodyType
        {
            Static,
            Kinematic,
            Dynamic
        }

        public BodyType Type = BodyType.Dynamic;
        public ColliderShape Shape = ColliderShape.Circle;
        public PhysicsMaterialAuth Material = null;
        public bool IsTrigger = false;
        public float LinearDrag = 0;
        public float AngularDrag = 0.05f;
        public float GravityScale = 1;
        public bool FireEvents = false;
        public bool FreezeRotation = false;

        // Box
        public Vector2 BoxSize = Vector2.one;

        // Circle
        public float CircleRadius = 0.5f;

        // Circle & Box
        public Vector2 BoxAndCircleOffset = Vector2.zero;

        // Polygon
        public List<Vector2> PolygonPoints = new List<Vector2>();

        private void OnDrawGizmosSelected()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.color = Color.green;
            switch (Shape)
            {
                case ColliderShape.Circle:
                    Gizmos.DrawWireSphere(BoxAndCircleOffset, CircleRadius);
                    break;
                case ColliderShape.Box:
                    Gizmos.DrawWireCube(BoxAndCircleOffset, BoxSize);
                    break;
                case ColliderShape.Polygon:
                    for (int i = 0; i < PolygonPoints.Count; i++)
                    {
                        Gizmos.DrawLine(PolygonPoints[i], PolygonPoints[(i + 1) % PolygonPoints.Count]);
                    }
                    break;
            }

            Gizmos.matrix = Matrix4x4.identity;
        }

        public float CalculateColliderArea()
        {
            switch (Shape)
            {
                default:
                case ColliderShape.Circle:
                    return 3.1416f * CircleRadius * CircleRadius;

                case ColliderShape.Box:
                    return BoxSize.x * BoxSize.y;

                case ColliderShape.Polygon:
                    float area = 0;

                    // Find a reference point inside the hull.
                    var referencePoint = Vector2.zero;
                    for (var i = 0; i < PolygonPoints.Count; ++i)
                    {
                        referencePoint += PolygonPoints[i];
                    }
                    referencePoint /= PolygonPoints.Count;

                    for (var i = 0; i < PolygonPoints.Count; ++i)
                    {
                        Vector2 edge1 = PolygonPoints[i] - referencePoint;
                        Vector2 edge2 = (i + 1 < PolygonPoints.Count ? PolygonPoints[i + 1] : PolygonPoints[0]) - referencePoint;

                        float crossEdge = PhysicsMath.cross(edge1, edge2);
                        float triangleArea = crossEdge * 0.5f;
                        area += triangleArea;
                    }

                    return math.abs(area);
            }
        }
    }
}

#if UNITY_EDITOR
namespace CCC.Fix2D.Authoring.Editor
{
    [CustomEditor(typeof(PhysicsBodyAuth))]
    public class PhysicsBodyAuthEditor : UnityEditor.Editor
    {
        private SerializedProperty _propType;
        private SerializedProperty _propShape;
        private SerializedProperty _propMaterial;
        private SerializedProperty _propIsTrigger;
        private SerializedProperty _propLinearDrag;
        private SerializedProperty _propAngularDrag;
        private SerializedProperty _propGravityScale;
        private SerializedProperty _propFireEvents;
        private SerializedProperty _propBoxSize;
        private SerializedProperty _propColliderOffset;
        private SerializedProperty _propCircleRadius;
        private SerializedProperty _propPolygonPoints;
        private SerializedProperty _freezeRotation;

        private void OnEnable()
        {
            _propType = serializedObject.FindProperty(nameof(PhysicsBodyAuth.Type));
            _propShape = serializedObject.FindProperty(nameof(PhysicsBodyAuth.Shape));
            _propMaterial = serializedObject.FindProperty(nameof(PhysicsBodyAuth.Material));
            _propIsTrigger = serializedObject.FindProperty(nameof(PhysicsBodyAuth.IsTrigger));
            _propLinearDrag = serializedObject.FindProperty(nameof(PhysicsBodyAuth.LinearDrag));
            _propAngularDrag = serializedObject.FindProperty(nameof(PhysicsBodyAuth.AngularDrag));
            _propGravityScale = serializedObject.FindProperty(nameof(PhysicsBodyAuth.GravityScale));
            _propFireEvents = serializedObject.FindProperty(nameof(PhysicsBodyAuth.FireEvents));
            _propBoxSize = serializedObject.FindProperty(nameof(PhysicsBodyAuth.BoxSize));
            _propColliderOffset = serializedObject.FindProperty(nameof(PhysicsBodyAuth.BoxAndCircleOffset));
            _propCircleRadius = serializedObject.FindProperty(nameof(PhysicsBodyAuth.CircleRadius));
            _propPolygonPoints = serializedObject.FindProperty(nameof(PhysicsBodyAuth.PolygonPoints));
            _freezeRotation = serializedObject.FindProperty(nameof(PhysicsBodyAuth.FreezeRotation));
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            PropertyField(_propType);
            PropertyField(_propMaterial);
            PropertyField(_propIsTrigger);
            
            if (!_propIsTrigger.boolValue)
                PropertyField("Collision Events", _propFireEvents);

            PhysicsBodyAuth.BodyType bodyType = (PhysicsBodyAuth.BodyType)_propType.enumValueIndex;

            if (bodyType == PhysicsBodyAuth.BodyType.Dynamic)
            {
                PropertyField("Drag", _propLinearDrag);
                PropertyField("Rotation Drag", _propAngularDrag);
                PropertyField(_propGravityScale);
                PropertyField("Freeze Rotation", _freezeRotation);
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Shape", EditorStyles.boldLabel);
            PropertyField(_propShape);

            PhysicsBodyAuth.ColliderShape shape = (PhysicsBodyAuth.ColliderShape)_propShape.enumValueIndex;


            if (shape == PhysicsBodyAuth.ColliderShape.Box)
            {
                PropertyField("Size", _propBoxSize);
                // PropertyField("Offset", _propColliderOffset); Disabled for now. Not sure we need this

            }

            if (shape == PhysicsBodyAuth.ColliderShape.Circle)
            {
                PropertyField("Radius", _propCircleRadius);
                // PropertyField("Offset", _propColliderOffset); Disabled for now. Not sure we need this
            }

            if (shape == PhysicsBodyAuth.ColliderShape.Polygon)
            {
                PropertyField("Vertices", _propPolygonPoints);
            }

            serializedObject.ApplyModifiedProperties();
        }

        private static GUIContent s_tempContent = new GUIContent();

        private void PropertyField(string label, SerializedProperty prop)
        {
            s_tempContent.text = label;
            EditorGUILayout.PropertyField(prop, s_tempContent);
        }

        private static void PropertyField(SerializedProperty prop)
        {
            EditorGUILayout.PropertyField(prop);
        }
    }
}
#endif
