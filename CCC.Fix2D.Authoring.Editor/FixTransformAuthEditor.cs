using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngineX;

namespace CCC.Fix2D.Authoring.Editor
{
    [CustomEditor(typeof(FixTransformAuth), editorForChildClasses: true)]
    public class FixTransformAuthEditor : UnityEditor.Editor
    {
        private void Awake()
        {
            // this is needed to make sure we update the data EVEN if the inspector is closed
            EditorApplication.update += UpdateData;
        }

        private void OnDestroy()
        {
            EditorApplication.update -= UpdateData;
        }

        public override void OnInspectorGUI()
        {
            if (Application.isPlaying)
            {
                //EditorGUI.BeginChangeCheck();

                base.OnInspectorGUI();

                //if (EditorGUI.EndChangeCheck())
                //{
                //    ((FixTransformAuth)target).Editor_DirtyCachedAllValues();
                //}
            }
            else
            {
                EditorGUI.BeginChangeCheck();

                EditorGUILayout.HelpBox("Use the regular transform to edit this data", MessageType.Info);

                GUI.enabled = false;
                base.OnInspectorGUI();
                GUI.enabled = true;


                FixTransformAuth simTransform = (FixTransformAuth)target;
                Transform unityTransform = simTransform.gameObject.transform;

                ////////////////////////////////////////////////////////////////////////////////////////
                //      Apply changes from the UnityTransform to the SimTransform (and vice versa)
                ////////////////////////////////////////////////////////////////////////////////////////
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(unityTransform, "Change UnityTransform");

                    unityTransform.localPosition = simTransform.LocalPosition.ToUnityVec();
                    unityTransform.localRotation = simTransform.LocalRotation.ToUnityQuat();
                    unityTransform.localScale = simTransform.LocalScale.ToUnityVec();

                    PrefabUtility.RecordPrefabInstancePropertyModifications(target);
                }

                UpdateData();
            }
        }

        void UpdateData()
        {
            if (Application.isPlaying || !target)
            {
                return;
            }
            ////////////////////////////////////////////////////////////////////////////////////////
            //      Apply changes from the UnityTransform to the SimTransform
            ////////////////////////////////////////////////////////////////////////////////////////
            {
                FixTransformAuth simTransform = (FixTransformAuth)target;
                Transform unityTransform = simTransform.gameObject.transform;
                fix3 localPosition = unityTransform.localPosition.ToFixVec();
                fixQuaternion localRotation = unityTransform.localRotation.ToFixQuat();
                fix3 localScale = unityTransform.localScale.ToFixVec();

                bool change = localPosition != simTransform.LocalPosition
                            || localRotation != simTransform.LocalRotation
                            || localScale != simTransform.LocalScale;

                if (change)
                {
                    Undo.RecordObject(target, "Change SimTransform");

                    simTransform.LocalPosition = localPosition;
                    simTransform.LocalRotation = localRotation;
                    simTransform.LocalScale = localScale;

                    PrefabUtility.RecordPrefabInstancePropertyModifications(target);
                }
            }

            ////////////////////////////////////////////////////////////////////////////////////////
            //      Make sure the prefab modifications match between the UnityTransform and the SimTransform
            ////////////////////////////////////////////////////////////////////////////////////////
            List<Modif> simModifToClear = new List<Modif>();
            foreach (FixTransformAuth simTr in new PrefabChainEnumerator<FixTransformAuth>((FixTransformAuth)target, true))
            {
                Transform unityTr = simTr.GetComponent<Transform>();
                PropertyModification[] modifications = PrefabUtility.GetPropertyModifications(unityTr);
                if (modifications == null)
                {
                    continue;
                }

                bool[] modifsInUnityTr = new bool[_bindedVars.Length];

                foreach (PropertyModification modif in modifications)
                {
                    if (modif.target is UnityEngine.Transform)
                    {
                        for (int i = 0; i < _bindedVars.Length; i++)
                        {
                            if (modif.propertyPath == _bindedVars[i].UnityPropertyPath)
                            {
                                modifsInUnityTr[i] = true;
                            }
                        }
                    }
                }

                foreach (PropertyModification modif in modifications)
                {
                    if (modif.target is FixTransformAuth)
                    {
                        for (int i = 0; i < _bindedVars.Length; i++)
                        {
                            // If the SimTransform has a modif but the UnityTransform doesn't, we must fix it (we'll apply/revert the SimTransform property)
                            if (modifsInUnityTr[i] == false && modif.propertyPath == _bindedVars[i].SimPropertyPath)
                            {
                                simModifToClear.Add(new Modif() { BindedVar = _bindedVars[i], RealModif = modif });
                                modifsInUnityTr[i] = true;
                            }
                        }
                    }
                }

                if (simModifToClear.Count > 0)
                {
                    bool isAPrefab = AssetDatabase.Contains(simTr.gameObject);
                    foreach (Modif modif in simModifToClear)
                    {
                        // _________________________________________ Find asset to apply to _________________________________________ //
                        Transform assetToApplyTo = null;
                        foreach (Transform p in new PrefabChainEnumerator<Transform>(unityTr, false))
                        {
                            assetToApplyTo = p;

                            // If this parent has a modification on that same property, that means it's the asset we should apply our
                            // current modif to. If we go further than that in the prefab chain, we'll overrite data we don't want to.
                            var parentModifs = PrefabUtility.GetPropertyModifications(p);
                            if (parentModifs != null && parentModifs.Contains((x) => x.propertyPath == modif.BindedVar.UnityPropertyPath))
                            {
                                break;
                            }
                        }
                        string assetPath = AssetDatabase.GetAssetPath(assetToApplyTo);
                        SerializedProperty propToApply = serializedObject.FindProperty(modif.RealModif.propertyPath);

                        // _________________________________________ Apply modif _________________________________________ //
                        if (isAPrefab)
                        {
                            PrefabUtility.RevertPropertyOverride(propToApply, InteractionMode.AutomatedAction);
                        }
                        else
                        {
                            PrefabUtility.ApplyPropertyOverride(propToApply, assetPath, InteractionMode.AutomatedAction);
                        }

                    }
                }

                simModifToClear.Clear();

                break;
            }

        }

        struct Modif
        {
            public BindedVar BindedVar;
            public PropertyModification RealModif;
        }
        class BindedVar
        {
            public string UnityPropertyPath;
            public string SimPropertyPath;
        }

        BindedVar[] _bindedVars = new BindedVar[]
        {
        new BindedVar()
        {
            UnityPropertyPath = "m_LocalPosition.x",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalPosition)}.{nameof(fix3.x)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalPosition.y",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalPosition)}.{nameof(fix3.y)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalPosition.z",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalPosition)}.{nameof(fix3.z)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalRotation.x",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalRotation)}.{nameof(fixQuaternion.x)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalRotation.y",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalRotation)}.{nameof(fixQuaternion.y)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalRotation.z",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalRotation)}.{nameof(fixQuaternion.z)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalRotation.w",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalRotation)}.{nameof(fixQuaternion.w)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalScale.x",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalScale)}.{nameof(fix3.x)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalScale.y",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalScale)}.{nameof(fix3.y)}.{nameof(fix.RawValue)}",
        },new BindedVar()
        {
            UnityPropertyPath = "m_LocalScale.z",
            SimPropertyPath   = $"{nameof(FixTransformAuth._data)}.{nameof(FixTransformAuth.SerializedData.LocalScale)}.{nameof(fix3.z)}.{nameof(fix.RawValue)}",
        },
        };


        struct PrefabChainEnumerator<T> where T : UnityEngine.Object
        {
            T _obj;
            bool _includeFirstInstance;
            public PrefabChainEnumerator(T obj, bool includeThisInstance)
            {
                _obj = obj;
                _includeFirstInstance = includeThisInstance;
            }
            public PrefabChainEnumerator<T> GetEnumerator() => this;

            public T Current => _obj;

            public bool MoveNext()
            {
                if (_includeFirstInstance)
                {
                    _includeFirstInstance = false;
                    return _obj != null;
                }

                T newObj = PrefabUtility.GetCorrespondingObjectFromSource(_obj);
                if (newObj == null || newObj == _obj) // we stop when we have the same object twice (means we've reached the end of the prefab chain)
                {
                    return false;
                }
                else
                {
                    _obj = newObj;
                    return newObj;
                }
            }
        }
    }
}
