using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


namespace TKPhysics.AutomaticDynamicBone.Runtime.Editor
{
    using Runtime;
    [CustomEditor(typeof(ADBPhysicsSettingSwitcher))]
    public class ADBPhysicsSettingSwitcherEditor : UnityEditor.Editor
    {
        ADBPhysicsSettingSwitcher controller;
        public void OnEnable()
        {
            controller = (target as ADBPhysicsSettingSwitcher);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("runtimeController"), new GUIContent("Target"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("currentLinker"), new GUIContent("Current Setting Linker"), true);
            EditorGUILayout.Space(10);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("targetLinkers"), new GUIContent("Setting Linker List"), true);
            if (GUILayout.Button("Switch Setting"))
            {
                controller.Switch();
            }
            serializedObject.ApplyModifiedProperties();
        }

    }

}
