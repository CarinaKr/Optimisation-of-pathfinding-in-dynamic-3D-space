using System;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.SearchGraph;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PathfindingManager))]
public class PathfindingManagerEditor : Editor
{
    private SerializedProperty pathfindingMethodCellGrid;
    private SerializedProperty pathfindingMethodHybrid;
    private SerializedProperty pathfinderSetup;
    
    private void OnEnable()
    {
        pathfindingMethodCellGrid = serializedObject.FindProperty ("pathfindingMethodCellGrid");
        pathfindingMethodHybrid = serializedObject.FindProperty ("pathfindingMethodHybrid");
        pathfinderSetup = serializedObject.FindProperty ("pathfinderSetup");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        EditorGUILayout.HelpBox("You are using the search graph type: " + SearchGraphSelector.Instance.searchGraphType, MessageType.Info);
        if (SearchGraphSelector.Instance.searchGraphType == SearchGraphSelector.SearchGraphType.CELL_GRID)
        {
            EditorGUILayout.PropertyField(pathfindingMethodCellGrid);
        }
        else
        {
            EditorGUILayout.PropertyField(pathfindingMethodHybrid, new GUIContent("Pathfinding Method"));
        }
        EditorGUILayout.PropertyField(pathfinderSetup);
        serializedObject.ApplyModifiedProperties();
    }
}
