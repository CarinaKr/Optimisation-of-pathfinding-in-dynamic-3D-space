using System;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.SearchGraph;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(HybridManager))]
public class HybridManagerEditor : Editor
{
    private HybridManager hybridManager;

    private void Awake()
    {
        Init();
    }

    private void Init()
    {
        EditorApplication.playModeStateChanged += OnChangePlayMode;
        hybridManager = (HybridManager)target;
    }
    
    private void OnChangePlayMode(PlayModeStateChange obj)
    {
        if (hybridManager == null || target == null)
        {
            return;
        }
        if (obj == PlayModeStateChange.EnteredEditMode && hybridManager.IsBakingSearchGraph)
        {
            hybridManager.IsBakingSearchGraph = false;
            hybridManager.gameObject.SetActive(false);
            EditorUtility.SetDirty(target);
        }
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        if (hybridManager == null)
        {
            Init();
        }
        
        if (GUILayout.Button("Bake Search Graph"))
        {
            hybridManager.gameObject.SetActive(true);
            hybridManager.IsBakingSearchGraph = true;
            EditorUtility.SetDirty(target);
            EditorApplication.isPlaying = true;
        }
        if (GUILayout.Button("Stop Baking Search Graph"))
        {
            hybridManager.IsBakingSearchGraph = false;
            EditorUtility.SetDirty(target);
        }
    }
}
