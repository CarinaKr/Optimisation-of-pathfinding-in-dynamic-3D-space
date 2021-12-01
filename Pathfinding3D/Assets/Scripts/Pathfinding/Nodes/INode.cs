using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public interface INode
    {
        double gValue { get; set; }
        double rhsValue { get; set; }
        double priority { get; set; }
        double2 priorityKey { get; set; }
        int3 localPosition { get; set; }
        float3 globalPosition { get; set; }
        bool isInOpenList { get; set; }
        bool isInClosedList { get; set; }
        int index { get; set; }
        int parentNode { get; set; }
    }
    
    public struct NodesGroup
    {
        public float4 posXs;
        public float4 posYs;
        public float4 posZs;

        public float4 heuristics;
        public float4 plainHeuristics;

        public NodesGroup(float3 pos1, float3 pos2, float3 pos3, float3 pos4)
        {
            posXs = new float4(pos1.x, pos2.x, pos3.x, pos4.x);
            posYs = new float4(pos1.y, pos2.y, pos3.y, pos4.y);
            posZs = new float4(pos1.z, pos2.z, pos3.z, pos4.z);
            
            heuristics = 0;
            plainHeuristics = 0;
        }
    }
}
