using System;
using Unity.Mathematics;
using UnityEngine;
using Util;

namespace Pathfinding.Nodes
{
    public struct HybridNode : INode, INeighbours3X2, IEdges4X3, IVertices4X2
    {
        public double gValue { get; set; }
        public double rhsValue { get; set; }
        public double priority { get; set; }
        public double2 priorityKey { get; set; }
        public int3 localPosition { get; set; }
        public float3 globalPosition { get; set; }
        public bool isInOpenList { get; set; }
        public bool isInClosedList { get; set; }
        public int index { get; set; }
        public int parentNode { get; set; }
        public int4x3 edgeNeighbours { get; set; }
        public int4x3 orderedEdgeNeighbours { get; set; }
        public int4x3 edgePredecessors { get; set; }
        public int edgesCount { get; set; }
        public int neighboursCount { get; set; }
        public int3x2 neighbours { get; set; }
        public int3x2 orderedNeighbours { get; set; }
        public int3x2 predecessors { get; set; }
        public int4x2 verticesNeighbours { get; set; }
        public int4x2 orderedVerticesNeighbours { get; set; }
        public int4x2 verticesPredecessors { get; set; }
        public int verticesCount { get; set; }
        public bool isStaticNode { get; set; }
        public bool isDynamicNode { get; set; }
        public bool isDynamicNeighbourNode { get; set; }
        public float3x2 basicProjections { get; set; }
        public bool isUsed { get; set; }
        
        public HybridNode(int3 localPosition, float3 globalPosition, int index)
        {
            this.localPosition = localPosition;
            this.globalPosition = globalPosition;
            this.index = index;
            
            gValue = math.INFINITY_DBL;
            rhsValue = math.INFINITY_DBL;

            priority = math.INFINITY_DBL;
            priorityKey = new double2();

            neighboursCount = 0;
            neighbours = new int3x2();
            predecessors = new int3x2();
            orderedNeighbours = new int3x2();

            edgesCount = 0;
            edgeNeighbours = new int4x3();
            edgePredecessors = new int4x3();
            orderedEdgeNeighbours = new int4x3();

            verticesCount = 0;
            verticesNeighbours = new int4x2();
            verticesPredecessors = new int4x2();
            orderedVerticesNeighbours = new int4x2();

            parentNode = -1;
            isInOpenList = false;
            isInClosedList = false;
            
            isStaticNode = false;
            isDynamicNode = false;
            isDynamicNeighbourNode = false;

            basicProjections = -1;
            isUsed = false;
        }
        
        public HybridNode Reset()
        {
            gValue = math.INFINITY_DBL;
            rhsValue = math.INFINITY_DBL;
            parentNode = -1;

            priority = math.INFINITY_DBL;
            double2 key = priorityKey;
            priorityKey = key;

            isInOpenList = false;
            isInClosedList = false;
            return this;
        }

        public void SetBasicProjections(Transform transform)
        {
            float3x2 proj = 6;
            float3[] globalVertices = new float3[8];
            float3[] halfBaseAxis = {transform.right / 2, transform.up / 2, transform.forward / 2};
            for (int i = 0; i < 8; i += 4)
            {
                int zFactor = i == 0 ? 1 : -1;
                globalVertices[i] = globalPosition - halfBaseAxis[0] + halfBaseAxis[1] + zFactor * halfBaseAxis[2];
                globalVertices[i+1] = globalPosition + halfBaseAxis[0] + halfBaseAxis[1] + zFactor * halfBaseAxis[2];
                globalVertices[i+2] = globalPosition - halfBaseAxis[0] - halfBaseAxis[1] + zFactor * halfBaseAxis[2];
                globalVertices[i+3] = globalPosition + halfBaseAxis[0] - halfBaseAxis[1] + zFactor * halfBaseAxis[2];
            }
            
            for (int i = 0; i < 3; i++)
            {
                SeparatingAxisTheorem.Project(halfBaseAxis[i] * 2, globalVertices, out float projectionMin, out float projectionMax);
                proj[0][i] = projectionMin;
                proj[1][i] = projectionMax;
            }

            basicProjections = proj;
        }

        public void RecoverSaveState(HybridNodeSaveState node)
        {
            index = node.index;
            edgeNeighbours = node.edgeNeighbours;
            edgePredecessors = node.edgePredecessors;
            orderedEdgeNeighbours = node.orderedEdgeNeighbours;
            edgesCount = node.edgesCount;
            neighboursCount = node.neighboursCount;
            neighbours = node.neighbours;
            predecessors = node.predecessors;
            orderedNeighbours = node.orderedNeighbours;
            verticesNeighbours = node.verticesNeighbours;
            verticesPredecessors = node.verticesPredecessors;
            orderedVerticesNeighbours = node.orderedVerticesNeighbours;
            verticesCount = node.verticesCount;
            isStaticNode = node.isStaticNode;
            isDynamicNode = node.isDynamicNode;
            isDynamicNeighbourNode = node.isDynamicNeighbourNode;
            isUsed = true;
        }
        
    }

    [Serializable]
    public struct HybridNodeSaveState
    {
        public int index;
        
        public int4x3 edgeNeighbours;
        public int4x3 edgePredecessors;
        public int4x3 orderedEdgeNeighbours;
        public double4x3 edgePathCosts;
        public int edgesCount;
        public int neighboursCount;
        public int3x2 neighbours;
        public int3x2 predecessors;
        public int3x2 orderedNeighbours;
        public double3x2 pathCosts;
        public int4x2 verticesNeighbours;
        public int4x2 verticesPredecessors;
        public int4x2 orderedVerticesNeighbours;
        public double4x2 verticesPathCosts;
        public int verticesCount;
        
        public bool isStaticNode;
        public bool isDynamicNode;
        public bool isDynamicNeighbourNode;
        

        public HybridNodeSaveState(HybridNode node, HybridNodePathCosts nodePathCosts)
        {
            index = node.index;
            edgeNeighbours = node.edgeNeighbours;
            edgePredecessors = node.edgePredecessors;
            orderedEdgeNeighbours = node.orderedEdgeNeighbours;
            edgePathCosts = nodePathCosts.edgePathCosts;
            edgesCount = node.edgesCount;
            neighboursCount = node.neighboursCount;
            neighbours = node.neighbours;
            predecessors = node.predecessors;
            orderedNeighbours = node.orderedNeighbours;
            pathCosts = nodePathCosts.pathCosts;
            verticesNeighbours = node.verticesNeighbours;
            verticesPredecessors = node.verticesPredecessors;
            orderedVerticesNeighbours = node.orderedVerticesNeighbours;
            verticesPathCosts = nodePathCosts.verticesPathCosts;
            verticesCount = node.verticesCount;
            isStaticNode = node.isStaticNode;
            isDynamicNode = node.isDynamicNode;
            isDynamicNeighbourNode = node.isDynamicNeighbourNode;
        }
    }
}
