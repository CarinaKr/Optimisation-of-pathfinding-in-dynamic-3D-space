using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Util;

namespace Pathfinding.Algorithms
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct A_StarHybridJob : IJob, IPathfinderJob
    {
        public NativeHeap<HeapNode, HeapNodePriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeArray<HybridNode> nodes;
        public NativeArray<HybridNodePathCosts> nodePathCosts;
        public NativeArray<int> lastGoal;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<int> changes;
        public NativeArray<bool> update;
        public NativeArray<float> pathLength;
        public NativeArray<double> heuristic;
        public NativeArray<int> usedNodes;
        public float losOffsetMultiplier;

        public float3 aiPosition { get; set; }
        public float3 lookaheadPosition { get; set; }
        public float3 goalPosition { get; set; }
        public float3 searchGraphOrigin { get; set; }
        public float cellSize { get; set; }
        public int3 searchSpace { get; set; }
        public PathfinderSetup pathfinderSetup { get; set; }
        public int2 neighboursCountSize { get; set; }
        public int neighboursCount { get; set; }

        private int startNode;
        private int goalNode;
        private HybridNode tempCellNode, cellNode, nextCellNode;
        private int2 pos;
        private double3x2 neighbourGValues, neighbourFValues;
        private bool3x2 neighbourAddToOpen;
        private double4x3 edgeGValues, edgeFValues;
        private bool4x3 edgeAddToOpen;
        private double4x2 vertexGValues, vertexFValues;
        private bool4x2 vertexAddToOpen;
        private HeapNode heapNode;
        
        public void Execute()
        {
            for (int i = 0; i < usedNodes.Length; i++)
            {
                nodes[usedNodes[i]] = nodes[usedNodes[i]].Reset();
            }
            expanded[0] = 0;
            goalNode = CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // goalNode = 28859;

            if (lastGoal[0] == goalNode && changes.Length == 0) //TODO actually you'd need to check if the new goal node is on the current path, in which case there is no need to update
            {
                return;
            }
            update[0] = true;

            startNode = CellGridManagerHelper.GetClosestNode(lookaheadPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // startNode = 39361;
            if (startNode == -1)
            {
                return;
            }

            if (lastGoal[0] != goalNode)
            {
                lastGoal[0] = goalNode;
                PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition, pathfinderSetup.Epsilon, ref heuristic);
            }

            AddStartNodeToOpen(startNode);

            int nextNode = GetNextNode();
            
            while (nextNode != goalNode && nextNode != -1)
            {
                expanded[0]++;
                
                AStarBasedHybridPathfindingHelper.AddNeighboursToOpen(nextNode,nodePathCosts[nextNode].pathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                AStarBasedHybridPathfindingHelper.AddEdgesToOpen(nextNode,nodePathCosts[nextNode].edgePathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                AStarBasedHybridPathfindingHelper.AddVerticesToOpen(nextNode,nodePathCosts[nextNode].verticesPathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);

                nextNode = GetNextNode();
            }

            if (nextNode != -1)
            {
                corridor.Clear();
                float length = 0;
                PathfinderHelper.FillCorridor(goalNode, true, ref corridor, nodes, ref length);
                pathLength[0] = length;
            }
        }

        private int GetNextNode()
        {
            if (openList.Count <= 0)
            {
                return -1;
            }

            int nodeIndex = openList.Pop().Index;
            nextCellNode = nodes[nodeIndex];
            nextCellNode.isInOpenList = false;
            nextCellNode.isInClosedList = true;
            nodes[nodeIndex] = nextCellNode;
            return nodeIndex;
        }

        private void AddStartNodeToOpen(int childIndex)
        {
            AddToOpen(childIndex, heuristic[childIndex]);
        }

        private void AddToOpen(int childIndex, double priority, int parentIndex = -1, double gValue = 0)
        {
            cellNode = nodes[childIndex];

            cellNode.priority = priority;
            cellNode.gValue = gValue;
            cellNode.parentNode = parentIndex;
            
            if (cellNode.isInOpenList) //if node is already in open list, remove the entry to update it
            {
                 heapNode.Index = childIndex;
                 heapNode.Priority = priority;
                 openList.UpdateNode(heapIndex[childIndex], heapNode);
            }
            else
            {
                cellNode.isInOpenList = true;

                heapNode.Index = childIndex;
                heapNode.Priority = priority;
                heapIndex.Add(childIndex, openList.Insert(heapNode));
            }

            nodes[childIndex] = cellNode;
        }
    }
}