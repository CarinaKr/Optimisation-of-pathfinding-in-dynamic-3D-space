using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Profiling;
using Util;

namespace Pathfinding.Algorithms
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct A_StarJob : IJob, IPathfinderJob
    {
        public NativeHeap<CellNode, AStarBasedPriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeArray<CellNode> nodes;
        public NativeArray<int> lastGoal;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<int> changes;
        public NativeArray<bool> update;
        public NativeArray<float> pathLength;
        public NativeArray<double> heuristic;
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
        private CellNode tempCellNode, cellNode, nextCellNode;
        private double3x2 heuristics;
        private bool3x2 isInClosed;
        private double3x2 oldPriority;
        private double3x2 gValues, fValues;
        private int2 pos;
        
        public void Execute()
        {
            expanded[0] = 0; 
            goalNode = CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
            
            if (lastGoal[0] == goalNode && changes.Length == 0) //TODO actually you'd need to check if the new goal node is on the current path, in which case there is no need to update
            {
                return;
            }
            update[0] = true;
            
            startNode = CellGridManagerHelper.GetClosestNode(lookaheadPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
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

                AddNeighboursToOpen(nextNode);

                nextNode = GetNextNode();
            } 

            if (nextNode != -1)
            {
                 corridor.Clear();
                 float length = 0;
                 PathfinderHelper.FillCorridor(goalNode, true, ref corridor, nodes);//, ref length);
                 pathLength[0] = length;
            }
        }
        
        private int GetNextNode()
        {
            if (openList.Count <= 0)
            {
                return -1;
            }
            int nodeIndex = openList.Pop().index;
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
        
        private void AddNeighboursToOpen(int nodeIndex)
        {
            tempCellNode = nodes[nodeIndex];
            FillHeuristic(tempCellNode.neighbours);
            FillIsInClosed(tempCellNode.neighbours);
            FillOldPriority(tempCellNode.neighbours);
            
            gValues = tempCellNode.gValue + tempCellNode.pathCosts;
            fValues = gValues + heuristics;

            bool3x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;
            
            for (int i = 0; i < neighboursCount; i++)
            {
                if (addToOpen[i%2][i/2])
                {
                    AddToOpen(tempCellNode.neighbours[i%2][i/2], fValues[i%2][i/2], nodeIndex, gValues[i%2][i/2]);
                }
            }
        }

        private void FillHeuristic(int3x2 neighbours)
        {
            heuristics[0][0] = heuristic[neighbours[0][0]];
            heuristics[1][0] = heuristic[neighbours[1][0]];
            heuristics[0][1] = heuristic[neighbours[0][1]];
            heuristics[1][1] = heuristic[neighbours[1][1]];
            heuristics[0][2] = heuristic[neighbours[0][2]];
            heuristics[1][2] = heuristic[neighbours[1][2]];
        }

        private void FillIsInClosed(int3x2 neighbours)
        {
            isInClosed[0][0] = nodes[neighbours[0][0]].isInClosedList;
            isInClosed[1][0] = nodes[neighbours[1][0]].isInClosedList;
            isInClosed[0][1] = nodes[neighbours[0][1]].isInClosedList;
            isInClosed[1][1] = nodes[neighbours[1][1]].isInClosedList;
            isInClosed[0][2] = nodes[neighbours[0][2]].isInClosedList;
            isInClosed[1][2] = nodes[neighbours[1][2]].isInClosedList;
        }

        private void FillOldPriority(int3x2 neighbours)
        {
            oldPriority[0][0] = nodes[neighbours[0][0]].priority;
            oldPriority[1][0] = nodes[neighbours[1][0]].priority;
            oldPriority[0][1] = nodes[neighbours[0][1]].priority;
            oldPriority[1][1] = nodes[neighbours[1][1]].priority;
            oldPriority[0][2] = nodes[neighbours[0][2]].priority;
            oldPriority[1][2] = nodes[neighbours[1][2]].priority;
        }
        
        private void AddToOpen(int childIndex, double priority, int parentIndex = -1, double gValue = 0)
        {
            cellNode = nodes[childIndex];
            
            if (cellNode.isInOpenList)  //if node is already in open list, remove the entry to update it
            {
                openList.Remove(heapIndex[childIndex]);
                heapIndex.Remove(childIndex);
            }
            
            cellNode.priority = priority;
            cellNode.gValue = gValue;
            cellNode.parentNode = parentIndex;
            
            cellNode.isInOpenList = true;

            heapIndex.Add(childIndex, openList.Insert(cellNode));
            
            nodes[childIndex] = cellNode;
        }

        
    }
}
