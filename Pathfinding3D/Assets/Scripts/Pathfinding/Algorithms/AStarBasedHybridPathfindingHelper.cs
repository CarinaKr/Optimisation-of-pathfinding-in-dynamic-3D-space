using Pathfinding.Nodes;
using Unity.Collections;
using Unity.Mathematics;
using Util;

namespace Pathfinding.Algorithms
{
    public static class AStarBasedHybridPathfindingHelper
    {
        public static void AddNeighboursToOpen(int nodeIndex, double3x2 pathCosts, ref NativeArray<HybridNode> nodes, ref NativeArray<double> heuristic, ref NativeHeap<HeapNode, HeapNodePriority> openList, ref NativeHashMap<int, NativeHeapIndex> heapIndex)
        {
            HybridNode tempNode = nodes[nodeIndex];
            int3x2 neighbours = tempNode.neighbours;

            double3x2 heuristics = GetHeuristic(neighbours, ref heuristic);
            double3x2 oldPrio = GetOldPriority(neighbours, ref nodes);
            bool3x2 isInClosed = GetIsInClosed(neighbours, ref nodes);

            double3x2 gValues = tempNode.gValue + pathCosts;
            double3x2 fValues = gValues + heuristics;

            bool3x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPrio;

            for (int i = 0; i < 6; i++)
            {
                if (addToOpen[i % 2][i / 2])
                {
                    AddToOpen(ref nodes, ref openList, ref heapIndex, neighbours[i % 2][i / 2], fValues[i % 2][i / 2], nodeIndex, gValues[i % 2][i / 2]);
                }
            }
        }
        
        public static void AddEdgesToOpen(int nodeIndex, double4x3 edgePathCosts, ref NativeArray<HybridNode> nodes, ref NativeArray<double> heuristic, ref NativeHeap<HeapNode, HeapNodePriority> openList, ref NativeHashMap<int, NativeHeapIndex> heapIndex)
        {
            HybridNode tempNode = nodes[nodeIndex];
            //
            double4x3 heuristics = GetHeuristic(tempNode.edgeNeighbours, ref heuristic);
            double4x3 oldPrio = GetOldPriority(tempNode.edgeNeighbours, ref nodes);
            bool4x3 isInClosed = GetIsInClosed(tempNode.edgeNeighbours, ref nodes);
            
            double4x3 edgeGValues = tempNode.gValue + edgePathCosts;
            double4x3 edgeFValues = edgeGValues + heuristics;
            
            bool4x3 edgeAddToOpen = !isInClosed;
            edgeAddToOpen &= edgeFValues < oldPrio;
            
            for (int i = 0; i < 12; i++)
            {
                if (edgeAddToOpen[i%3][i/3])
                {
                    AddToOpen(ref nodes, ref openList, ref heapIndex,tempNode.edgeNeighbours[i%3][i/3], edgeFValues[i%3][i/3], nodeIndex, edgeGValues[i%3][i/3]);
                }
            }
        }

        public static void AddVerticesToOpen(int nodeIndex, double4x2 verticesPathCosts, ref NativeArray<HybridNode> nodes, ref NativeArray<double> heuristic, ref NativeHeap<HeapNode, HeapNodePriority> openList, ref NativeHashMap<int, NativeHeapIndex> heapIndex)
        {
            HybridNode tempNode = nodes[nodeIndex];

            double4x2 heuristics = GetHeuristic(tempNode.verticesNeighbours, ref heuristic);
            double4x2 oldPrio = GetOldPriority(tempNode.verticesNeighbours, ref nodes);
            bool4x2 isInClosed = GetIsInClosed(tempNode.verticesNeighbours, ref nodes);
            
            double4x2 vertexGValues = tempNode.gValue + verticesPathCosts;
            double4x2 vertexFValues = vertexGValues + heuristics;
            
            bool4x2 vertexAddToOpen = !isInClosed;
            vertexAddToOpen &= vertexFValues < oldPrio;
            
            for (int i = 0; i < 8; i++)
            {
                if (vertexAddToOpen[i%2][i/2])
                {
                    AddToOpen(ref nodes, ref openList, ref heapIndex, tempNode.verticesNeighbours[i%2][i/2], vertexFValues[i%2][i/2], nodeIndex, vertexGValues[i%2][i/2]);
                }
            }
        }

        public static double3x2 GetHeuristic(int3x2 neighbours, ref NativeArray<double> heuristic)
        {
            double3x2 heuristics = math.INFINITY_DBL;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    heuristics[i][j] = heuristic[neighbours[i][j]];
                }
            }

            return heuristics;
        }

        public static double4x3 GetHeuristic(int4x3 neighbours, ref NativeArray<double> heuristic)
        {
            double4x3 heuristics = math.INFINITY_DBL;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    heuristics[i][j] = heuristic[neighbours[i][j]];
                }
            }

            return heuristics;
        }

        public static double4x2 GetHeuristic(int4x2 neighbours, ref NativeArray<double> heuristic)
        {
            double4x2 heuristics = math.INFINITY_DBL;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    heuristics[i][j] = heuristic[neighbours[i][j]];
                }
            }

            return heuristics;
        }

        public static bool3x2 GetIsInClosed(int3x2 neighbours, ref NativeArray<HybridNode> nodes)
        {
            bool3x2 isInClosedMatrix = false;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    isInClosedMatrix[i][j] = nodes[neighbours[i][j]].isInClosedList;
                }
            }

            return isInClosedMatrix;
        }

        public static bool4x3 GetIsInClosed(int4x3 neighbours, ref NativeArray<HybridNode> nodes)
        {
            bool4x3 isInClosedMatrix = false;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    isInClosedMatrix[i][j] = nodes[neighbours[i][j]].isInClosedList;
                }
            }

            return isInClosedMatrix;
        }

        public static bool4x2 GetIsInClosed(int4x2 neighbours, ref NativeArray<HybridNode> nodes)
        {
            bool4x2 isInClosedMatrix = false;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    isInClosedMatrix[i][j] = nodes[neighbours[i][j]].isInClosedList;
                }
            }

            return isInClosedMatrix;
        }

        public static double3x2 GetOldPriority(int3x2 neighbours, ref NativeArray<HybridNode> nodes)
        {
            double3x2 oldPriorityMatrix = -1;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    oldPriorityMatrix[i][j] = nodes[neighbours[i][j]].priority;
                }
            }

            return oldPriorityMatrix;
        }

        public static double4x3 GetOldPriority(int4x3 neighbours, ref NativeArray<HybridNode> nodes)
        {
            double4x3 oldPriorityMatrix = -1;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    oldPriorityMatrix[i][j] = nodes[neighbours[i][j]].priority;
                }
            }

            return oldPriorityMatrix;
        }

        public static double4x2 GetOldPriority(int4x2 neighbours, ref NativeArray<HybridNode> nodes)
        {
            double4x2 oldPriorityMatrix = -1;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    oldPriorityMatrix[i][j] = nodes[neighbours[i][j]].priority;
                }
            }

            return oldPriorityMatrix;
        }
        
        private static void AddToOpen(ref NativeArray<HybridNode> nodes, ref NativeHeap<HeapNode, HeapNodePriority> openList, ref NativeHashMap<int, NativeHeapIndex> heapIndex,  int childIndex, double priority, int parentIndex = -1, double gValue = 0)
        {
            HybridNode hybridNode = nodes[childIndex];
            HeapNode heapNode;

            hybridNode.priority = priority;
            hybridNode.gValue = gValue;
            hybridNode.parentNode = parentIndex;
            
            if (hybridNode.isInOpenList) //if node is already in open list, remove the entry to update it
            {
                heapNode.Index = childIndex;
                heapNode.Priority = priority;
                openList.UpdateNode(heapIndex[childIndex], heapNode);
            }
            else
            {
                hybridNode.isInOpenList = true;

                heapNode.Index = childIndex;
                heapNode.Priority = priority;
                heapIndex.Add(childIndex, openList.Insert(heapNode));
            }

            nodes[childIndex] = hybridNode;
        }
    }
}
