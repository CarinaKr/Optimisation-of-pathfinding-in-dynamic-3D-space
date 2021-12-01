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
    public struct Theta_StarHybridJob : IJob, IPathfinderJob
    {
        public NativeHeap<HeapNode, HeapNodePriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeArray<HybridNode> nodes;
        public NativeArray<HybridNodePathCosts> nodePathCosts;
        public NativeArray<int> usedNodes;
        public NativeArray<int> lastGoal;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<int> changes;
        public NativeArray<bool> update;
        public NativeArray<float> pathLength;
        public NativeArray<double> heuristic;
        public float losOffsetMultiplier;
        public bool fastMode;
        public NativeList<float> losTimes;
        public NativeArray<int> lastStart;
        public bool resetAlgorithm;
        

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
        private HybridNode tempNode, cellNode, nextCellNode;
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
            if (resetAlgorithm)
            {
                lastStart[0] = startNode;
                lastStart[1] = lastStart[0];
            }
            // startNode = 39361;
            
            //if there is no line of sight between the aiPosition and the lookaheadPosition, then take the aiPosition as start node
            if (startNode == -1 || !nodePathCosts[startNode].isFree || (startNode == lastStart[0] && !LineOfSightHelper.LineOfSight(nodes[lastStart[1]].localPosition, nodes[startNode].localPosition, searchSpace, losOffsetMultiplier, nodePathCosts, losTimes)))
            {
                startNode = CellGridManagerHelper.GetClosestNode(aiPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);;
            }
            
            if (startNode == -1)
            {
                return;
            }

            if (lastStart[0] != startNode)
            {
                lastStart[1] = lastStart[0];
                lastStart[0] = startNode;
            }
            
            if (lastGoal[0] != goalNode)
            {
                lastGoal[0] = goalNode;
                PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition, pathfinderSetup.Epsilon, ref heuristic);
            }
            
            AddStartNodeToOpen(startNode);
            
            int nextNode = GetNextNode();
            if(nextNode != goalNode)
            {
                expanded[0]++;
                AStarBasedHybridPathfindingHelper.AddNeighboursToOpen(nextNode, nodePathCosts[nextNode].pathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                AStarBasedHybridPathfindingHelper.AddEdgesToOpen(nextNode, nodePathCosts[nextNode].edgePathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                AStarBasedHybridPathfindingHelper.AddVerticesToOpen(nextNode, nodePathCosts[nextNode].verticesPathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                nextNode = GetNextNode();
            }
            
            while (nextNode != goalNode && nextNode != -1)
            {
                expanded[0]++;

                if(fastMode && nodes[nextNode].isDynamicNode)
                {
                    AStarBasedHybridPathfindingHelper.AddNeighboursToOpen(nextNode, nodePathCosts[nextNode].pathCosts, ref nodes, ref heuristic, ref openList, ref heapIndex);
                    AStarBasedHybridPathfindingHelper.AddEdgesToOpen(nextNode, nodePathCosts[nextNode].edgePathCosts,  ref nodes, ref heuristic, ref openList, ref heapIndex);
                    AStarBasedHybridPathfindingHelper.AddVerticesToOpen(nextNode, nodePathCosts[nextNode].verticesPathCosts,  ref nodes, ref heuristic, ref openList, ref heapIndex);
                }
                else
                {
                    AddNeighboursToOpen(nextNode);
                    AddEdgesToOpen(nextNode);
                    AddVerticesToOpen(nextNode);
                    
                }

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

        private void AddNeighboursToOpen(int nodeIndex)
        {
            tempNode = nodes[nodeIndex];
            
            double3x2 heuristics = AStarBasedHybridPathfindingHelper.GetHeuristic(tempNode.neighbours, ref heuristic);
            double3x2 oldPriority = AStarBasedHybridPathfindingHelper.GetOldPriority(tempNode.neighbours, ref nodes);
            bool3x2 isInClosed = AStarBasedHybridPathfindingHelper.GetIsInClosed(tempNode.neighbours, ref nodes);
            
            double3x2 baseGValues = tempNode.gValue;
            double3x2 pathCosts = nodePathCosts[nodeIndex].pathCosts;
            int3x2 newParents = nodeIndex;
            FillPathCostsAndParents(nodeIndex, tempNode.neighbours, tempNode.parentNode, ref baseGValues, ref newParents, ref pathCosts);
            
            double3x2 gValues = baseGValues + pathCosts;
            double3x2 fValues = gValues + heuristics;
            
            bool3x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;
            
            for (int i = 0; i < tempNode.neighboursCount; i++)
            {
                if (addToOpen[i%2][i/2])
                {
                    AddToOpen(tempNode.neighbours[i%2][i/2], fValues[i%2][i/2], newParents[i%2][i/2], gValues[i%2][i/2]);
                }
            }
        }
        
        private void AddEdgesToOpen(int nodeIndex)
        {
            tempNode = nodes[nodeIndex];
            
            double4x3 heuristics = AStarBasedHybridPathfindingHelper.GetHeuristic(tempNode.edgeNeighbours, ref heuristic);
            double4x3 oldPriority = AStarBasedHybridPathfindingHelper.GetOldPriority(tempNode.edgeNeighbours, ref nodes);
            bool4x3 isInClosed = AStarBasedHybridPathfindingHelper.GetIsInClosed(tempNode.edgeNeighbours, ref nodes);
            
            double4x3 baseGValues = tempNode.gValue;
            double4x3 pathCosts = nodePathCosts[nodeIndex].edgePathCosts;
            int4x3 newParents = nodeIndex;
            FillPathCostsAndParents(nodeIndex, tempNode.edgeNeighbours, tempNode.parentNode, ref baseGValues, ref newParents, ref pathCosts);
            
            double4x3 gValues = baseGValues + pathCosts;
            double4x3 fValues = gValues + heuristics;
            
            bool4x3 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;
            
            for (int i = 0; i < tempNode.edgesCount; i++)
            {
                if (addToOpen[i%3][i/3])
                {
                    AddToOpen(tempNode.edgeNeighbours[i%3][i/3], fValues[i%3][i/3], newParents[i%3][i/3], gValues[i%3][i/3]);
                }
            }
        }
        
        private void AddVerticesToOpen(int nodeIndex)
        {
            tempNode = nodes[nodeIndex];
            
            double4x2 heuristics = AStarBasedHybridPathfindingHelper.GetHeuristic(tempNode.verticesNeighbours, ref heuristic);
            double4x2 oldPriority = AStarBasedHybridPathfindingHelper.GetOldPriority(tempNode.verticesNeighbours, ref nodes);
            bool4x2 isInClosed = AStarBasedHybridPathfindingHelper.GetIsInClosed(tempNode.verticesNeighbours, ref nodes);
            
            double4x2 baseGValues = tempNode.gValue;
            double4x2 pathCosts = nodePathCosts[nodeIndex].verticesPathCosts;
            int4x2 newParents = nodeIndex;
            FillPathCostsAndParents(nodeIndex, tempNode.verticesNeighbours, tempNode.parentNode, ref baseGValues, ref newParents, ref pathCosts);
            
            double4x2 gValues = baseGValues + pathCosts;
            double4x2 fValues = gValues + heuristics;
            
            bool4x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;
            
            for (int i = 0; i < tempNode.verticesCount; i++)
            {
                if (addToOpen[i%2][i/2])
                {
                    AddToOpen(tempNode.verticesNeighbours[i%2][i/2], fValues[i%2][i/2], newParents[i%2][i/2], gValues[i%2][i/2]);
                }
            }
        }
  
        private void FillPathCostsAndParents(int nodeIndex, int3x2 neighbours, int nodeParentIndex, ref double3x2 baseGValue, ref int3x2 newParents, ref double3x2 pathCosts)
        {
            HybridNode node = nodes[nodeIndex];
            HybridNode parent = nodes[nodeParentIndex];
            HybridNode neighbour;

            for (int m = 0; m < node.neighboursCount; m++)
            {
                int i = m % 2;
                int j = m / 2;
                neighbour = nodes[neighbours[i][j]];
                
                //neighbour is blocked or outside of grid
                if (!nodePathCosts[nodeIndex].isFree || neighbour.isInClosedList)
                {
                    continue;
                }
                
                //check if neighbour, node, and parent are on a straight line
                bool4 direction = false;
                direction.yzw = parent.localPosition == neighbour.localPosition;
                direction.yzw &= node.localPosition == neighbour.localPosition;
                int dir = math.bitmask(direction);

                //neighbour is on a straight line from the parent of the current node or passes a line-of-sight check
                if (math.countbits(dir) == 2 || LineOfSightHelper.LineOfSight(parent.localPosition, neighbour.localPosition, searchSpace,losOffsetMultiplier, nodePathCosts, losTimes))
                {
                    newParents[i][j] = nodeParentIndex;
                    float3 diff = (parent.globalPosition - neighbour.globalPosition);
                    pathCosts[i][j] = math.sqrt(math.csum(diff * diff));
                    
                    baseGValue[i][j] = parent.gValue;
                }
            }
        }
        
        private void FillPathCostsAndParents(int nodeIndex, int4x3 neighbours, int nodeParentIndex, ref double4x3 baseGValue, ref int4x3 newParents, ref double4x3 pathCosts)
        {
            HybridNode node = nodes[nodeIndex];
            HybridNode parent = nodes[nodeParentIndex];
            HybridNode neighbour;

            for(int m = 0; m<node.edgesCount; m++)
            {
                int i = m % 3;
                int j = m / 3;
                neighbour = nodes[neighbours[i][j]];
                
                if (double.IsPositiveInfinity(nodePathCosts[nodeIndex].edgePathCosts[i][j]) || neighbour.isInClosedList)
                {
                    continue;
                }

                bool onTheSameLine = math.normalize(parent.localPosition - node.localPosition).Equals(math.normalize(node.localPosition - neighbour.localPosition));
                if (onTheSameLine || LineOfSightHelper.LineOfSight(parent.localPosition, neighbour.localPosition, searchSpace,losOffsetMultiplier, nodePathCosts, losTimes))
                {
                    newParents[i][j] = nodeParentIndex;
                    float3 diff = (parent.globalPosition - neighbour.globalPosition);
                    pathCosts[i][j] = math.sqrt(math.csum(diff * diff));
                    
                    baseGValue[i][j] = parent.gValue;
                }
            }
        }
        
        private void FillPathCostsAndParents(int nodeIndex, int4x2 neighbours, int nodeParentIndex, ref double4x2 baseGValue, ref int4x2 newParents, ref double4x2 pathCosts)
        {
            HybridNode node = nodes[nodeIndex];
            HybridNode parent = nodes[nodeParentIndex];
            HybridNode neighbour;

            for(int m = 0;m<node.verticesCount;m++)
            {
                int i = m % 2;
                int j = m / 2;
                neighbour = nodes[neighbours[i][j]];
                
                //neighbour is blocked or outside of grid
                if (double.IsPositiveInfinity(nodePathCosts[nodeIndex].verticesPathCosts[i][j]) || neighbour.isInClosedList)
                {
                    continue;
                }

                //neighbour is on a straight line from the parent of the current node or passes a line-of-sight check
                bool onTheSameLine = math.normalize(parent.localPosition - node.localPosition).Equals(math.normalize(node.localPosition - neighbour.localPosition));
                if (onTheSameLine || LineOfSightHelper.LineOfSight(parent.localPosition, neighbour.localPosition, searchSpace,losOffsetMultiplier, nodePathCosts, losTimes))
                {
                    newParents[i][j] = nodeParentIndex;
                    float3 diff = (parent.globalPosition - neighbour.globalPosition);
                    pathCosts[i][j] = math.sqrt(math.csum(diff * diff));
                    
                    baseGValue[i][j] = parent.gValue;
                }
            }   
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
