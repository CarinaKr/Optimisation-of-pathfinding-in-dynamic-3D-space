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
    public struct Theta_StarJob : IJob, IPathfinderJob
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
        public NativeList<float> losTimes;
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
        private CellNode tempCellNode, cellNode, nextCellNode;
        private double3x2 heuristics;
        private bool3x2 isInClosed;
        private double3x2 oldPriority;
        private double3x2 gValues, fValues;
        private int3x2 newParents;
        private double3x2 pathCosts;
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
            
            //if there is no line of sight between the aiPosition and the lookaheadPosition, then take the aiPosition as start node
            int aiPositionNode = CellGridManagerHelper.GetClosestNode(aiPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
            if (!LineOfSight(nodes[aiPositionNode].localPosition, nodes[startNode].localPosition))
            {
                startNode = aiPositionNode;
            }
            
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
            expanded[0]++;
            AddNeighboursOfStartToOpen(nextNode);
            nextNode = GetNextNode();
            
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

        private void AddNeighboursOfStartToOpen(int nodeIndex)
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
        
        private void AddNeighboursToOpen(int nodeIndex)
        {
            tempCellNode = nodes[nodeIndex];
            
            FillHeuristic(tempCellNode.neighbours);
            FillIsInClosed(tempCellNode.neighbours);
            FillOldPriority(tempCellNode.neighbours);

            double3x2 baseGValues = tempCellNode.gValue;
            pathCosts = nodes[nodeIndex].pathCosts;
            newParents = nodeIndex;
            FillPathCostsAndParents(nodeIndex, tempCellNode.neighbours, tempCellNode.parentNode, ref baseGValues);

            gValues = baseGValues + pathCosts;
            fValues = gValues + heuristics;

            bool3x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;

            for (int i = 0; i < neighboursCount; i++)
            {
                if (addToOpen[i%2][i/2])
                {
                    AddToOpen(tempCellNode.neighbours[i%2][i/2], fValues[i%2][i/2], newParents[i%2][i/2], gValues[i%2][i/2]);
                }
            }
        }

        private void FillPathCostsAndParents(int nodeIndex, int3x2 neighbours, int nodeParentIndex, ref double3x2 baseGValue)
        {
            CellNode node = nodes[nodeIndex];
            CellNode parent = nodes[nodeParentIndex];
            CellNode neighbour;

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    neighbour = nodes[neighbours[i][j]];
                    
                    //neighbour is blocked or outside of grid
                    if (node.pathCosts[i][j].Equals(math.INFINITY_DBL) || !neighbour.isFree || neighbour.isInClosedList)
                    {
                        continue;
                    }
                    
                    //check if neighbour, node, and parent are on a straight line
                    bool4 direction = false;
                    direction.yzw = parent.localPosition == neighbour.localPosition;
                    direction.yzw &= node.localPosition == neighbour.localPosition;
                    int dir = math.bitmask(direction);

                    //neighbour is on a straight line from the parent of the current node or passes a line-of-sight check
                    if (math.countbits(dir) == 2 || LineOfSight(parent.localPosition, neighbour.localPosition))
                    {
                        newParents[i][j] = nodeParentIndex;
                        float3 diff = (parent.globalPosition - neighbour.globalPosition);
                        pathCosts[i][j] = math.sqrt(math.csum(diff * diff));
                        
                        baseGValue[i][j] = parent.gValue;
                    }
                }
            }
        }

        private bool LineOfSight(int3 fromNode, int3 toNode)
        {
            float3 direction = toNode - fromNode;
            float3 directionAbs = math.abs(direction);
            
            float maxAbs = math.cmax(directionAbs);
            //line is direct diagonal in 2 or 3 dimensions
            if (math.all(directionAbs == 0 | directionAbs == maxAbs))
            {
                return DiagonalLOS(maxAbs, fromNode, direction);
            }
            return GeneralLOS(fromNode, direction, directionAbs);
            
        }

        private bool DiagonalLOS(float maxAbs, int3 fromNode, float3 direction)
        {
            int3 lastPosition = fromNode;
            int3 position = fromNode;
            int index;
            int3 checkDirection = (int3)math.sign(direction);
            for(int i = 0; i < maxAbs; i++)
            {
                position += checkDirection;
                index = position.PositionToIndexNoSafety(searchSpace);
                if (!nodes[index].isFree)
                {
                    return false;
                }
                
                for (int m = 0; m < 3; m++)
                {
                    int3 dir = 0;
                    dir[m] = checkDirection[m];
                    index = (lastPosition + dir).PositionToIndexNoSafety(searchSpace);
                    if (!nodes[index].isFree)
                    {
                        return false;
                    }

                    
                    dir[m] *= -1;
                    index = (position + dir).PositionToIndexNoSafety(searchSpace);
                    if (!nodes[index].isFree)
                    {
                        return false;
                    }
                }
                
                lastPosition = position;
            }
            
            return true;
        }

        private bool GeneralLOS(int3 fromNode, float3 direction, float3 directionAbs)
        {
            losTimes.Clear();
            
            int3 position = fromNode;
            int index;
            float t;
            float offset = losOffsetMultiplier / math.length(direction);
            
            for (int k = 0; k < 3; k++)
            {
                for (int i = 0; i < directionAbs[k]; i++)
                {
                    t = math.mad((0.5f + i), 1/directionAbs[k], offset);
                    losTimes.Add(t);
                }
            }
            
            int3 lastPosition;
            losTimes.Sort();
            bool checkDiagonal;
            for(int i = 0; i < losTimes.Length; i++)
            {
                t = losTimes[i];
                
                lastPosition = position;
                position = (int3) math.round(math.mad(t, direction, fromNode));
                checkDiagonal = lastPosition.Equals(position);
                    
                index = position.PositionToIndexNoSafety(searchSpace);
                if (!nodes[index].isFree)
                {
                    return false;
                }

                if (!checkDiagonal)
                {
                    continue;
                }

                lastPosition = (int3) math.round(math.mad(t-(2*offset), direction, fromNode));
                int3 checkDirection = (position - lastPosition);
                for (int m = 0; m < 3; m++)
                {
                    if (checkDirection[m] == 0)
                    {
                        continue;
                    }
                    
                    int3 dir = 0;
                    dir[m] = checkDirection[m];
                    index = (lastPosition + dir).PositionToIndexNoSafety(searchSpace);
                    if(!nodes[index].isFree)
                    {
                        return false;
                    }
                    
                    dir[m] *= -1;
                    index = (position + dir).PositionToIndexNoSafety(searchSpace);
                    if(!nodes[index].isFree)
                    {
                        return false;
                    }
                }
            }
            
            return true;
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
