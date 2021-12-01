using System.Collections.Generic;
using System.Linq;
using BovineLabs.Common.Extensions;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using Util;

namespace Pathfinding.Algorithms
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct A_StarJumpPointJob : IJob, IPathfinderJob
    {
        public NativeHeap<CellNode, AStarBasedPriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeArray<CellNode> nodes;
        public NativeArray<int> lastGoal;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<bool> update;
        // public NativeArray<float> pathLength;
        public NativeArray<int> changes;

        public float3 aiPosition { get; set; }
        public float3 lookaheadPosition { get; set; }
        public float3 goalPosition { get; set; }
        public float3 searchGraphOrigin { get; set; }
        public float cellSize { get; set; }
        public int3 searchSpace { get; set; }
        public PathfinderSetup pathfinderSetup { get; set; }
        public int2 neighboursCountSize { get; set; }
        public int neighboursCount { get; set; }
        public NativeList<int> neighbourIndices;
        public NativeList<int> successor;
        public NativeHashMap<int, int3x2> forcedNeighboursOfNode;
        public NativeHashMap<int, int3x2> jumpPointsOfNode;
        public NativeArray<int3> normalizedDirections;
        public NativeArray<double> heuristic;

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
            expanded[0]++;
            
            neighbourIndices.Clear();
            for (int i = 0; i < nodes[nextNode].neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                if (nodes[nodes[nextNode].neighbours[pos.x][pos.y]].isFree)
                {
                    neighbourIndices.Add(nodes[nextNode].neighbours[pos.x][pos.y]);
                }
            }
            GetSuccessors(nextNode);
            AddJumpPointsToOpen(nextNode, successor);

            nextNode = GetNextNode();
            while (nextNode != goalNode && nextNode != -1)
            {
                expanded[0]++;

                neighbourIndices.Clear();
                GetRemainingNeighbours(nextNode);

                GetSuccessors(nextNode);
                
                AddJumpPointsToOpen(nextNode, successor);

                nextNode = GetNextNode();
            } 

            if (nextNode != -1)
            {
                 corridor.Clear();
                 // float length = 0;
                 PathfinderHelper.FillCorridor(goalNode, true, ref corridor, nodes);//, ref length);
                 // pathLength[0] = length;
            }
        }

        private void GetRemainingNeighbours(int nodeIndex)
        {
            CellNode node = nodes[nodeIndex];

            int3 normalizedDirection = (node.localPosition - nodes[node.parentNode].localPosition).NormalizedDirection();
            
            int naturalNeighbour = (node.localPosition+normalizedDirection).PositionToIndex(searchSpace);
            bool addNaturalNeighbour = true;
            bool hasSavedForcedNeighbours = forcedNeighboursOfNode.TryGetValue(nodeIndex, out int3x2 forcedNeighbours);
            bool hasSavedJumpPoints = jumpPointsOfNode.TryGetValue(nodeIndex, out int3x2 jumpPoints);
            
            if (hasSavedForcedNeighbours)
            {
                for (int i = 0; i < 6; i++)
                {
                    CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                    //if there is a saved forced neighbour in this direction and no jump point in this direction, then add neighbour to the list
                    if (forcedNeighbours[pos.x][pos.y] != -1 && (!hasSavedJumpPoints || jumpPoints[pos.x][pos.y] == -1))
                    {
                        neighbourIndices.Add(forcedNeighbours[pos.x][pos.y]);
                    }
                }
            
                forcedNeighboursOfNode.Remove(nodeIndex);
            }

            //don't add natural neighbour if jump node or forced neighbour already exist in that direction
            for (int i = 0; i < 6; i++)
            {
                if (math.all(normalizedDirections[i] == normalizedDirection))
                {
                    CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                    if ((hasSavedJumpPoints && jumpPoints[pos.x][pos.y] != -1) || (hasSavedForcedNeighbours && forcedNeighbours[pos.x][pos.y] != -1))
                    {
                        addNaturalNeighbour = false;
                        break;
                    }
                }
            }
            
            if (naturalNeighbour != -1 && nodes[naturalNeighbour].isFree && addNaturalNeighbour)
            {
                neighbourIndices.Add(naturalNeighbour);
            }
        }
        
        private bool GetForcedNeighbours(ref int3x2 neighbours, int nodeIndex, int normalizedDirectionIndex)
        {
            //by adding +1 to the normalizedDirectionIndex on the x-value, the direction is inversed getting the neighbour opposite to the normalizedDirection
            int backtrackNodeIndex = nodes[nodeIndex].orderedNeighbours[(normalizedDirectionIndex + 1) % 2][normalizedDirectionIndex / 2];
            bool gotNeighbour = false;

            for (int i = 0; i < 6; i++)
            {
                if(i/2 == normalizedDirectionIndex/2)
                {
                    continue;
                }
                
                int backtrackIndex = nodes[backtrackNodeIndex].orderedNeighbours[i % 2][i / 2];
                int forcedNeighbourIndex = nodes[nodeIndex].orderedNeighbours[i % 2][i / 2];

                if (backtrackIndex != -1 && !nodes[backtrackIndex].isFree && forcedNeighbourIndex != -1 && nodes[forcedNeighbourIndex].isFree)
                {
                    neighbours[i % 2][i / 2] = forcedNeighbourIndex;
                    gotNeighbour = true;
                }
            }
            return gotNeighbour;
        }

        private void GetSuccessors(int nextNode)
        {
            successor.Clear();
            bool findJumpPointsOfNeighbours = true;
            
            if (jumpPointsOfNode.TryGetValue(nextNode, out int3x2 jumpPoints))
            {
                for (int i = 0; i < 6; i++)
                {
                    CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                    if (jumpPoints[pos.x][pos.y] != -1)
                    {
                        successor.Add(jumpPoints[pos.x][pos.y]);
                    }
                }

                if (math.any(jumpPoints.c0 == goalNode) || math.any(jumpPoints.c1 == goalNode))
                {
                    findJumpPointsOfNeighbours = false;
                }
                jumpPointsOfNode.Remove(nextNode);
            }
            
            if(findJumpPointsOfNeighbours)
            {
                for (int i = 0; i < neighbourIndices.Length; i++)
                {
                    int3 normalizedDirection = (nodes[neighbourIndices[i]].localPosition - nodes[nextNode].localPosition).NormalizedDirection();
                    int normalizedDirectionIndex = (nodes[neighbourIndices[i]].localPosition - nodes[nextNode].localPosition).NormalizedDirectionIndex();
                    int jumpPoint = GetJumpPoint(nextNode, normalizedDirectionIndex, normalizedDirection, true);
                    if (jumpPoint != -1 && !successor.Contains(jumpPoint))
                    {
                        successor.Add(jumpPoint);
                    }
                }
            }
        }
        
        private int GetJumpPoint(int nodeIndex, int normalizedDirectionIndex, int3 normalizedDirection, bool3x2 checkDirections)
        {
            int jumpPointNode = nodes[nodeIndex].orderedNeighbours[normalizedDirectionIndex % 2][normalizedDirectionIndex / 2];
            
            //jump point node is blocked or outside of the search are 
            if (jumpPointNode == -1 || !nodes[jumpPointNode].isFree)
            {
                return -1;
            }
            
            if (jumpPointNode == goalNode)
            {
                return jumpPointNode;
            }
            
            int3x2 jumpPointForcedNeighbours = -1;
            if(GetForcedNeighbours(ref jumpPointForcedNeighbours, jumpPointNode, normalizedDirectionIndex))
            {
                AddOrUpdateForcedNeighboursKey(jumpPointNode, jumpPointForcedNeighbours);
                return jumpPointNode;
            }
            
            int3x2 jumpPointsOnLines = -1;

            checkDirections.c0 &= normalizedDirection == 0;
            checkDirections.c1 = checkDirections.c0;

            bool returnJumpPoint = false;
            for (int i = 0; i < 6; i++)
            {
                if (checkDirections[i % 2][i / 2])
                {
                    int jumpPointOnLine = GetJumpPoint(jumpPointNode, i, normalizedDirections[i], checkDirections);
                    jumpPointsOnLines[i%2][i/2] = jumpPointOnLine;
                    returnJumpPoint |= jumpPointOnLine != -1;
                }
            }
            
            if (returnJumpPoint)
            {
                AddOrUpdateNodeJumpPointsKey(jumpPointNode, jumpPointsOnLines);
                return jumpPointNode;
            }
            
            //add support for diagonal here

            return GetJumpPoint(jumpPointNode, normalizedDirectionIndex, normalizedDirection, checkDirections);
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

        private void AddJumpPointsToOpen(int nodeIndex, NativeList<int> successors)
        {
            tempCellNode = nodes[nodeIndex];
            heuristics = math.INFINITY_DBL;
            isInClosed = true;
            oldPriority = math.INFINITY_DBL;
            double3x2 pathCosts = math.INFINITY_DBL;
            float3x2 globalPositionsX = -1, globalPositionsY = -1, globalPositionsZ = -1;
            for (int i = 0; i < successors.Length; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                heuristics[pos.x][pos.y] = heuristic[successors[i]];
                isInClosed[pos.x][pos.y] = nodes[successors[i]].isInClosedList;
                oldPriority[pos.x][pos.y] = nodes[successors[i]].priority;
                globalPositionsX[pos.x][pos.y] = nodes[successors[i]].globalPosition.x;
                globalPositionsY[pos.x][pos.y] = nodes[successors[i]].globalPosition.y;
                globalPositionsZ[pos.x][pos.y] = nodes[successors[i]].globalPosition.z;
            }
            
            float3x2 diffX = nodes[nodeIndex].globalPosition.x - globalPositionsX;
            float3x2 diffY = nodes[nodeIndex].globalPosition.y - globalPositionsY;
            float3x2 diffZ = nodes[nodeIndex].globalPosition.z - globalPositionsZ;
                 
            float3x2 squareDistanceX = diffX * diffX;
            float3x2 squareDistanceY = diffY * diffY;
            float3x2 squareDistanceZ = diffZ * diffZ;
            
            pathCosts.c0 = math.sqrt(squareDistanceX.c0 + squareDistanceY.c0 + squareDistanceZ.c0);
            pathCosts.c1 = math.sqrt(squareDistanceX.c1 + squareDistanceY.c1 + squareDistanceZ.c1);
            
            gValues = tempCellNode.gValue + pathCosts;
            fValues = gValues + heuristics;

            bool3x2 addToOpen = !isInClosed;
            addToOpen &= fValues < oldPriority;

            for (int i = 0; i < neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                if (addToOpen[pos.x][pos.y])
                {
                    AddToOpen(successors[i], fValues[pos.x][pos.y], nodeIndex, gValues[pos.x][pos.y]);
                }
            }
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

        private void AddOrUpdateNodeJumpPointsKey(int node, int3x2 nodeJumpPoints)
        {
            if (jumpPointsOfNode.TryGetValue(node, out int3x2 jumpPoints))
            {
                nodeJumpPoints.c0 = math.max(jumpPoints.c0, nodeJumpPoints.c0);
                nodeJumpPoints.c1 = math.max(jumpPoints.c1, nodeJumpPoints.c1);
                jumpPointsOfNode.Remove(node);
            }
            jumpPointsOfNode.Add(node, nodeJumpPoints);
        }
        
        private void OverwriteNodeJumpPointsKey(int node, int3x2 nodeJumpPoints)
        {
            if (jumpPointsOfNode.ContainsKey(node))
            {
                jumpPointsOfNode.Remove(node);
            }
            jumpPointsOfNode.Add(node, nodeJumpPoints);
        }
        
        private void AddOrUpdateForcedNeighboursKey(int jumpPointNode, int3x2 jumpPointForcedNeighbours)
        {
            if (forcedNeighboursOfNode.TryGetValue(jumpPointNode, out int3x2 forcedNeighbours))
            {
                jumpPointForcedNeighbours.c0 = math.max(forcedNeighbours.c0, jumpPointForcedNeighbours.c0);
                jumpPointForcedNeighbours.c1 = math.max(forcedNeighbours.c1, jumpPointForcedNeighbours.c1);
                forcedNeighboursOfNode.Remove(jumpPointNode);
            }
            forcedNeighboursOfNode.Add(jumpPointNode, jumpPointForcedNeighbours);
        }
        
        private void OverwriteForcedNeighboursKey(int jumpPointNode, int3x2 jumpPointForcedNeighbours)
        {
            if (forcedNeighboursOfNode.TryGetValue(jumpPointNode, out int3x2 forcedNeighbours))
            {
                forcedNeighboursOfNode.Remove(jumpPointNode);
            }
            forcedNeighboursOfNode.Add(jumpPointNode, jumpPointForcedNeighbours);
        }
        
    }
}
