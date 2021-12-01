using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Util;

namespace Pathfinding.Algorithms
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct MTD_StarLiteHybridJob : IJob, IPathfinderJob
    {
        public NativeArray<HybridNode> nodes;
        public NativeArray<HybridNodePathCosts> nodePathCosts;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<int> changes;
        public NativeArray<bool> update;
        public bool resetAlgorithm;
        public bool useBasicMT;
        public NativeArray<float> pathLength;
        public NativeArray<double> heuristic;
        public NativeArray<double> plainHeuristic;
        public int2 edgesCount;
        public int2 verticesCount;
        public NativeMultiHashMap<int, int> predecessorsMap;
        public NativeArray<int> usedNodes;
        public float losOffsetMultiplier;

        public float3 aiPosition { get; set; }
        public float3 lookaheadPosition { get; set; }
        public float3 goalPosition { get; set; }
        public float3 searchGraphOrigin { get; set; }
        public float cellSize { get; set; }
        public int3 searchSpace { get; set; }
        public PathfinderSetup pathfinderSetup { get; set; }
        public int neighboursCount { get; set; }
        public int2 neighboursCountSize { get; set; }

        public NativeHeap<DStarHeapNode, DStarHeapNodePriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeHashSet<int> searchTree;
        public NativeHashSet<int> subTree;

        private int startNode;
        private int goalNode;
        public NativeArray<float> km;
        public NativeArray<int> lastGoal;
        public NativeArray<int> lastStart;
        private int2 pos;
        private DStarHeapNode heapNode;

        private void ResetPathfinding()
        {
            openList.Clear();
            searchTree.Clear();
            heapIndex.Clear();
            // startNode = GetStartNode();
            startNode = CellGridManagerHelper.GetClosestNode(lookaheadPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // startNode = 39361;
            lastStart[0] = startNode;
            HybridNode tempNode = nodes[startNode];
            tempNode.rhsValue = 0;
            nodes[startNode] = tempNode;
            km[0] = 0;
            goalNode =  CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // goalNode = 28859;
            lastGoal[0] = goalNode;
            PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition, pathfinderSetup.Epsilon, ref heuristic, ref plainHeuristic);
            AddToOpen(startNode);
            resetAlgorithm = false;
        }
        
        public void Execute()
        {
            if (resetAlgorithm)
            {
                resetAlgorithm = false;
                ResetPathfinding();
                ShortestPathLoop();
                update[0] = true;
                return;
            }

            goalNode = CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // goalNode = 28859;

            if (lastGoal[0] == goalNode && changes.Length == 0) //TODO actually you'd need to check if the new goal node is on the current path, in which case there is no need to update
            {
                return;
            }
            
            update[0] = true;
            startNode = CellGridManagerHelper.GetClosestNode(lookaheadPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
            // startNode = 39361;
            if (startNode == -1 || !nodePathCosts[startNode].isFree)
            {
                startNode = GetStartNode();
            }

            if (startNode == -1)
            {
                return;
            }

            if (lastGoal[0] != goalNode)
            {
                km[0] += (float) (plainHeuristic[goalNode] - plainHeuristic[lastGoal[0]]) * pathfinderSetup.Epsilon;

                PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition, pathfinderSetup.Epsilon, ref heuristic, ref plainHeuristic);
                lastGoal[0] = goalNode;
            }

            if(lastStart[0] != startNode)
            {
                if (useBasicMT)
                {
                    HybridNode tempNode = nodes[startNode];
                    tempNode.parentNode = -1;
                    nodes[startNode] = tempNode;
                    UpdateNodeRhs(lastStart[0]);
                }
                else if (!useBasicMT)
                {
                    HybridNode tempNode = nodes[startNode];
                    tempNode.parentNode = -1;
                    nodes[startNode] = tempNode;
                    subTree.Clear();
                    FillSubTree(startNode);
                
                    NativeHashSet<int> diffTree = D_starBasedHelper.GetDiffTree(searchTree, subTree);
                
                    NativeArray<int> tempDiffTree = diffTree.ToNativeArray(Allocator.Temp);
                    for (int i = 0; i < tempDiffTree.Length; i++)
                    {
                        HybridNode difNode = nodes[tempDiffTree[i]];
                        difNode.parentNode = -1;
                        difNode.rhsValue = Mathf.Infinity;
                        difNode.gValue = Mathf.Infinity;
                        nodes[tempDiffTree[i]] = difNode;
                
                        RemoveFromOpen(tempDiffTree[i]);
                    }
                
                    for (int i = 0; i < tempDiffTree.Length; i++)
                    {
                        UpdateNodeRhs(tempDiffTree[i]);
                    }
                }
                lastStart[0] = startNode;
            }
            
            for (int i = 0; i < changes.Length; i++)
            {
                UpdateNodeRhs(changes[i]);
            }
            
            ShortestPathLoop();

        }
        
        private void ShortestPathLoop()
        {
            expanded[0] = 0; 
            bool pathFound = true;
            if (startNode == -1)
            {
                startNode = GetStartNode();
            }

            int nextNode = GetNextNode();
        
            if (nodePathCosts[startNode].isFree && nextNode != -1)
            {
                
                while ((!nodes[goalNode].gValue.Equals(nodes[goalNode].rhsValue)) || D_starBasedHelper.IsLowerKey(nextNode, CalculateKeyValue(nodes[goalNode]), nodes))
                {
                    expanded[0]++;

                    HybridNode nextCellNode = nodes[nextNode];
                    double2 oldKey = nextCellNode.priorityKey;
                    double2 newKey = CalculateKeyValue(nextCellNode);
                    if (oldKey.x < newKey[0] || oldKey.x.Equals(newKey[0]) && oldKey.y < newKey[1])
                    {
                        AddToOpen(nextNode);
                    }
                    else if (nextCellNode.gValue > nextCellNode.rhsValue) //overconsistent
                    {
                        nextCellNode.gValue = nextCellNode.rhsValue; //set g-value
                        nodes[nextNode] = nextCellNode;
                        RemoveFromOpen(nextNode);

                        UpdateNeighboursOfNode(nextNode);
                        UpdateEdgesOfNode(nextNode);
                        UpdateVerticesOfNode(nextNode);
                    }
                    else //underconsistent
                    {
                        nextCellNode.gValue = math.INFINITY_DBL; //set g-value to infinite
                        nodes[nextNode] = nextCellNode;
                        UpdateNodeRhs(nextNode);   //update own rhs-value

                        int parentIndex = -1;
                        for(int i = 0; i < nextCellNode.neighboursCount; i++)//update rhs-value of all side successors
                        {
                            CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                            parentIndex = nodes[nextCellNode.neighbours[pos.x][pos.y]].parentNode;
                            if (parentIndex == nextNode || parentIndex == -1)
                            {
                                UpdateNodeRhs(nextCellNode.neighbours[pos.x][pos.y]);
                            }
                        }
                        for(int i = 0; i < nextCellNode.edgesCount; i++)//update rhs-value of all edge successors
                        {
                            CellGridManagerHelper.IndexToPosition(i, edgesCount, ref pos);
                            parentIndex = nodes[nextCellNode.edgeNeighbours[pos.x][pos.y]].parentNode;
                            if (parentIndex == nextNode || parentIndex == -1)
                            {
                                UpdateNodeRhs(nextCellNode.edgeNeighbours[pos.x][pos.y]);
                            }
                        }
                        for(int i = 0; i < nextCellNode.verticesCount; i++)//update rhs-value of all vertex successors
                        {
                            CellGridManagerHelper.IndexToPosition(i, verticesCount, ref pos);
                            parentIndex = nodes[nextCellNode.verticesNeighbours[pos.x][pos.y]].parentNode;
                            if (parentIndex == nextNode || parentIndex == -1)
                            {
                                UpdateNodeRhs(nextCellNode.verticesNeighbours[pos.x][pos.y]);
                            }
                        }
                    }

                    nextNode = GetNextNode();
                    if (nextNode == -1)
                    {
                        pathFound = false;
                    }
                }
        
                if(pathFound)
                {
                    corridor.Clear();
                    float length = 0;
                    PathfinderHelper.FillCorridor(goalNode, true, ref corridor, nodes);//, ref length);
                    pathLength[0] = length;
                }
            }
        }

        private void UpdateNeighboursOfNode(int nodeIndex)
        {
            double3x2 newRhsValues = nodes[nodeIndex].gValue + nodePathCosts[nodeIndex].pathCosts;
            double3x2 oldRhsValues = new double3x2(nodes[nodes[nodeIndex].neighbours[0][0]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][0]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[0][1]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][1]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[0][2]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][2]].rhsValue);
            bool3x2 useNewRhsValue = newRhsValues < oldRhsValues;
            
            for (int i = 0; i < nodes[nodeIndex].neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                if (useNewRhsValue[pos.x][pos.y])
                {
                    HybridNode tempNode = nodes[nodes[nodeIndex].neighbours[pos.x][pos.y]];
                    tempNode.rhsValue = newRhsValues[pos.x][pos.y];
                    tempNode.parentNode = nodeIndex;
                    nodes[nodes[nodeIndex].neighbours[pos.x][pos.y]] = tempNode;
                    UpdateNodeInOpenList(nodes[nodeIndex].neighbours[pos.x][pos.y]);
                }
            }
        }
        
        private void UpdateEdgesOfNode(int nodeIndex)
        {
            HybridNode node = nodes[nodeIndex];
            int4x3 edgeNeighbours = node.edgeNeighbours;
            double4x3 newRhsValues = node.gValue + nodePathCosts[nodeIndex].edgePathCosts;
            double4x3 oldRhsValues = GetOldEdgeRhsValues(edgeNeighbours);
            bool4x3 useNewRhsValue = newRhsValues < oldRhsValues;
            
            for (int i = 0; i < node.edgesCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, edgesCount, ref pos);
                if (useNewRhsValue[pos.x][pos.y])
                {
                    HybridNode tempNode = nodes[edgeNeighbours[pos.x][pos.y]];
                    tempNode.rhsValue = newRhsValues[pos.x][pos.y];
                    tempNode.parentNode = nodeIndex;
                    nodes[edgeNeighbours[pos.x][pos.y]] = tempNode;
                    UpdateNodeInOpenList(edgeNeighbours[pos.x][pos.y]);
                }
            }
        }
        
        private void UpdateVerticesOfNode(int nodeIndex)
        {
            HybridNode node = nodes[nodeIndex];
            int4x2 vertexNeighbours = node.verticesNeighbours;
            double4x2 newRhsValues = node.gValue + nodePathCosts[nodeIndex].verticesPathCosts;
            double4x2 oldRhsValues = GetOldVertexRhsValues(vertexNeighbours);
            bool4x2 useNewRhsValue = newRhsValues < oldRhsValues;
            
            for (int i = 0; i < node.verticesCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, verticesCount, ref pos);
                if (useNewRhsValue[pos.x][pos.y])
                {
                    HybridNode tempNode = nodes[vertexNeighbours[pos.x][pos.y]];
                    tempNode.rhsValue = newRhsValues[pos.x][pos.y];
                    tempNode.parentNode = nodeIndex;
                    nodes[vertexNeighbours[pos.x][pos.y]] = tempNode;
                    UpdateNodeInOpenList(vertexNeighbours[pos.x][pos.y]);
                }
            }
        }

        private void UpdateNodeRhs(int nodeIndex)
        {
            if (nodeIndex == startNode)
            {
                UpdateNodeInOpenList(nodeIndex);
                return;
            }
            
            HybridNode tempNode = nodes[nodeIndex];
            
            if (tempNode.isDynamicNode)
            {
                UpdateDynamicNodeParallel(nodeIndex);
                return;
            }
            
            UpdateNode(nodeIndex);
        }
        
        private void UpdateDynamicNodeParallel(int nodeIndex)
        {
            HybridNode tempNode = nodes[nodeIndex];
            int3x2 predecessors = tempNode.predecessors;
            double3x2 gValueOfPredecessors = new double3x2(nodes[predecessors[0][0]].gValue,
                                                            nodes[predecessors[1][0]].gValue, 
                                                            nodes[predecessors[0][1]].gValue, 
                                                            nodes[predecessors[1][1]].gValue,
                                                            nodes[predecessors[0][2]].gValue, 
                                                            nodes[predecessors[1][2]].gValue);
            double3x2 rhsValues = gValueOfPredecessors + nodePathCosts[nodeIndex].pathCosts;
            
            double neighbourRhsValue = math.min(math.cmin(rhsValues.c0), math.cmin(rhsValues.c1));
            
            int4x3 edgePredecessors = tempNode.edgePredecessors;
            double4x3 edgeGValueOfPredecessors = GetGValues(edgePredecessors);
            double4x3 edgeRhsValues = edgeGValueOfPredecessors + nodePathCosts[nodeIndex].edgePathCosts;
            
            double edgeRhsValue = math.min(math.min(math.cmin(edgeRhsValues.c0), math.cmin(edgeRhsValues.c1)), math.cmin(edgeRhsValues.c2));

            int4x2 vertexPredecessors = tempNode.verticesPredecessors;
            double4x2 vertexGValueOfPredecessors = GetGValues(vertexPredecessors);
            double4x2 vertexRhsValues = vertexGValueOfPredecessors + nodePathCosts[nodeIndex].verticesPathCosts;
            
            double vertexRhsValue = math.min(math.cmin(vertexRhsValues.c0), math.cmin(vertexRhsValues.c1));

            double rhsValue = math.min(neighbourRhsValue, math.min(edgeRhsValue, vertexRhsValue));
            int parent;
            
            if (rhsValue.Equals(math.INFINITY_DBL))
            {
                parent = -1;
            }
            else if (neighbourRhsValue.Equals(rhsValue))
            {
                bool3x2 isParentNode = rhsValues == neighbourRhsValue;
                int3 parentsColumn0 = math.select(new int3(-1), predecessors.c0, isParentNode.c0);
                int3 parentsColumn1 = math.select(new int3(-1), predecessors.c1, isParentNode.c1);
                parent = math.max(math.cmax(parentsColumn0), math.cmax(parentsColumn1));
            }
            else if (edgeRhsValue.Equals(rhsValue))
            {
                bool4x3 isParentNode = edgeRhsValues == edgeRhsValue;
                int4 parentsColumn0 = math.select(new int4(-1), edgePredecessors.c0, isParentNode.c0);
                int4 parentsColumn1 = math.select(new int4(-1), edgePredecessors.c1, isParentNode.c1);
                int4 parentsColumn2 = math.select(new int4(-1), edgePredecessors.c2, isParentNode.c2);
                parent = math.max(math.max(math.cmax(parentsColumn0), math.cmax(parentsColumn1)), math.cmax(parentsColumn2));
            }
            else
            {
                bool4x2 isParentNode = vertexRhsValues == vertexRhsValue;
                int4 parentsColumn0 = math.select(new int4(-1), vertexPredecessors.c0, isParentNode.c0);
                int4 parentsColumn1 = math.select(new int4(-1), vertexPredecessors.c1, isParentNode.c1);
                parent = math.max(math.cmax(parentsColumn0), math.cmax(parentsColumn1));
            }
            
            tempNode.parentNode = parent;
            tempNode.rhsValue = rhsValue;
            nodes[nodeIndex] = tempNode;
            UpdateNodeInOpenList(nodeIndex);
        }

        private void UpdateNode(int nodeIndex)
        {
            double minRHS = math.INFINITY_DBL;
            int bestParentNode = -1;
            HybridNode tempNode = nodes[nodeIndex];
            NativeMultiHashMap<int, int>.Enumerator enumerator = predecessorsMap.GetValuesForKey(nodeIndex);
            while (enumerator.MoveNext())
            {
                int pred = enumerator.Current;
                if (nodes[pred].gValue < minRHS) //rhs=g+c(n,n') only calculate rhs values based on nodes, which have a smaller g-value
                {
                    double rhs = nodes[pred].gValue + GetPathCost(pred, nodeIndex);
                    if (rhs < minRHS)
                    {
                        minRHS = rhs;
                        bestParentNode = pred;
                    }
                }
            }
            enumerator.Dispose();
            
            tempNode.rhsValue = minRHS;
            tempNode.parentNode = bestParentNode;
            nodes[nodeIndex] = tempNode;
            UpdateNodeInOpenList(nodeIndex);
        }

        private double GetPathCost(int fromIndex, int toIndex)
        {
            double pathCost = math.INFINITY_DBL;
            
            bool4 direction = false;
            direction.yzw = nodes[toIndex].localPosition == nodes[fromIndex].localPosition;
            int dir = math.bitmask(direction);
            int dimension = math.countbits(dir);    //dimension = 2 => neighbour, dimension = 1 => edge, dimension = 0 => vertex

            switch (dimension)
            {
                case 2:
                {
                    GetNeighbourIndexInNode(fromIndex, toIndex, out int i, out int j);
                    pathCost = nodePathCosts[fromIndex].pathCosts[i][j];
                    break;
                }
                case 1:
                {
                    GetEdgeIndexInNode(fromIndex, toIndex, out int i, out int j);
                    pathCost = nodePathCosts[fromIndex].edgePathCosts[i][j];
                    break;
                }
                default:
                {
                    GetVertexIndexInNode(fromIndex, toIndex, out int i, out int j);
                    pathCost = nodePathCosts[fromIndex].verticesPathCosts[i][j];
                    break;
                }
            }
            
            return pathCost;
        }

        private void GetNeighbourIndexInNode(int nodeIndex, int neighbourIndex, out int i, out int j)
        {
            j = 0;
            int3x2 neighbours = nodes[nodeIndex].neighbours;
            for (i = 0; i < 2; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    if (neighbours[i][j] == neighbourIndex)
                    {
                        return;
                    }
                }
            }
        }
        private void GetEdgeIndexInNode(int nodeIndex, int neighbourIndex, out int i, out int j)
        {
            j = 0;
            int4x3 neighbours = nodes[nodeIndex].edgeNeighbours;
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 4; j++)
                {
                    if (neighbours[i][j] == neighbourIndex)
                    {
                        return;
                    }
                }
            }
        }
        private void GetVertexIndexInNode(int nodeIndex, int neighbourIndex, out int i, out int j)
        {
            j = 0;
            int4x2 neighbours = nodes[nodeIndex].verticesNeighbours;
            for (i = 0; i < 2; i++)
            {
                for (j = 0; j < 4; j++)
                {
                    if (neighbours[i][j] == neighbourIndex)
                    {
                        return;
                    }
                }
            }
        }

        private void UpdateNodeInOpenList(int nodeIndex)
        {
            if (!nodes[nodeIndex].rhsValue.Equals(nodes[nodeIndex].gValue)) //if node is inconsistent
            {
                AddToOpen(nodeIndex);
            }
            else //if node is consistent
            {
                RemoveFromOpen(nodeIndex);
             
                if (searchTree.Contains(nodeIndex) && double.IsPositiveInfinity(nodes[nodeIndex].gValue))
                {
                    searchTree.Remove(nodeIndex);
                }
            }
        }
        
        private void FillSubTree(int source)
        {
            subTree.Add(source);

            for (int i = 0; i < nodes[source].neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                int pred = nodes[source].predecessors[pos.x][pos.y];
                if (nodes[pred].parentNode == source && !subTree.Contains(pred))
                {
                    FillSubTree(pred);
                }
            }
        }
        
        private int GetStartNode()
        {
            return CellGridManagerHelper.GetClosestNode(aiPosition, searchGraphOrigin, cellSize, searchSpace, nodes, nodePathCosts, usedNodes, neighboursCountSize, losOffsetMultiplier);
        }
        
        private void AddToOpen(int childIndex)
        {
            double2 newKeyValue = CalculateKeyValue(nodes[childIndex]);
            if (newKeyValue[0] >= float.MaxValue)
            {
                return;
            }

            HybridNode cellNode = nodes[childIndex];
            cellNode.priorityKey = newKeyValue;
            if (cellNode.isInOpenList) //if node is already in open list, remove the entry to update it
            {
                heapNode.Index = childIndex;
                heapNode.Priority = newKeyValue;
                openList.UpdateNode(heapIndex[childIndex], heapNode);
            }
            else
            {
                cellNode.isInOpenList = true;

                heapNode.Index = childIndex;
                heapNode.Priority = newKeyValue;
                heapIndex.Add(childIndex, openList.Insert(heapNode));
            }
            
            if (!searchTree.Contains(childIndex))
            {
                searchTree.Add(childIndex);
            }
            nodes[childIndex] = cellNode;
        }

        private void RemoveFromOpen(int nodeIndex)
        {
            HybridNode cellNode = nodes[nodeIndex];
            if (cellNode.isInOpenList)
            {
                cellNode.isInOpenList = false;
                openList.Remove(heapIndex[nodeIndex]);
                heapIndex.Remove(nodeIndex);
            }
            nodes[nodeIndex] = cellNode;
        }

        private int GetNextNode()
        {
            if (openList.TryPeek(out DStarHeapNode node))
            {
                return node.Index;
            }

            return -1;
        }
        
        private double2 CalculateKeyValue(HybridNode child)
        {
            double2 key = 0;
            if (child.gValue > child.rhsValue)
            {
                key[0] = child.rhsValue + heuristic[child.index] + km[0];
                key[1] = child.rhsValue;
            }
            else
            {
                key[0] = child.gValue + plainHeuristic[child.index] + km[0];
                key[1] = child.gValue;
            }

            return key;
        }

        private double4x3 GetOldEdgeRhsValues(int4x3 neighbours)
        {
            double4x3 oldRhsValues = 0;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    oldRhsValues[i][j] = nodes[neighbours[i][j]].rhsValue;
                }
            }
            return oldRhsValues;
        }
        
        private double4x2 GetOldVertexRhsValues(int4x2 neighbours)
        {
            double4x2 oldRhsValues = 0;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    oldRhsValues[i][j] = nodes[neighbours[i][j]].rhsValue;
                }
            }
            return oldRhsValues;
        }

        private double4x3 GetGValues(int4x3 predecessors)
        {
            double4x3 gValues = 0;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    gValues[i][j] = nodes[predecessors[i][j]].gValue;
                }
            }
            return gValues;
        }
        
        private double4x2 GetGValues(int4x2 predecessors)
        {
            double4x2 gValues = 0;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    gValues[i][j] = nodes[predecessors[i][j]].gValue;
                }
            }
            return gValues;
        }
    }
}
