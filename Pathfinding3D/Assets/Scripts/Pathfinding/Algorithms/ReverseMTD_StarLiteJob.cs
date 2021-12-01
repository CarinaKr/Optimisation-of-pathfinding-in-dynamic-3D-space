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
    public struct ReverseMTD_StarLiteJob : IJob, IPathfinderJob
    {
        public NativeArray<CellNode> nodes;
        public NativeArray<CellNode> globalNodes;
        public NativeArray<int> expanded;
        public NativeList<float3> corridor;
        public NativeArray<int> changes;
        public NativeArray<bool> update;
        public bool resetAlgorithm;
        public bool useBasicMT;
        public NativeArray<float> pathLength;

        public float3 aiPosition { get; set; }
        public float3 lookaheadPosition { get; set; }
        public float3 goalPosition { get; set; }
        public float3 searchGraphOrigin { get; set; }
        public float cellSize { get; set; }
        public int3 searchSpace { get; set; }
        public PathfinderSetup pathfinderSetup { get; set; }
        public int neighboursCount { get; set; }
        public int2 neighboursCountSize { get; set; }

        public NativeHeap<CellNode, DStarBasedPriority> openList;
        public NativeHashMap<int, NativeHeapIndex> heapIndex;
        public NativeHashSet<int> searchTree;
        public NativeHashSet<int> subTree;

        private int startNode;
        private int goalNode;
        public NativeArray<float> km;
        public NativeArray<int> lastGoal;
        public NativeArray<int> lastStart;
        private int2 pos;

        private void ResetPathfinding()
        {
            openList.Clear();
            searchTree.Clear();
            heapIndex.Clear();
            startNode = GetStartNode();
            lastStart[0] = startNode;
            CellNode tempNode = nodes[startNode];
            tempNode.rhsValue = 0;
            nodes[startNode] = tempNode;
            km[0] = 0;
            goalNode = CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
            lastGoal[0] = goalNode;
            nodes = PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition, pathfinderSetup.Epsilon);
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

            for (int i = 0; i < changes.Length; i++)
            {
                CellNode tempNode = nodes[changes[i]];
                tempNode.isFree = globalNodes[changes[i]].isFree;
                tempNode.pathCosts = globalNodes[changes[i]].pathCosts;
                nodes[changes[i]] = tempNode;
            }
            
            goalNode = CellGridManagerHelper.GetClosestNode(goalPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);

            if (lastGoal[0] == goalNode && changes.Length == 0) //TODO actually you'd need to check if the new goal node is on the current path, in which case there is no need to update
            {
                return;
            }
            
            update[0] = true;
            startNode = CellGridManagerHelper.GetClosestNode(lookaheadPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
            if (startNode == -1 || !nodes[startNode].isFree)
            {
                startNode = GetStartNode();
            }

            if (startNode == -1)
            {
                return;
            }

            if (lastGoal[0] != goalNode)
            {
                km[0] += (float) (nodes[goalNode].plainHeuristic - nodes[lastGoal[0]].plainHeuristic) *
                         pathfinderSetup.Epsilon;

                nodes = PathfinderHelper.UpdateHeuristic(nodes, nodes[goalNode].globalPosition,
                    pathfinderSetup.Epsilon);
                lastGoal[0] = goalNode;
            }

            if(lastStart[0] != startNode)
            {
                if (useBasicMT)
                {
                    CellNode tempNode = nodes[startNode];
                    tempNode.parentNode = -1;
                    nodes[startNode] = tempNode;
                    UpdateNode(lastStart[0]);
                }
                else if (!useBasicMT)
                {
                    CellNode tempNode = nodes[startNode];
                    tempNode.parentNode = -1;
                    nodes[startNode] = tempNode;
                    subTree.Clear();
                    FillSubTree(startNode);
                
                    NativeHashSet<int> diffTree = D_starBasedHelper.GetDiffTree(searchTree, subTree);
                
                    NativeArray<int> tempDiffTree = diffTree.ToNativeArray(Allocator.Temp);
                    for (int i = 0; i < tempDiffTree.Length; i++)
                    {
                        CellNode difNode = nodes[tempDiffTree[i]];
                        difNode.parentNode = -1;
                        difNode.rhsValue = Mathf.Infinity;
                        difNode.gValue = Mathf.Infinity;
                        nodes[tempDiffTree[i]] = difNode;
                
                        RemoveFromOpen(tempDiffTree[i]);
                    }
                
                    for (int i = 0; i < tempDiffTree.Length; i++)
                    {
                        UpdateNodeParallel(tempDiffTree[i]);
                    }
                }
                lastStart[0] = startNode;
            }
            
            for (int i = 0; i < changes.Length; i++)
            {
                UpdateNodeParallel(changes[i]);
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
        
            if (nodes[startNode].isFree && nextNode != -1)
            {
                
                while ((!nodes[goalNode].gValue.Equals(nodes[goalNode].rhsValue)) || D_starBasedHelper.IsLowerKey(nextNode, CalculateKeyValue(nodes[goalNode]), nodes))
                {
                    expanded[0]++;

                    CellNode nextCellNode = nodes[nextNode];
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
                    }
                    else //underconsistent
                    {
                        nextCellNode.gValue = math.INFINITY_DBL; //set g-value to infinite
                        nodes[nextNode] = nextCellNode;
                        UpdateNodeParallel(nextNode);   //update own rhs-value
                        
                        for(int i = 0; i < nextCellNode.neighboursCount; i++)//update rhs-value of all successors
                        {
                            CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                            if (nodes[nextCellNode.neighbours[pos.x][pos.y]].parentNode == -1 || nodes[nextCellNode.neighbours[pos.x][pos.y]].parentNode == nextNode)
                            {
                                UpdateNodeParallel(nextCellNode.neighbours[pos.x][pos.y]);
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
            double3x2 newRhsValues = nodes[nodeIndex].gValue + nodes[nodeIndex].pathCosts;
            double3x2 oldRhsValues = new double3x2(nodes[nodes[nodeIndex].neighbours[0][0]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][0]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[0][1]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][1]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[0][2]].rhsValue,
                                                    nodes[nodes[nodeIndex].neighbours[1][2]].rhsValue);
            bool3x2 useNewRhsValue = newRhsValues < oldRhsValues;
            
            for (int i = 0; i < neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                if (useNewRhsValue[pos.x][pos.y])
                {
                    CellNode tempNode = nodes[nodes[nodeIndex].neighbours[pos.x][pos.y]];
                    tempNode.rhsValue = newRhsValues[pos.x][pos.y];
                    tempNode.parentNode = nodeIndex;
                    nodes[nodes[nodeIndex].neighbours[pos.x][pos.y]] = tempNode;
                    UpdateNodeInOpenList(nodes[nodeIndex].neighbours[pos.x][pos.y]);
                }
            }
        }
        
        private void UpdateNode(int nodeIndex, int nodeIndexInPredecessor, int predecessor)
        {
            double rhs = D_starBasedHelper.CalculateRHS(nodeIndexInPredecessor, nodes[predecessor], neighboursCountSize);
            if (rhs < nodes[nodeIndex].rhsValue)
            {
                CellNode tempNode = nodes[nodeIndex];
                tempNode.rhsValue = rhs;
                tempNode.parentNode = predecessor;
                nodes[nodeIndex] = tempNode;
                UpdateNodeInOpenList(nodeIndex);
            }
        }

        private void UpdateNodeParallel(int nodeIndex)
        {
            if (nodeIndex == startNode)
            {
                UpdateNodeInOpenList(nodeIndex);
                return;
            }
            
            CellNode tempNode = nodes[nodeIndex];
            int3x2 predecessors = tempNode.predecessors;
            double3x2 gValueOfPredecessors = new double3x2(nodes[predecessors[0][0]].gValue,
                                                            nodes[predecessors[1][0]].gValue, 
                                                            nodes[predecessors[0][1]].gValue, 
                                                            nodes[predecessors[1][1]].gValue,
                                                            nodes[predecessors[0][2]].gValue, 
                                                            nodes[predecessors[1][2]].gValue);
            double3x2 rhsValues = gValueOfPredecessors + tempNode.pathCosts;
            
            double rhsValue = math.min(math.cmin(rhsValues.c0), math.cmin(rhsValues.c1));
            
            bool3x2 isParentNode = rhsValues == rhsValue;
            isParentNode &= rhsValue < math.INFINITY_DBL;
            int3 parentsColumn0 = math.select(new int3(-1), predecessors.c0, isParentNode.c0);
            int3 parentsColumn1 = math.select(new int3(-1), predecessors.c1, isParentNode.c1);
            int parent = math.max(math.cmax(parentsColumn0), math.cmax(parentsColumn1));

            tempNode.parentNode = parent;
            tempNode.rhsValue = rhsValue;
            nodes[nodeIndex] = tempNode;
            UpdateNodeInOpenList(nodeIndex);
        }
        
        private void UpdateNode(int nodeIndex)  //TODO keep this so you can add comparison of both version in the thesis
        {
            //update rhs-value
            if (nodeIndex != startNode)
            {
                double minRHS = math.INFINITY_DBL;
                int bestParentNode = -1;
                CellNode tempNode = nodes[nodeIndex];
                for(int i = 0; i < neighboursCount; i++)
                {
                    CellGridManagerHelper.IndexToPosition(i, neighboursCountSize, ref pos);
                    if (nodes[tempNode.predecessors[pos.x][pos.y]].gValue < minRHS) //rhs=g+c(n,n') only calculate rhs values based on nodes, which have a smaller g-value
                    {
                        double rhs = D_starBasedHelper.CalculateRHS(i, nodes[tempNode.predecessors[pos.x][pos.y]], neighboursCountSize);
                        if (rhs < minRHS)
                        {
                            minRHS = rhs;
                            bestParentNode = tempNode.predecessors[pos.x][pos.y];
                        }
                    }
                }

                tempNode.rhsValue = minRHS;
                tempNode.parentNode = bestParentNode;
                nodes[nodeIndex] = tempNode;
            }
            
            UpdateNodeInOpenList(nodeIndex);
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
            return CellGridManagerHelper.GetClosestNode(aiPosition, searchGraphOrigin, cellSize, searchSpace, nodes, neighboursCountSize);
        }
        
        private void AddToOpen(int childIndex)
        {
            double2 newKeyValue = CalculateKeyValue(nodes[childIndex]);
            if (newKeyValue[0] >= float.MaxValue)
            {
                return;
            }

            CellNode cellNode = nodes[childIndex];
            cellNode.priorityKey = newKeyValue;
            if (!cellNode.isInOpenList) //add to open list
            {
                cellNode.isInOpenList = true;
                heapIndex.Add(childIndex, openList.Insert(cellNode));
            }
            else //update in open list
            {
                openList.Remove(heapIndex[childIndex]);
                heapIndex.Remove(childIndex);
                heapIndex.Add(childIndex, openList.Insert(cellNode));
                cellNode.isInOpenList = true;
            }
            
            if (!searchTree.Contains(childIndex))
            {
                searchTree.Add(childIndex);
            }
            nodes[childIndex] = cellNode;
        }

        private void RemoveFromOpen(int nodeIndex)
        {
            CellNode cellNode = nodes[nodeIndex];
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
            if (openList.Count <= 0)
            {
                return -1;
            }
            int nodeIndex = openList.Peek().index;
            return nodeIndex;
        }
        
        private double2 CalculateKeyValue(CellNode child)
        {
            double2 key = new double2();
            if (child.gValue > child.rhsValue)
            {
                key[0] = child.rhsValue + child.heuristic + km[0];
                key[1] = child.rhsValue;
            }
            else
            {
                key[0] = child.gValue + child.plainHeuristic + km[0];
                key[1] = child.gValue;
            }

            return key;
        }
    }
}
