using Pathfinding.Nodes;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Pathfinding.SearchGraph
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct UpdatePathCostsJob : IJob
    {
        public NativeArray<HybridNode> allNodes;
        public NativeArray<HybridNodePathCosts> usedNodesPathCosts;
        public NativeList<int> changedNodes;
        public NativeArray<int> dynamicNodes;
        public NativeArray<bool> nodeIsBlocked;
        public NativeArray<int2x2> splitEdgeOrderedNeighbours;
        public NativeArray<int2x3> splitVertexOrderedSideNeighbours;
        public NativeArray<int2x3> splitVertexOrderedEdgeNeighbours;
        public NativeArray<int2> splitEdgeNormalizedDirectionIndices;
        public NativeArray<int3> splitVertexNormalizedDirectionIndices;
        public NativeArray<int3> splitVertexNormalizedEdgeDirectionIndices;
        public NativeArray<int3> normalizedDirections;
        public NativeArray<int3> normalizedEdgeDirections;
        public NativeArray<int3> normalizedVertexDirections;
        public int3 searchSpace;
        
        
        private static readonly int2 neighbourCount = new int2(2, 3);
        private static readonly int2 edgesCount = new int2(3, 4);
        private static readonly int2 verticesCount = new int2(2, 4);
        
        private float size;
        private float vertexPathCost;
        private float edgePathCost;

        public UpdatePathCostsJob(float cellSize)
        {
            size = cellSize;
            vertexPathCost = math.sqrt(3) * cellSize;
            edgePathCost = math.sqrt(2) * cellSize;
            
            allNodes = default;
            changedNodes = default;
            dynamicNodes = default;
            nodeIsBlocked = default;
            splitEdgeOrderedNeighbours = default;
            splitVertexOrderedSideNeighbours = default;
            splitVertexOrderedEdgeNeighbours = default;
            splitEdgeNormalizedDirectionIndices = default;
            splitVertexNormalizedDirectionIndices = default;
            splitVertexNormalizedEdgeDirectionIndices = default;
            searchSpace = default;
            normalizedDirections = default;
            normalizedEdgeDirections = default;
            normalizedVertexDirections = default;
            usedNodesPathCosts = default;
        }
        
        public void Execute()
        {
            //transfer results to allNodes array
            changedNodes.Clear();
             for (int i = 0; i < dynamicNodes.Length; i++)
             {
                 if (usedNodesPathCosts[dynamicNodes[i]].isFree == nodeIsBlocked[i])
                 {
                     changedNodes.Add(dynamicNodes[i]);
                     HybridNodePathCosts node = usedNodesPathCosts[dynamicNodes[i]];
                     node.isFree = !nodeIsBlocked[i];
                     usedNodesPathCosts[dynamicNodes[i]] = node;
                 }
             }
            
             //update path costs of changed nodes and affected neighbours of changed nodes
             NativeHashSet<int> additionalChangedNodes = new NativeHashSet<int>(0, Allocator.Temp);
             NativeList<int> changedNeighbours = new NativeList<int>(Allocator.Temp);
             for (int i = 0; i < changedNodes.Length; i++)
             {
                 UpdatePathCosts(changedNodes[i], ref additionalChangedNodes, ref changedNeighbours);
             }

             for (int i = 0; i < changedNeighbours.Length; i++)
             {
                 UpdateAdditionalNodesPathCosts(changedNeighbours[i]);
             }
            
             changedNodes.AddRange(additionalChangedNodes.ToNativeArray(Allocator.Temp));
        }
        
        private void UpdatePathCosts(int index, ref NativeHashSet<int> additionalChangedNodes, ref NativeList<int> changedNeighbours)
        {
            HybridNode node = allNodes[index];
            HybridNodePathCosts nodePathCosts = usedNodesPathCosts[index];
            int2 pos = 0;
            
            double3x2 pathCosts = math.INFINITY_DBL;
            for (int i = 0; i < node.neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighbourCount, ref pos);
                int neighbour = allNodes[index].neighbours[pos.x][pos.y];
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree)
                {
                    pathCosts[pos.x][pos.y] = size;
                }

                if (!changedNodes.Contains(neighbour) && !additionalChangedNodes.Contains(neighbour))
                {
                    additionalChangedNodes.Add(neighbour);
                    changedNeighbours.Add(neighbour);
                }
            }
            nodePathCosts.pathCosts = pathCosts;
            
            double4x3 edgeCosts = math.INFINITY_DBL;
            int counter = 0;
            for (int i = 0; i < 12; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, edgesCount, ref pos);
                int neighbour = node.orderedEdgeNeighbours[pos.x][pos.y];
                if (neighbour == -1)
                {
                    continue;
                }

                int2 unorderedPosition = CellGridManagerHelper.IndexToPosition(counter, edgesCount);
                counter++;
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree && SplitEdgeNeighboursAreFree(node, i))   
                {
                    edgeCosts[unorderedPosition.x][unorderedPosition.y] = edgePathCost;
                }
                if (!changedNodes.Contains(neighbour) && !additionalChangedNodes.Contains(neighbour))
                {
                    additionalChangedNodes.Add(neighbour);
                    changedNeighbours.Add(neighbour);
                }
            }
            nodePathCosts.edgePathCosts = edgeCosts;
            
            double4x2 verticesCosts = math.INFINITY_DBL;
            counter = 0;
            for (int i = 0; i < 8; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, verticesCount, ref pos);
                int neighbour = allNodes[index].orderedVerticesNeighbours[pos.x][pos.y];
                if (neighbour == -1)
                {
                    continue;
                }
                
                int2 unorderedPosition = CellGridManagerHelper.IndexToPosition(counter, verticesCount);
                counter++;
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree && SplitVertexNeighboursAreFree(node, i))   
                {
                    verticesCosts[unorderedPosition.x][unorderedPosition.y] = vertexPathCost;
                }
                if (!changedNodes.Contains(neighbour) && !additionalChangedNodes.Contains(neighbour))
                {
                    additionalChangedNodes.Add(neighbour);
                    changedNeighbours.Add(neighbour);
                }
            }
            nodePathCosts.verticesPathCosts = verticesCosts;

            usedNodesPathCosts[index] = nodePathCosts;
        }

        private void UpdateAdditionalNodesPathCosts(int index)
        {
            HybridNode node = allNodes[index];
            HybridNodePathCosts nodePathCosts = usedNodesPathCosts[index];
            int2 pos = 0;

            double3x2 pathCosts = nodePathCosts.pathCosts;
            for (int i = 0; i < node.neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighbourCount, ref pos);
                int neighbour = allNodes[index].neighbours[pos.x][pos.y];
                if (!allNodes[neighbour].isDynamicNode)
                {
                    continue;
                }
                
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree)
                {
                    pathCosts[pos.x][pos.y] = size;
                }
                else
                {
                    pathCosts[pos.x][pos.y] = math.INFINITY_DBL;
                }
            }
            nodePathCosts.pathCosts = pathCosts;
            
            double4x3 edgeCosts = nodePathCosts.edgePathCosts;
            int counter = 0;
            for (int i = 0; i < 12; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, edgesCount, ref pos);
                int neighbour = node.orderedEdgeNeighbours[pos.x][pos.y];
                if (neighbour == -1)
                {
                    continue;
                }

                int3 distance = node.localPosition - allNodes[neighbour].localPosition;
                if((!allNodes[neighbour].isDynamicNode && !allNodes[neighbour].isDynamicNeighbourNode) || math.any(math.abs(distance) > 1))
                {
                    counter++;
                    continue;
                }
                
                int2 unorderedPosition = CellGridManagerHelper.IndexToPosition(counter, edgesCount);
                counter++;
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree && SplitEdgeNodesAreFree(node.localPosition, i))   
                {
                    edgeCosts[unorderedPosition.x][unorderedPosition.y] = edgePathCost;
                }
                else
                {
                    edgeCosts[unorderedPosition.x][unorderedPosition.y] = math.INFINITY_DBL;
                }
            }
            nodePathCosts.edgePathCosts = edgeCosts;
            
            double4x2 verticesCosts = nodePathCosts.verticesPathCosts;
            counter = 0;
            for (int i = 0; i < 8; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, verticesCount, ref pos);
                int neighbour = allNodes[index].orderedVerticesNeighbours[pos.x][pos.y];
                if (neighbour == -1)
                {
                    continue;
                }
                int3 distance = node.localPosition - allNodes[neighbour].localPosition;
                if((!allNodes[neighbour].isDynamicNode && !allNodes[neighbour].isDynamicNeighbourNode) || math.any(math.abs(distance) > 1))
                {
                    counter++;
                    continue;
                }
                
                int2 unorderedPosition = CellGridManagerHelper.IndexToPosition(counter, verticesCount);
                counter++;
                if (usedNodesPathCosts[index].isFree && usedNodesPathCosts[neighbour].isFree && SplitVertexNodesAreFree(node.localPosition, i))   
                {
                    verticesCosts[unorderedPosition.x][unorderedPosition.y] = vertexPathCost;
                }
                else
                {
                    verticesCosts[unorderedPosition.x][unorderedPosition.y] = math.INFINITY_DBL;
                }
            }
            nodePathCosts.verticesPathCosts = verticesCosts;

            usedNodesPathCosts[index] = nodePathCosts;
        }

        private bool SplitEdgeNeighboursAreFree(HybridNode node, int edgeDirectionIndex)
        {
            int2 edgeNeighbour = splitEdgeOrderedNeighbours[edgeDirectionIndex].c0;
            int splitEdgeNeighbour = node.orderedNeighbours[edgeNeighbour.x][edgeNeighbour.y];
            if (splitEdgeNeighbour == -1 || !usedNodesPathCosts[splitEdgeNeighbour].isFree)
            {
                return false;
            }
            
            edgeNeighbour = splitEdgeOrderedNeighbours[edgeDirectionIndex].c1;
            splitEdgeNeighbour = node.orderedNeighbours[edgeNeighbour.x][edgeNeighbour.y];
            if (splitEdgeNeighbour == -1 || !usedNodesPathCosts[splitEdgeNeighbour].isFree)
            {
                return false;
            }

            return true;
        }

        private bool SplitEdgeNodesAreFree(int3 localPosition, int edgeDirectionIndex)
        {
            int splitVertexNeighbour;
            int2 normalizedDirectionIndices = splitEdgeNormalizedDirectionIndices[edgeDirectionIndex];

            for (int i = 0; i < 2; i++)
            {
                splitVertexNeighbour = (localPosition + normalizedDirections[normalizedDirectionIndices[i]]).PositionToIndexNoSafety(searchSpace);
                if (!usedNodesPathCosts[splitVertexNeighbour].isFree)
                {
                    return false;
                }
            }
            
            return true;
        }
        
        private bool SplitVertexNeighboursAreFree(HybridNode node, int vertexDirectionIndex)
        {
            int2 edgeNeighbour = 0;
            int splitVertexNeighbour = -1;
            for (int i = 0; i < 3; i++)
            {
                edgeNeighbour = splitVertexOrderedSideNeighbours[vertexDirectionIndex][i];
                splitVertexNeighbour = node.orderedNeighbours[edgeNeighbour.x][edgeNeighbour.y];
                if (splitVertexNeighbour == -1 || !usedNodesPathCosts[splitVertexNeighbour].isFree)
                {
                    return false;
                }
            }

            for (int i = 0; i < 3; i++)
            {
                edgeNeighbour = splitVertexOrderedEdgeNeighbours[vertexDirectionIndex][i];
                splitVertexNeighbour = node.orderedEdgeNeighbours[edgeNeighbour.x][edgeNeighbour.y];
                if (splitVertexNeighbour == -1 || !usedNodesPathCosts[splitVertexNeighbour].isFree)
                {
                    return false;
                }
            }

            return true;
        }

        private bool SplitVertexNodesAreFree(int3 localPosition, int vertexDirectionIndex)
        {
            int splitVertexNeighbour;
            int3 normalizedDirectionIndices = splitVertexNormalizedDirectionIndices[vertexDirectionIndex];

            for (int i = 0; i < 3; i++)
            {
                splitVertexNeighbour = (localPosition + normalizedDirections[normalizedDirectionIndices[i]]).PositionToIndexNoSafety(searchSpace);
                if (!usedNodesPathCosts[splitVertexNeighbour].isFree)
                {
                    return false;
                }
            }
            
            normalizedDirectionIndices = splitVertexNormalizedEdgeDirectionIndices[vertexDirectionIndex];
            for (int i = 0; i < 3; i++)
            {
                splitVertexNeighbour = (localPosition + normalizedEdgeDirections[normalizedDirectionIndices[i]]).PositionToIndexNoSafety(searchSpace);
                if (!usedNodesPathCosts[splitVertexNeighbour].isFree)
                {
                    return false;
                }
            }
            
            return true;
        }
    }
}
