using Pathfinding.Algorithms;
using Pathfinding.Nodes;
using Unity.Collections;
using Unity.Mathematics;

namespace Pathfinding.SearchGraph
{
    public static class CellGridManagerHelper
    {
        public static int3[] GetNormalizedDirections()
        {
            int3[] normalizedDirections = new int3[6];
            normalizedDirections[0] = new int3(1, 0, 0);
            normalizedDirections[1] = new int3(-1, 0, 0);
            normalizedDirections[2] = new int3(0, 1, 0);
            normalizedDirections[3] = new int3(0, -1, 0);
            normalizedDirections[4] = new int3(0, 0, 1);
            normalizedDirections[5] = new int3(0, 0, -1);
            return normalizedDirections;
        }

        public static int3[] GetNormalizedEdgeDirections()
        {
            int3[] normalizedEdgeDirections = new int3[12];
            normalizedEdgeDirections[0] = new int3(+1, 1, 0);
            normalizedEdgeDirections[1] = new int3(-1, 1, 0);
            normalizedEdgeDirections[2] = new int3(0,  1, +1);
            normalizedEdgeDirections[3] = new int3(0,  1, -1);
            normalizedEdgeDirections[4] = new int3(+1, -1, 0);
            normalizedEdgeDirections[5] = new int3(-1, -1, 0);
            normalizedEdgeDirections[6] = new int3(0, -1, +1);
            normalizedEdgeDirections[7] = new int3(0, -1, -1);
            normalizedEdgeDirections[8] = new int3(+1, 0, 1);
            normalizedEdgeDirections[9] = new int3(-1, 0, 1);
            normalizedEdgeDirections[10] = new int3(+1, 0, -1);
            normalizedEdgeDirections[11] = new int3(-1, 0, -1);
            return normalizedEdgeDirections;
        }
        
        public static int2x2[] GetSplitEdgeOrderedNeighbourIndices()
        {
            int2x2[] splitEdgeOrderedNeighbourIndices = new int2x2[12];
            splitEdgeOrderedNeighbourIndices[0] = new  int2x2(new int2(0,0), new int2(0,1));
            splitEdgeOrderedNeighbourIndices[1] = new  int2x2(new int2(1,0), new int2(0,1));
            splitEdgeOrderedNeighbourIndices[2] = new  int2x2(new int2(0,1), new int2(0,2));
            splitEdgeOrderedNeighbourIndices[3] = new  int2x2(new int2(0,1), new int2(1,2));
            splitEdgeOrderedNeighbourIndices[4] = new  int2x2(new int2(0,0), new int2(1,1));
            splitEdgeOrderedNeighbourIndices[5] = new  int2x2(new int2(1,0), new int2(1,1));
            splitEdgeOrderedNeighbourIndices[6] = new  int2x2(new int2(1,1), new int2(0,2));
            splitEdgeOrderedNeighbourIndices[7] = new  int2x2(new int2(1,1), new int2(1,2));
            splitEdgeOrderedNeighbourIndices[8] = new  int2x2(new int2(0,0), new int2(0,2));
            splitEdgeOrderedNeighbourIndices[9] = new  int2x2(new int2(1,0), new int2(0,2));
            splitEdgeOrderedNeighbourIndices[10] = new int2x2(new int2(0,0), new int2(1,2));
            splitEdgeOrderedNeighbourIndices[11] = new int2x2(new int2(1,0), new int2(1,2));
            return splitEdgeOrderedNeighbourIndices;
        }

        public static int2[] GetSplitEdgeNormalizedDirectionIndices()
        {
            int2[] splitEdgeNormalizedDirectionIndices = new int2[12];
            splitEdgeNormalizedDirectionIndices[0] = new int2(0,2);
            splitEdgeNormalizedDirectionIndices[1] = new int2(1,2);
            splitEdgeNormalizedDirectionIndices[2] = new int2(2,4);
            splitEdgeNormalizedDirectionIndices[3] = new int2(2,5);
            splitEdgeNormalizedDirectionIndices[4] = new int2(0,3);
            splitEdgeNormalizedDirectionIndices[5] = new int2(1,3);
            splitEdgeNormalizedDirectionIndices[6] = new int2(3,4);
            splitEdgeNormalizedDirectionIndices[7] = new int2(3,5);
            splitEdgeNormalizedDirectionIndices[8] = new int2(0,4);
            splitEdgeNormalizedDirectionIndices[9] = new int2(1,4);
            splitEdgeNormalizedDirectionIndices[10] = new int2(0,5);
            splitEdgeNormalizedDirectionIndices[11] = new int2(1,5);
            
            return splitEdgeNormalizedDirectionIndices;
        }

        public static int3[] GetNormalizedVertexDirections()
        {
            int3[] normalizedVertexDirections = new int3[8];
            normalizedVertexDirections[0] = new int3(+1, +1, +1);
            normalizedVertexDirections[1] = new int3(+1, +1, -1);
            normalizedVertexDirections[2] = new int3(-1, +1, +1);
            normalizedVertexDirections[3] = new int3(-1, +1, -1);
            normalizedVertexDirections[4] = new int3(+1, -1, +1);
            normalizedVertexDirections[5] = new int3(+1, -1, -1);
            normalizedVertexDirections[6] = new int3(-1, -1, +1);
            normalizedVertexDirections[7] = new int3(-1, -1, -1);
            
            return normalizedVertexDirections;
        }

        public static int2x3[] GetSplitVertexOrderedNeighbourIndices()
        {
            int2x3[] splitVertexOrderedNeighbourIndices = new int2x3[8];
            splitVertexOrderedNeighbourIndices[0] = new  int2x3(new int2(0,0), new int2(0,1), new int2(0,2));
            splitVertexOrderedNeighbourIndices[1] = new  int2x3(new int2(0,0), new int2(0,1), new int2(1,2));
            splitVertexOrderedNeighbourIndices[2] = new  int2x3(new int2(1,0), new int2(0,1), new int2(0,2));
            splitVertexOrderedNeighbourIndices[3] = new  int2x3(new int2(1,0), new int2(0,1), new int2(1,2));
            splitVertexOrderedNeighbourIndices[4] = new  int2x3(new int2(0,0), new int2(1,1), new int2(0,2));
            splitVertexOrderedNeighbourIndices[5] = new  int2x3(new int2(0,0), new int2(1,1), new int2(1,2));
            splitVertexOrderedNeighbourIndices[6] = new  int2x3(new int2(1,0), new int2(1,1), new int2(0,2));
            splitVertexOrderedNeighbourIndices[7] = new  int2x3(new int2(1,0), new int2(1,1), new int2(1,2));
            return splitVertexOrderedNeighbourIndices;
        }
        
        public static int3[] GetSplitVertexNormalizedDirectionIndices()
        {
            int3[] splitVertexNormalizedDirectionIndices = new int3[8];
            splitVertexNormalizedDirectionIndices[0] = new int3(0,2,4);
            splitVertexNormalizedDirectionIndices[1] = new int3(0,2,5);
            splitVertexNormalizedDirectionIndices[2] = new int3(1,2,4);
            splitVertexNormalizedDirectionIndices[3] = new int3(1,2,5);
            splitVertexNormalizedDirectionIndices[4] = new int3(0,3,4);
            splitVertexNormalizedDirectionIndices[5] = new int3(0,3,5);
            splitVertexNormalizedDirectionIndices[6] = new int3(1,3,4);
            splitVertexNormalizedDirectionIndices[7] = new int3(1,3,5);
            return splitVertexNormalizedDirectionIndices;
        }

        public static int2x3[] GetSplitVertexOrderedEdgesIndices()
        {
            int2x3[] splitVertexOrderedNeighbourIndices = new int2x3[8];
            splitVertexOrderedNeighbourIndices[0] = new  int2x3(new int2(0,0), new int2(2,0), new int2(2,2));
            splitVertexOrderedNeighbourIndices[1] = new  int2x3(new int2(0,0), new int2(0,1), new int2(1,3));
            splitVertexOrderedNeighbourIndices[2] = new  int2x3(new int2(1,0), new int2(2,0), new int2(0,3));
            splitVertexOrderedNeighbourIndices[3] = new  int2x3(new int2(1,0), new int2(0,1), new int2(2,3));
            
            splitVertexOrderedNeighbourIndices[4] = new  int2x3(new int2(1,1), new int2(0,2), new int2(2,2));
            splitVertexOrderedNeighbourIndices[5] = new  int2x3(new int2(1,1), new int2(1,2), new int2(1,3));
            splitVertexOrderedNeighbourIndices[6] = new  int2x3(new int2(2,1), new int2(0,2), new int2(0,3));
            splitVertexOrderedNeighbourIndices[7] = new  int2x3(new int2(2,1), new int2(1,2), new int2(2,3));
            return splitVertexOrderedNeighbourIndices;
        }
        
        public static int3[] GetSplitVertexNormalizedEdgeDirectionIndices()
        {
            int3[] splitVertexNormalizedEdgeDirectionIndices = new int3[8];
            splitVertexNormalizedEdgeDirectionIndices[0] = new int3(0,2,8);
            splitVertexNormalizedEdgeDirectionIndices[1] = new int3(0,3,10);
            splitVertexNormalizedEdgeDirectionIndices[2] = new int3(1,2,9);
            splitVertexNormalizedEdgeDirectionIndices[3] = new int3(1,3,11);
            
            splitVertexNormalizedEdgeDirectionIndices[4] = new int3(4,6,8);
            splitVertexNormalizedEdgeDirectionIndices[5] = new int3(4,7,10);
            splitVertexNormalizedEdgeDirectionIndices[6] = new int3(5,6,9);
            splitVertexNormalizedEdgeDirectionIndices[7] = new int3(5,7,11);
            return splitVertexNormalizedEdgeDirectionIndices;
        }
        
        public static int GetClosestNode(float3 position, float3 cellGridOrigin, float cellSize, int3 searchSpace, NativeArray<HybridNode> allNodes, NativeArray<HybridNodePathCosts> nodePathCosts, NativeArray<int> usedNodes, int2 neighbourCount, float losOffsetMultiplier)
        {
            int closestCell = GetClosestNode(position, cellGridOrigin, cellSize, searchSpace, allNodes, nodePathCosts, neighbourCount);
            if (allNodes[closestCell].isUsed)
            {
                return closestCell;
            }
            
            HybridNode node = allNodes[closestCell];
            double minSquaredDistance = math.INFINITY_DBL;
            int minDistanceIndex = -1;
            for (int j = 0; j < usedNodes.Length; j++)
            {
                if (allNodes[usedNodes[j]].isDynamicNode)
                {
                    continue;
                }
                
                double squaredDistance = SquaredDistance(allNodes[usedNodes[j]].globalPosition, node.globalPosition);
                    
                if (squaredDistance < minSquaredDistance)
                {
                    NativeList<float> losTimes = new NativeList<float>(Allocator.Temp);
                    if(LineOfSightHelper.LineOfSight(allNodes[usedNodes[j]].localPosition, node.localPosition, searchSpace, losOffsetMultiplier, nodePathCosts, losTimes))
                    {
                        minSquaredDistance = squaredDistance;
                        minDistanceIndex = usedNodes[j];
                    }

                    losTimes.Dispose();
                }
                
                if (minSquaredDistance <= 1)
                {
                    break;
                }
            }

            return minDistanceIndex;
        }
        
        
        public static int GetClosestNode(float3 position, float3 cellGridOrigin, float cellSize, int3 searchSpace, NativeArray<CellNode> nodes, int2 neighbourCount)// where T : struct, INode, INeighbours3X2
        {
            int nodeIndex = -1;
            cellGridOrigin -= (cellSize / 2);
            if(math.all(position > cellGridOrigin | position < cellGridOrigin + new float3(searchSpace.x, searchSpace.y, searchSpace.z)))
            {
                float3 diff = (position - cellGridOrigin) / cellSize; 
                nodeIndex = PositionToIndex((int)diff.x, (int)diff.y, (int)diff.z, searchSpace);
            }

            if (!nodes[nodeIndex].isFree)
            {
                nodeIndex = GetFreeNeighbour(nodeIndex, neighbourCount, nodes);
            }

            return nodeIndex;
        }
        
        public static int GetClosestNode(float3 position, float3 cellGridOrigin, float cellSize, int3 searchSpace, NativeArray<HybridNode> nodes, NativeArray<HybridNodePathCosts> nodePathCosts, int2 neighbourCount)// where T : struct, INode, INeighbours3X2
        {
            int nodeIndex = -1;
            cellGridOrigin -= (cellSize / 2);
            if(math.all(position > cellGridOrigin | position < cellGridOrigin + new float3(searchSpace.x, searchSpace.y, searchSpace.z)))
            {
                float3 diff = (position - cellGridOrigin) / cellSize; 
                nodeIndex = PositionToIndex((int)diff.x, (int)diff.y, (int)diff.z, searchSpace);
            }

            if (!nodePathCosts[nodeIndex].isFree)
            {
                nodeIndex = GetFreeNeighbour(nodeIndex, neighbourCount, nodes, nodePathCosts);
            }

            return nodeIndex;
        }

        private static int GetFreeNeighbour(int nodeIndex, int2 neighbourCount, NativeArray<CellNode> nodes)
        {
            int neighbourIndex = -1;
            int3x2 neighbours = nodes[nodeIndex].neighbours;
            int count = nodes[nodeIndex].neighboursCount;
            int2 pos = new int2();
            for (int i = 0; i < count; i++)
            {
                IndexToPosition(i, neighbourCount, ref pos);
                int neighbour = neighbours[pos.x][pos.y];
                if (nodes[neighbour].isFree)
                {
                    neighbourIndex = neighbour;
                }
            }

            return neighbourIndex;
        }
        
        private static int GetFreeNeighbour(int nodeIndex, int2 neighbourCount, NativeArray<HybridNode> nodes, NativeArray<HybridNodePathCosts> nodePathCosts)
        {
            int neighbourIndex = -1;
            int3x2 neighbours = nodes[nodeIndex].neighbours;
            int count = nodes[nodeIndex].neighboursCount;
            int2 pos = new int2();
            for (int i = 0; i < count; i++)
            {
                IndexToPosition(i, neighbourCount, ref pos);
                int neighbour = neighbours[pos.x][pos.y];
                if (nodePathCosts[neighbour].isFree)
                {
                    neighbourIndex = neighbour;
                }
            }

            return neighbourIndex;
        }

        
        public static int PositionToIndex(this int3 position, int3 searchSpace)
        {
            return (math.any(position < 0) || math.any(position >= searchSpace)) ? -1 : (position.z * (searchSpace.x * searchSpace.y)) + (position.y * searchSpace.x) + position.x;
            // return PositionToIndex(position.x, position.y, position.z, searchSpace);
        }

        public static int PositionToIndex(int x, int y, int z, int3 searchSpace)
        {
            return (x >= 0 && x < searchSpace.x && y >= 0 &&  y < searchSpace.y && z >= 0 && z < searchSpace.z) ? (z * (searchSpace.x * searchSpace.y)) + (y * searchSpace.x) + x : -1;
        }

        public static int PositionToIndexNoSafety(this int3 position, int3 searchSpace)
        {
            return (position.z * (searchSpace.x * searchSpace.y)) + (position.y * searchSpace.x) + position.x;
        }
        public static int PositionToIndexNoSafety(int positionX, int positionY, int positionZ, int3 searchSpace)
        {
            return (positionZ * (searchSpace.x * searchSpace.y)) + (positionY * searchSpace.x) + positionX;
        }
        
        public static void IndexToPosition(int index, int2 neighboursCount, ref int2 pos)
        {
            IndexToPosition(index, neighboursCount.x, neighboursCount.y, ref pos);
        }

        public static int2 IndexToPosition(int index, int2 neighboursCount)
        {
            return IndexToPosition(index, neighboursCount.x, neighboursCount.y);
        }
        
        public static int2 IndexToPosition(int index, int columns, int rows, ref int2 pos)
        {
            // int2 pos = new int2 {x = index % columns, y = index / (rows - 1)};
            pos.x = index % columns;
            // pos.y = index / (rows - 1);
            pos.y = index / columns;
            return pos;
        }
        
        public static int2 IndexToPosition(int index, int columns, int rows)
        {
            int2 pos = -1;
            pos.x = index % columns;
            // pos.y = index / (rows - 1);
            pos.y = index / columns;
            return pos;
        }

        public static double Distance(float3 from, float3 to)
        {
            float3 diff = to - from;
            return math.sqrt(math.csum(diff * diff));
        }
        
        public static double SquaredDistance(float3 from, float3 to)
        {
            float3 diff = to - from;
            return math.csum(diff * diff);
        }

        public static float3[] GlobalNodeVertices(float3 globalPosition, float size)
        {
            float3[] globalNodeVertices = new float3[8];
            float sizeHalf = size / 2;

            globalNodeVertices[0] = globalPosition + sizeHalf;
            globalNodeVertices[1] = globalPosition - sizeHalf;

            globalNodeVertices[2] = globalPosition + new float3(-sizeHalf, sizeHalf, sizeHalf);
            globalNodeVertices[3] = globalPosition + new float3(-sizeHalf, sizeHalf, -sizeHalf);
            globalNodeVertices[4] = globalPosition + new float3(sizeHalf, sizeHalf, -sizeHalf);
            
            globalNodeVertices[5] = globalPosition + new float3(-sizeHalf, -sizeHalf, sizeHalf);
            globalNodeVertices[6] = globalPosition + new float3(-sizeHalf, -sizeHalf, -sizeHalf);
            globalNodeVertices[7] = globalPosition + new float3(sizeHalf, -sizeHalf, -sizeHalf);
            
            return globalNodeVertices;
        }
        
        // public static bool LineOfSight(int3 fromNode, int3 toNode, NativeArray<HybridNodePathCosts> nodes, int3 searchSpace, float losOffsetMultiplier)
        // {
        //     float3 direction = toNode - fromNode;
        //     float3 directionAbs = math.abs(direction);
        //     
        //     float maxAbs = math.cmax(directionAbs);
        //     //line is direct diagonal in 2 or 3 dimensions
        //     
        //     if (math.all(directionAbs == 0 | directionAbs == maxAbs))
        //     {
        //         return DiagonalLOS(maxAbs, fromNode, direction, nodes, searchSpace);
        //     }
        //     return GeneralLOS(fromNode, direction, directionAbs, nodes, searchSpace, losOffsetMultiplier);
        //     
        // }
        //
        // private static bool DiagonalLOS(float maxAbs, int3 fromNode, float3 direction, NativeArray<HybridNodePathCosts> nodes, int3 searchSpace)
        // {
        //     int3 lastPosition = fromNode;
        //     int3 position = fromNode;
        //     int index;
        //     int3 checkDirection = (int3)math.sign(direction);
        //     for(int i = 0; i < maxAbs; i++)
        //     {
        //         position += checkDirection;
        //         index = position.PositionToIndexNoSafety(searchSpace);
        //         if (!nodes[index].isFree)
        //         {
        //             return false;
        //         }
        //         
        //         for (int m = 0; m < 3; m++)
        //         {
        //             int3 dir = 0;
        //             dir[m] = checkDirection[m];
        //             index = (lastPosition + dir).PositionToIndexNoSafety(searchSpace);
        //             if (!nodes[index].isFree)
        //             {
        //                 return false;
        //             }
        //
        //             
        //             dir[m] *= -1;
        //             index = (position + dir).PositionToIndexNoSafety(searchSpace);
        //             if (!nodes[index].isFree)
        //             {
        //                 return false;
        //             }
        //         }
        //         
        //         lastPosition = position;
        //     }
        //     
        //     return true;
        // }
        //
        // private static bool GeneralLOS(int3 fromNode, float3 direction, float3 directionAbs, NativeArray<HybridNodePathCosts> nodes, int3 searchSpace, float losOffsetMultiplier)
        // {
        //     NativeList<float> losTimes = new NativeList<float>(Allocator.Temp);
        //     
        //     int3 position = fromNode;
        //     int index;
        //     float t;
        //     float offset = losOffsetMultiplier / math.length(direction);
        //     
        //     for (int k = 0; k < 3; k++)
        //     {
        //         for (int i = 0; i < directionAbs[k]; i++)
        //         {
        //             t = math.mad((0.5f + i), 1/directionAbs[k], offset);
        //             losTimes.Add(t);
        //         }
        //     }
        //     
        //     int3 lastPosition;
        //     losTimes.Sort();
        //     bool checkDiagonal;
        //     for(int i = 0; i < losTimes.Length; i++)
        //     {
        //         t = losTimes[i];
        //         
        //         lastPosition = position;
        //         position = (int3) math.round(math.mad(t, direction, fromNode));
        //         checkDiagonal = lastPosition.Equals(position);
        //             
        //         index = position.PositionToIndexNoSafety(searchSpace);
        //         if (!nodes[index].isFree)
        //         {
        //             return false;
        //         }
        //
        //         if (!checkDiagonal)
        //         {
        //             continue;
        //         }
        //
        //         lastPosition = (int3) math.round(math.mad(t-(2*offset), direction, fromNode));
        //         int3 checkDirection = (position - lastPosition);
        //         for (int m = 0; m < 3; m++)
        //         {
        //             if (checkDirection[m] == 0)
        //             {
        //                 continue;
        //             }
        //             
        //             int3 dir = 0;
        //             dir[m] = checkDirection[m];
        //             index = (lastPosition + dir).PositionToIndexNoSafety(searchSpace);
        //             if(!nodes[index].isFree)
        //             {
        //                 return false;
        //             }
        //             
        //             dir[m] *= -1;
        //             index = (position + dir).PositionToIndexNoSafety(searchSpace);
        //             if(!nodes[index].isFree)
        //             {
        //                 return false;
        //             }
        //         }
        //     }
        //     
        //     return true;
        // }
    }
}
