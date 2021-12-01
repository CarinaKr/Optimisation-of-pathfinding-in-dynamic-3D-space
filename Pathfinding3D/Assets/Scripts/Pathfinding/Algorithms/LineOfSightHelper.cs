using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Unity.Mathematics;

namespace Pathfinding.Algorithms
{
    public static class LineOfSightHelper
    {
        public static bool LineOfSight(int3 fromNode, int3 toNode, int3 searchSpace, float losOffsetMultiplier, NativeArray<HybridNodePathCosts> nodePathCosts, NativeList<float> losTimes)
        {
            float3 direction = toNode - fromNode;
            float3 directionAbs = math.abs(direction);
            
            float maxAbs = math.cmax(directionAbs);
            //line is direct diagonal in 2 or 3 dimensions
            if (math.all(directionAbs == 0 | directionAbs == maxAbs))
            {
                return DiagonalLOS(maxAbs, fromNode, direction, searchSpace, nodePathCosts);
            }
            
            return GeneralLOS(fromNode, direction, directionAbs, losOffsetMultiplier, nodePathCosts, searchSpace, losTimes);
            
        }

        public static bool DiagonalLOS(float maxAbs, int3 fromNode, float3 direction, int3 searchSpace, NativeArray<HybridNodePathCosts> nodePathCosts)
        {
            int3 lastPosition = fromNode;
            int3 position = fromNode;
            int index;
            int3 checkDirection = (int3)math.sign(direction);
            for(int i = 0; i < maxAbs; i++)
            {
                position += checkDirection;
                index = position.PositionToIndexNoSafety(searchSpace);
                if (!nodePathCosts[index].isFree)
                {
                    return false;
                }
                
                for (int m = 0; m < 3; m++)
                {
                    int3 dir = 0;
                    dir[m] = checkDirection[m];
                    index = (lastPosition + dir).PositionToIndexNoSafety(searchSpace);
                    if (!nodePathCosts[index].isFree)
                    {
                        return false;
                    }

                    
                    dir[m] *= -1;
                    index = (position + dir).PositionToIndexNoSafety(searchSpace);
                    if (!nodePathCosts[index].isFree)
                    {
                        return false;
                    }
                }
                
                lastPosition = position;
            }
            
            return true;
        }

        public static bool GeneralLOS(int3 fromNode, float3 direction, float3 directionAbs, float losOffsetMultiplier, NativeArray<HybridNodePathCosts> nodePathCosts, int3 searchSpace, NativeList<float> losTimes)
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
            for (int i = 0; i < losTimes.Length; i++)
            {
                t = losTimes[i];
                lastPosition = position;
                position = (int3) math.round(math.mad(t, direction, fromNode));
            
                bool checkDiagonal = lastPosition.Equals(position);
                if (!checkDiagonal && !nodePathCosts[position.PositionToIndexNoSafety(searchSpace)].isFree)
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
                    if(!nodePathCosts[index].isFree)
                    {
                        return false;
                    }
                    
                    dir[m] *= -1;
                    index = (position + dir).PositionToIndexNoSafety(searchSpace);
                    if(!nodePathCosts[index].isFree)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
