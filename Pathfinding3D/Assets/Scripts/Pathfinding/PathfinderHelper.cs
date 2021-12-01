using BovineLabs.Common.Extensions;
using Pathfinding.Nodes;
using Unity.Collections;
using Unity.Mathematics;

namespace Pathfinding
{
    public static class PathfinderHelper
    {
        public static NativeArray<CellNode> UpdateHeuristic(NativeArray<CellNode> nodes, float3 goal, float inflate)
        {
            NativeArray<CellNode> updatedNodes = nodes;

            float4 goalX = goal.xxxx;
            float4 goalY = goal.yyyy;
            float4 goalZ = goal.zzzz;
            NodesGroup nodesGroup;
            for (int i = 0; i < nodes.Length; i += 4)
            {
                nodesGroup = new NodesGroup(nodes[i].globalPosition, nodes[i+1].globalPosition, nodes[i+2].globalPosition, nodes[i+3].globalPosition);
                float4 diffX = goalX - nodesGroup.posXs;
                float4 diffY = goalY - nodesGroup.posYs;
                float4 diffZ = goalZ - nodesGroup.posZs;
                
                float4 squareDistanceX = diffX * diffX;
                float4 squareDistanceY = diffY * diffY;
                float4 squareDistanceZ = diffZ * diffZ;
                
                float4 distance = math.sqrt(squareDistanceX + squareDistanceY + squareDistanceZ);

                nodesGroup.plainHeuristics = distance;
                nodesGroup.heuristics = distance * inflate;

                for (int j = 0; j < 4; j++)
                {
                    CellNode node = nodes[i+j];
                    node.heuristic = nodesGroup.heuristics[j];
                    node.plainHeuristic = nodesGroup.plainHeuristics[j];
                    updatedNodes[i+j] = node;
                }
            }

            int nodesLeft = nodes.Length % 4;
            for (int i = 0; i < nodesLeft; i++)
            {
                float3 diff = goal - nodes[nodes.Length - 1 - i].globalPosition;
                float distance = math.sqrt(math.csum(diff * diff));
                
                CellNode node = nodes[nodes.Length - 1 - i];
                node.heuristic = distance * inflate;
                node.plainHeuristic = distance;
                updatedNodes[nodes.Length - 1 - i] = node;
            }
            
            return updatedNodes;
        }

        public static void UpdateHeuristic<T>(NativeArray<T> nodes, float3 goal, float inflate, ref NativeArray<double> heuristics) where T : struct, INode
        {
            float4 goalX = goal.xxxx;
            float4 goalY = goal.yyyy;
            float4 goalZ = goal.zzzz;
            NodesGroup nodesGroup;
            for (int i = 0; i < nodes.Length; i += 4)
            {
                nodesGroup = new NodesGroup(nodes[i].globalPosition, nodes[i+1].globalPosition, nodes[i+2].globalPosition, nodes[i+3].globalPosition);
                float4 diffX = goalX - nodesGroup.posXs;
                float4 diffY = goalY - nodesGroup.posYs;
                float4 diffZ = goalZ - nodesGroup.posZs;
                
                float4 squareDistanceX = diffX * diffX;
                float4 squareDistanceY = diffY * diffY;
                float4 squareDistanceZ = diffZ * diffZ;
                
                float4 distance = math.sqrt(squareDistanceX + squareDistanceY + squareDistanceZ);

                nodesGroup.plainHeuristics = distance;
                nodesGroup.heuristics = distance * inflate;

                for (int j = 0; j < 4; j++)
                {
                    heuristics[i+j] = nodesGroup.heuristics[j];
                }
            }

            int nodesLeft = nodes.Length % 4;
            for (int i = 0; i < nodesLeft; i++)
            {
                float3 diff = goal - nodes[nodes.Length - 1 - i].globalPosition;
                float distance = math.sqrt(math.csum(diff * diff));
                
                heuristics[nodes.Length - 1 - i] = distance * inflate;
            }
        }
        
        public static void UpdateHeuristic<T>(NativeArray<T> nodes, float3 goal, float inflate, ref NativeArray<double> heuristics, ref NativeArray<double> plainHeuristic, float plainHeuristicMultiplier = 1) where T : struct, INode
        {
            float4 goalX = goal.xxxx;
            float4 goalY = goal.yyyy;
            float4 goalZ = goal.zzzz;
            NodesGroup nodesGroup;
            for (int i = 0; i < nodes.Length; i += 4)
            {
                nodesGroup = new NodesGroup(nodes[i].globalPosition, nodes[i+1].globalPosition, nodes[i+2].globalPosition, nodes[i+3].globalPosition);
                float4 diffX = goalX - nodesGroup.posXs;
                float4 diffY = goalY - nodesGroup.posYs;
                float4 diffZ = goalZ - nodesGroup.posZs;
                
                float4 squareDistanceX = diffX * diffX;
                float4 squareDistanceY = diffY * diffY;
                float4 squareDistanceZ = diffZ * diffZ;
                
                float4 distance = math.sqrt(squareDistanceX + squareDistanceY + squareDistanceZ);

                nodesGroup.plainHeuristics = distance * plainHeuristicMultiplier;
                nodesGroup.heuristics = distance * inflate;

                for (int j = 0; j < 4; j++)
                {
                    heuristics[i+j] = nodesGroup.heuristics[j];
                    plainHeuristic[i + j] = nodesGroup.plainHeuristics[j];
                }
            }

            int nodesLeft = nodes.Length % 4;
            for (int i = 0; i < nodesLeft; i++)
            {
                float3 diff = goal - nodes[nodes.Length - 1 - i].globalPosition;
                float distance = math.sqrt(math.csum(diff * diff));
                
                heuristics[nodes.Length - 1 - i] = distance * inflate;
                plainHeuristic[nodes.Length - 1 - i] = distance;
            }
        }
        
        public static void FillCorridor<T>(int child, bool goalToStart, ref NativeList<float3> corridor, NativeArray<T> nodes, ref float pathLength) where T : struct, INode, INeighbours3X2
        {
            if (corridor.Contains(nodes[child].globalPosition))
            {
                return;
            }
            
            corridor.Add(nodes[child].globalPosition);

            if (nodes[child].parentNode == -1)
            {
                if (goalToStart)
                {
                    corridor.Reverse();
                }
                return;
            }

            float3 diff;
            diff = (nodes[child].globalPosition - nodes[nodes[child].parentNode].globalPosition);
            diff = math.pow(diff, 2);
            float distance = math.sqrt(diff.x + diff.y + diff.z);
            pathLength += distance;
            FillCorridor(nodes[child].parentNode, goalToStart, ref corridor, nodes, ref pathLength);
        }
        
        public static void FillCorridor<T>(int child, bool goalToStart, ref NativeList<float3> corridor, NativeArray<T> nodes) where T : struct, INode
        {
            corridor.Add(nodes[child].globalPosition);

            if (nodes[child].parentNode == -1)
            {
                if (goalToStart)
                {
                    corridor.Reverse();
                }
                return;
            }

            FillCorridor(nodes[child].parentNode, goalToStart, ref corridor, nodes);
        }

        public static int3 NormalizedDirection(this int3 direction)
        {
            return math.clamp(direction, -1, 1);
        }

        public static int NormalizedDirectionIndex(this int3 direction)
        {
            for (int i = 0; i < 3; i++)
            {
                if (direction[i] != 0)
                {
                    return direction[i] > 0 ? i * 2 : (i * 2) + 1;
                }
            }

            return -1;
        }
    }
}
