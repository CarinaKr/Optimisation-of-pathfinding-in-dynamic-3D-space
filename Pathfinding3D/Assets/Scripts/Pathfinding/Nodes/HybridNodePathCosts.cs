using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public struct HybridNodePathCosts : IFree
    {
        public bool isFree { get; set; }
        public double3x2 pathCosts { get; set; }
        public double4x3 edgePathCosts { get; set; }
        public double4x2 verticesPathCosts { get; set; }

        public HybridNodePathCosts(bool free)
        {
            isFree = free;
            pathCosts = math.INFINITY_DBL;
            edgePathCosts = math.INFINITY_DBL;
            verticesPathCosts = math.INFINITY_DBL;
        }

        public void RecoverSaveState(HybridNodeSaveState hybridNode)
        {
            pathCosts = hybridNode.pathCosts;
            edgePathCosts = hybridNode.edgePathCosts;
            verticesPathCosts = hybridNode.verticesPathCosts;
        }
    }
}
