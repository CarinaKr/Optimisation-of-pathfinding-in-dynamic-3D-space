using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public struct HeapNode
    {
        public int Index;
        public double Priority;
    }

    public struct DStarHeapNode
    {
        public int Index;
        public double2 Priority;
    }
}
