using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public interface IEdges4X3
    {
        int4x3 edgeNeighbours { get; set; }
        int4x3 orderedEdgeNeighbours { get; set; }
        int4x3 edgePredecessors { get; set; }
        int edgesCount { get; set; }
    }
}
