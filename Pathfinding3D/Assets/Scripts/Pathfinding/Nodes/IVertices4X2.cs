using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public interface IVertices4X2
    {
        int4x2 verticesNeighbours { get; set; }
        int4x2 orderedVerticesNeighbours { get; set; }
        int4x2 verticesPredecessors { get; set; }
        int verticesCount { get; set; }
    }
}
