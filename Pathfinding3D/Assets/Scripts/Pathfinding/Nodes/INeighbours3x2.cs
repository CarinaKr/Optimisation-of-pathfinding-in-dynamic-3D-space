using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public interface INeighbours3X2
    {
        int3x2 neighbours { get; set; }
        int3x2 orderedNeighbours { get; set; }
        int3x2 predecessors { get; set; }
        int neighboursCount { get; set; }
    }
}
