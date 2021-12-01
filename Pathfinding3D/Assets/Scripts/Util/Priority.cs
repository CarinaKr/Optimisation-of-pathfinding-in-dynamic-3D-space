using System.Collections.Generic;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using UnityEngine;

namespace Util
{
    public struct DStarBasedPriority : IComparer<CellNode>
    {
        public int Compare(CellNode x, CellNode y)
        {
            int compareValue = 0;

            if (x.priorityKey[0] < y.priorityKey[0])
            {
                compareValue = -1;
            }
            else if (x.priorityKey[0] > y.priorityKey[0])
            {
                compareValue = 1;
            }
            else
            {
                if (x.priorityKey[1] < y.priorityKey[1])
                {
                    compareValue = -1;
                }
                else if (x.priorityKey[1] > y.priorityKey[1])
                {
                    compareValue = 1;
                }
            }

            return compareValue;
        }
    }

    public struct AStarBasedPriority : IComparer<CellNode>
    {
        public int Compare(CellNode x, CellNode y)
        {
            int compareValue = 0;

            if (x.priority < y.priority)
            {
                compareValue = -1;
            }
            else if (x.priority > y.priority)
            {
                compareValue = 1;
            }

            return compareValue;
        }
    }

    public struct HeapNodePriority : IComparer<HeapNode>
    {
        public int Compare(HeapNode x, HeapNode y)
        {
            return x.Priority > y.Priority ? 1 : -1;
        }
    }

    public struct DStarHeapNodePriority : IComparer<DStarHeapNode>
    {
        public int Compare(DStarHeapNode x, DStarHeapNode y)
        {
            int compareValue = 0;

            if (x.Priority[0] < y.Priority[0])
            {
                compareValue = -1;
            }
            else if (x.Priority[0] > y.Priority[0])
            {
                compareValue = 1;
            }
            else
            {
                if (x.Priority[1] < y.Priority[1])
                {
                    compareValue = -1;
                }
                else if (x.Priority[1] > y.Priority[1])
                {
                    compareValue = 1;
                }
            }

            return compareValue;
        }
    }
    
}