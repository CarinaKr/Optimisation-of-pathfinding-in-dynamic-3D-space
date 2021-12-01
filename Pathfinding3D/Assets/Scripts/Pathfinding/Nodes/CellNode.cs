using Unity.Mathematics;

namespace Pathfinding.Nodes
{
    public struct CellNode : INode, IFree, INeighbours3X2
    {
        public double gValue { get; set; }
        public double rhsValue { get; set; }
        public double priority { get; set; }
        public double2 priorityKey { get; set; }
        public int3 localPosition { get; set; }
        public float3 globalPosition { get; set; }
        public bool isFree { get; set; }
        public int3x2 neighbours { get; set; }
        public int3x2 orderedNeighbours { get; set; }
        public int3x2 predecessors { get; set; }
        public double3x2 pathCosts { get; set; }
        public int parentNode { get; set; }
        public double heuristic { get; set; }
        public double plainHeuristic { get; set; }
        public bool isInOpenList { get; set; }
        public bool isInClosedList { get; set; }
        public int index { get; set; }
        public int neighboursCount { get; set; }
        public bool isAtObstacle { get; set; }

        public CellNode(int3 localPosition, float3 globalPosition, int index)
        {
            this.localPosition = localPosition;
            this.globalPosition = globalPosition;
            this.index = index;
            
            gValue = math.INFINITY_DBL;
            rhsValue = math.INFINITY_DBL;

            priority = math.INFINITY_DBL;
            priorityKey = new double2();

            isFree = false;
            neighboursCount = 0;
            neighbours = new int3x2();
            orderedNeighbours = new int3x2();
            predecessors = new int3x2();
            pathCosts = new double3x2(math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL);
            
            parentNode = -1;
            heuristic = 0;
            plainHeuristic = 0;
            isInOpenList = false;
            isInClosedList = false;

            isAtObstacle = false;
        }
        
        public void Reset()
        {
            gValue = math.INFINITY_DBL;
            rhsValue = math.INFINITY_DBL;
            parentNode = -1;

            priority = math.INFINITY_DBL;
            double2 key = priorityKey;
            key[0] = math.INFINITY_DBL;
            key[1] = math.INFINITY_DBL;
            priorityKey = key;

            isInOpenList = false;
            isInClosedList = false;

            isAtObstacle = false;
        }
    }
}
