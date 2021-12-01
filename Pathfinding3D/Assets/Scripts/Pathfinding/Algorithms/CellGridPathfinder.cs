using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Util;

namespace Pathfinding.Algorithms
{
    public class CellGridPathfinder : Pathfinder
    {
        
        protected CellGridManager cellGridManager;
        protected NativeHeap<CellNode, AStarBasedPriority> openList;
        protected NativeArray<CellNode> nodes;

        protected override void StartPathfinding()
        {
            base.StartPathfinding();
            cellGridManager = searchGraphManager as CellGridManager;
            openList = new NativeHeap<CellNode, AStarBasedPriority>(Allocator.Persistent);
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
        }

        protected override void ScheduleJob()
        {
        }

        protected override void DisposeAfterJob()
        {
        }

        public override IPathfinderJob Job { get; set; }

        protected override void OnDisable()
        {
            base.OnDisable();
            nodes.Dispose();
        }
    }
}
