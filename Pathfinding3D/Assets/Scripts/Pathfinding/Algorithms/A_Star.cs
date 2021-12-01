using Pathfinding.Nodes;
using Unity.Collections;
using Util;
using Unity.Jobs;

namespace Pathfinding.Algorithms
{
    public class A_Star : CellGridPathfinder
    {
        private A_StarJob job;

        public override IPathfinderJob Job { get => job; set => job = (A_StarJob)value; }

        protected override void StartPathfinding()
        {
            job = new A_StarJob();
            base.StartPathfinding();
            FileHandler.self.WriteDebugString("A* with epsilon " + PathfinderSetup.Epsilon + "\n");
            
            job.lastGoal = lastGoal;
            job.heuristic = heuristic;
            job.pathLength = pathLength;
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
            nodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);

            openList.Clear();
            heapIndex.Clear();

            job.openList = openList;
            job.heapIndex = heapIndex;
            job.nodes = nodes;
            job.expanded = expandedArray;
            job.corridor = corridorList;
            job.changes = changes;
            job.update = changed;
        }

        protected override void ScheduleJob()
        {
            jobHandle = job.Schedule();
            JobHandle.ScheduleBatchedJobs();
        }

        protected override void DisposeAfterJob()
        {
            nodes.Dispose();
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            openList.Dispose();
        }
    }
}
