using Pathfinding.SearchGraph;
using Util;
using Unity.Jobs;

namespace Pathfinding.Algorithms
{
    public class A_StarHybrid : HybridPathfinder
    {
        private A_StarHybridJob job;

        public override IPathfinderJob Job { get => job; set => job = (A_StarHybridJob)value; }

        protected override void StartPathfinding()
        {
            job = new A_StarHybridJob();
            base.StartPathfinding();
            hybridManager = searchGraphManager as HybridManager;
            FileHandler.self.WriteDebugString("A* with epsilon " + PathfinderSetup.Epsilon + " and search graph " +SearchGraphSelector.Instance.searchGraphType+ "\n");
            
            job.usedNodes = usedNodes;
            job.nodes = nodes;
            job.lastGoal = lastGoal;
            job.heuristic = heuristic;
            job.pathLength = pathLength;
            job.losOffsetMultiplier = losOffsetMultiplier;
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
            nodePathCosts.CopyFrom(hybridManager.AllNodesPathCosts);
            openList.Clear();
            heapIndex.Clear();

            job.nodePathCosts = nodePathCosts;
            job.openList = openList;
            job.heapIndex = heapIndex;
            
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

        protected override void OnDisable()
        {
            base.OnDisable();
            openList.Dispose();
        }
    }
}
