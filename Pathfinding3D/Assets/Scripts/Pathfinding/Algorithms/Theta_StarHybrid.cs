using Pathfinding.SearchGraph;
using UnityEngine;
using Util;
using Unity.Jobs;

namespace Pathfinding.Algorithms
{
    public class Theta_StarHybrid : HybridPathfinder
    {
        [SerializeField] private bool useFastMode;
        private Theta_StarHybridJob job;

        public override IPathfinderJob Job { get => job; set => job = (Theta_StarHybridJob)value; }

        protected override void StartPathfinding()
        {
            job = new Theta_StarHybridJob();
            base.StartPathfinding();
            FileHandler.self.WriteDebugString("Theta* with epsilon " + PathfinderSetup.Epsilon+", fast = "+useFastMode + " and search graph " +SearchGraphSelector.Instance.searchGraphType+ "\n");

            job.nodes = nodes;
            job.lastGoal = lastGoal;
            job.pathLength = pathLength;
            job.heuristic = heuristic;
            job.usedNodes = usedNodes;
            job.losOffsetMultiplier = losOffsetMultiplier;
            job.losTimes = losTimes;
            job.fastMode = useFastMode;
            job.lastStart = lastStart;
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
            job.resetAlgorithm = resetAlgorithm;
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