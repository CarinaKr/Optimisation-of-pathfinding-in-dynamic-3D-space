using System.Collections;
using System.Collections.Generic;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using UnityEngine;
using Util;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Profiling;

namespace Pathfinding.Algorithms
{
    public class Theta_Star : CellGridPathfinder
    {
        [SerializeField] private float losOffsetMultiplier = 0.3f;
        private Theta_StarJob job;
        private NativeList<float> losTimes;

        public override IPathfinderJob Job { get => job; set => job = (Theta_StarJob)value; }

        protected override void StartPathfinding()
        {
            job = new Theta_StarJob();
            base.StartPathfinding();
            FileHandler.self.WriteDebugString("Theta* with epsilon " + PathfinderSetup.Epsilon + "\n");
            losTimes = new NativeList<float>(Allocator.Persistent);

            job.lastGoal = lastGoal;
            job.pathLength = pathLength;
            job.heuristic = heuristic;
            job.losTimes = losTimes;
            job.losOffsetMultiplier = losOffsetMultiplier;
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
            losTimes.Dispose();
        }
    }
}