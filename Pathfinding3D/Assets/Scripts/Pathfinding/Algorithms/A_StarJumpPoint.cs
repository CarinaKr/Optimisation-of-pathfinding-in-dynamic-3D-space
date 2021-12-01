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
    public class A_StarJumpPoint : CellGridPathfinder
    {
        private A_StarJumpPointJob job;
        private NativeList<int> neighbourIndices, hasNeighbourIndices;
        private NativeList<int> successors;
        private NativeHashSet<int> expandedNodes;
        private NativeHashMap<int, int3x2> forcedNeighboursOfNode;
        private NativeHashMap<int, int3x2> jumpPointsOfNode;
        private NativeArray<int3> normalizedDirections;

        public override IPathfinderJob Job { get => job; set => job = (A_StarJumpPointJob)value; }

        protected override void StartPathfinding()
        {
            job = new A_StarJumpPointJob();
            base.StartPathfinding();
            cellGridManager = searchGraphManager as CellGridManager;
            FileHandler.self.WriteDebugString("Jump point A* with epsilon " + PathfinderSetup.Epsilon + "\n");
            neighbourIndices = new NativeList<int>(Allocator.Persistent);
            hasNeighbourIndices = new NativeList<int>(Allocator.Persistent);
            successors = new NativeList<int>(Allocator.Persistent);
            expandedNodes = new NativeHashSet<int>(5000, Allocator.Persistent);
            forcedNeighboursOfNode = new NativeHashMap<int, int3x2>(5000, Allocator.Persistent);
            jumpPointsOfNode = new NativeHashMap<int, int3x2>(5000, Allocator.Persistent);
            normalizedDirections = new NativeArray<int3>(6, Allocator.Persistent);
            normalizedDirections[0] = new int3(1, 0, 0);
            normalizedDirections[1] = new int3(-1, 0, 0);
            normalizedDirections[2] = new int3(0, 1, 0);
            normalizedDirections[3] = new int3(0, -1, 0);
            normalizedDirections[4] = new int3(0, 0, 1);
            normalizedDirections[5] = new int3(0, 0, -1);
            
            job.lastGoal = lastGoal;
            job.neighbourIndices = neighbourIndices;
            job.successor = successors;
            job.forcedNeighboursOfNode = forcedNeighboursOfNode;
            job.jumpPointsOfNode = jumpPointsOfNode;
            job.normalizedDirections = normalizedDirections;
            job.heuristic = heuristic;
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
            nodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);

            openList.Clear();
            heapIndex.Clear();
            expandedNodes.Clear();
            neighbourIndices.Clear();
            successors.Clear();
            hasNeighbourIndices.Clear();
            forcedNeighboursOfNode.Clear();
            jumpPointsOfNode.Clear();

            job.openList = openList;
            job.heapIndex = heapIndex;
            job.nodes = nodes;
            job.expanded = expandedArray;
            job.corridor = corridorList;
            job.changes = changes;
            job.update = changed;
        }

        protected override void DisposeAfterJob()
        {
            base.DisposeAfterJob();
            nodes.Dispose();
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
            neighbourIndices.Dispose();
            successors.Dispose();
            hasNeighbourIndices.Dispose();
            expandedNodes.Dispose();
            forcedNeighboursOfNode.Dispose();
            normalizedDirections.Dispose();
            jumpPointsOfNode.Dispose();
        }
    }
}
