using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using Util;
using Debug = UnityEngine.Debug;

namespace Pathfinding.Algorithms
{
    public class ReverseMTD_StarLite : CellGridPathfinder
    {
        public bool useBasicMT = true;

        public override IPathfinderJob Job
        {
            get => job;
            set => job = (ReverseMTD_StarLiteJob) value;
        }
        private ReverseMTD_StarLiteJob job;
        
        private NativeHeap<CellNode, DStarBasedPriority> dStarBasedOpenList;
        private NativeHashSet<int> searchTree;
        private NativeHashSet<int> subTree;
        private NativeList<int> diffTree;
        private NativeArray<CellNode> globalNodes;
        private NativeArray<float> km;
        

        protected override void StartPathfinding()
        {
            job = new ReverseMTD_StarLiteJob();
            base.StartPathfinding();
            FileHandler.self.WriteDebugString("Reverse MT-D* Lite with epsilon " + PathfinderSetup.Epsilon + " and Basic MT=" + useBasicMT + "\n");
            resetAlgorithm = true;
            dStarBasedOpenList = new NativeHeap<CellNode, DStarBasedPriority>(Allocator.Persistent);
            searchTree = new NativeHashSet<int>(cellGridManager.searchLength * cellGridManager.searchHeight * cellGridManager.searchWidth, Allocator.Persistent);
            subTree = new NativeHashSet<int>(cellGridManager.searchLength * cellGridManager.searchHeight * cellGridManager.searchWidth, Allocator.Persistent);
            diffTree = new NativeList<int>(cellGridManager.searchLength * cellGridManager.searchHeight * cellGridManager.searchWidth, Allocator.Persistent);
            km = new NativeArray<float>(1, Allocator.Persistent);
            globalNodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);
            nodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);
            job.openList = dStarBasedOpenList;
            job.heapIndex = heapIndex;
            job.searchTree = searchTree;
            job.subTree = subTree;
            job.lastGoal = lastGoal;
            job.lastStart = lastStart;
            job.nodes = nodes;
            job.km = km;
            job.pathLength = pathLength;
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            globalNodes.Dispose();
            searchTree.Dispose();
            subTree.Dispose();
            diffTree.Dispose();
            km.Dispose();
            dStarBasedOpenList.Dispose();
            openList.Dispose();
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
            // globalNodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);
            globalNodes.CopyFrom(cellGridManager.Nodes);
           
            job.globalNodes = globalNodes;
            job.openList = dStarBasedOpenList;
            job.expanded = expandedArray;
            job.corridor = corridorList;
            job.resetAlgorithm = resetAlgorithm;
            job.useBasicMT = useBasicMT;
            job.changes = changes;
            job.update = changed;

            if (resetAlgorithm)
            {
                nodes.Dispose();
                nodes = new NativeArray<CellNode>(cellGridManager.Nodes, Allocator.Persistent);
                job.nodes = nodes;
            }
        }

        protected override void ScheduleJob()
        {
            jobHandle = job.Schedule();
            JobHandle.ScheduleBatchedJobs();
        }
    }
}