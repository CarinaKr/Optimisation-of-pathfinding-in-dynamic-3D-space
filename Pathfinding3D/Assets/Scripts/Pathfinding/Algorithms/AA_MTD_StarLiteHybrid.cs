using System.Collections.Generic;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Util;

namespace Pathfinding.Algorithms
{
    public class AA_MTD_StarLiteHybrid : HybridPathfinder
    {
        [SerializeField] private bool useBasicMT = true;
        [SerializeField] private bool useFastVersion;
        [SerializeField] private float plainHeuristicMultiplier = 0.5f;

        public override IPathfinderJob Job
        {
            get => job;
            set => job = (AA_MTD_StarLiteHybridJob) value;
        }
        private AA_MTD_StarLiteHybridJob job;
        
        private NativeHeap<DStarHeapNode, DStarHeapNodePriority> dStarBasedOpenList;
        private NativeHashSet<int> searchTree;
        private NativeHashSet<int> subTree;
        private NativeList<int> diffTree;
        private NativeArray<float> km;
        private NativeArray<double> plainHeuristic;
        private NativeMultiHashMap<int, int> predecessors;
        private NativeMultiHashMap<int, int> childNodes;
        private NativeHashSet<int2> parentChildTuples;
        private NativeHashMap<int, bool> losToParent;
        private NativeHashSet<int> collectedParents;
        

        protected override void StartPathfinding()
        {
            job = new AA_MTD_StarLiteHybridJob();
            base.StartPathfinding();
            FileHandler.self.WriteDebugString("Any-angle MT-D* Lite with epsilon " + PathfinderSetup.Epsilon+", plain mult. "+plainHeuristicMultiplier + ", Basic MT=" + useBasicMT+", fast = "+useFastVersion + " and search graph " +SearchGraphSelector.Instance.searchGraphType+ "\n");
            resetAlgorithm = true;
            dStarBasedOpenList = new NativeHeap<DStarHeapNode, DStarHeapNodePriority>(Allocator.Persistent);
            searchTree = new NativeHashSet<int>(hybridManager.searchLength * hybridManager.searchHeight * hybridManager.searchWidth, Allocator.Persistent);
            subTree = new NativeHashSet<int>(hybridManager.searchLength * hybridManager.searchHeight * hybridManager.searchWidth, Allocator.Persistent);
            diffTree = new NativeList<int>(hybridManager.searchLength * hybridManager.searchHeight * hybridManager.searchWidth, Allocator.Persistent);
            km = new NativeArray<float>(1, Allocator.Persistent);
            plainHeuristic = new NativeArray<double>(searchGraphManager.NodesCount, Allocator.Persistent);
            childNodes = new NativeMultiHashMap<int, int>(usedNodes.Length, Allocator.Persistent);
            parentChildTuples = new NativeHashSet<int2>(1, Allocator.Persistent);
            losToParent = new NativeHashMap<int, bool>(1, Allocator.Persistent);
            collectedParents = new NativeHashSet<int>(100, Allocator.Persistent);
            
            predecessors = new NativeMultiHashMap<int, int>(0, Allocator.Persistent);
            foreach (string key in hybridManager.PredecessorDictionary.Keys)
            {
                List<object> pred = hybridManager.PredecessorDictionary[key] as List<object>;
                foreach (long entry in pred)
                {
                    predecessors.Add(int.Parse(key), (int)entry);
                }
            }

            job.nodes = nodes;
            job.openList = dStarBasedOpenList;
            job.heapIndex = heapIndex;
            job.searchTree = searchTree;
            job.subTree = subTree;
            job.lastGoal = lastGoal;
            job.lastStart = lastStart;
            job.nodes = nodes;
            job.km = km;
            job.pathLength = pathLength;
            job.heuristic = heuristic;
            job.plainHeuristic = plainHeuristic;
            job.edgesCount = hybridManager.edgesCount;
            job.verticesCount = hybridManager.verticesCount;
            job.predecessorsMap = predecessors;
            job.usedNodes = usedNodes;
            job.losOffsetMultiplier = losOffsetMultiplier;
            job.childNodes = childNodes;
            job.losTimes = losTimes;
            job.parentChildTuples = parentChildTuples;
            job.collectedParents = collectedParents;
            job.plainHeuristicMultiplier = plainHeuristicMultiplier;
            job.useFastVersion = useFastVersion;
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            searchTree.Dispose();
            subTree.Dispose();
            diffTree.Dispose();
            km.Dispose();
            plainHeuristic.Dispose();
            childNodes.Dispose();
            parentChildTuples.Dispose();
            losToParent.Dispose();
            collectedParents.Dispose();
            dStarBasedOpenList.Dispose();
            openList.Dispose();
            predecessors.Dispose();
        }

        protected override void PrepareJob()
        {
            base.PrepareJob();
            nodePathCosts.CopyFrom(hybridManager.AllNodesPathCosts);

            job.nodePathCosts = nodePathCosts;
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
                nodes = new NativeArray<HybridNode>(hybridManager.AllNodes, Allocator.Persistent);
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