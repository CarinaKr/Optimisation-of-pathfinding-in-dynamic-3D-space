using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using UnityEngine;
using Util;

namespace Pathfinding.Algorithms
{
    public class HybridPathfinder : Pathfinder
    {
        [SerializeField] protected float losOffsetMultiplier = 0.3f;
        protected NativeHeap<HeapNode, HeapNodePriority> openList;
        protected HybridManager hybridManager;
        protected NativeArray<HybridNode> nodes;
        protected NativeArray<HybridNodePathCosts> nodePathCosts;
        protected NativeArray<int> usedNodes;
        protected NativeList<float> losTimes;

        protected override void StartPathfinding()
        {
            base.StartPathfinding();
            hybridManager = searchGraphManager as HybridManager;
            openList = new NativeHeap<HeapNode, HeapNodePriority>(Allocator.Persistent);
            nodes = new NativeArray<HybridNode>(hybridManager.AllNodes, Allocator.Persistent);
            usedNodes = new NativeArray<int>(hybridManager.UsedNodes, Allocator.Persistent);
            nodePathCosts = new NativeArray<HybridNodePathCosts>(hybridManager.AllNodesPathCosts, Allocator.Persistent);
            losTimes = new NativeList<float>(Allocator.Persistent);
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
            usedNodes.Dispose();
            nodes.Dispose();
            nodePathCosts.Dispose();
            losTimes.Dispose();
        }
    }
}
