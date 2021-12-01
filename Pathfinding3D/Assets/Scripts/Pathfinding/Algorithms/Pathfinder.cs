using System;
using System.Collections.Generic;
using System.Diagnostics;
using AI;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Util;
using Debug = UnityEngine.Debug;

namespace Pathfinding.Algorithms
{
    public abstract class Pathfinder : MonoBehaviour
    {
        public static Action<List<float3>> OnNewCorridorList;

        [SerializeField] protected AIController ufoController;

        protected Stopwatch algTimer;
        protected NativeArray<bool> changed;
        protected NativeArray<int> changes;
        protected NativeList<float3> corridorList;
        protected int count;
        protected NativeArray<int> expandedArray;
        protected NativeHashMap<int, NativeHeapIndex> heapIndex;
        protected NativeArray<double> heuristic;
        private bool isRunning;
        protected bool isRunningJob;
        protected JobHandle jobHandle;
        protected NativeArray<int> lastStart, lastGoal;

        private List<float3> newCorridor;
        protected NativeArray<float> pathLength;
        protected bool resetAlgorithm;
        protected SearchGraphManager searchGraphManager;

        protected int totalExpanded;

        protected virtual bool WaitOnMainThread => PathfinderSetup.WaitOnMainThread;
        public abstract IPathfinderJob Job { get; set; }
        public PathfinderSetup PathfinderSetup { get; set; }

        private void Awake()
        {
            PlayerLoopHelper.SetCustomGameLoop(7,typeof(Pathfinder), PathfinderUpdate, true);   //post late updated
            if(!WaitOnMainThread)
            {
                PlayerLoopHelper.SetCustomGameLoop(2,typeof(Pathfinder), PathfinderComplete, true);   //early update
            }
        }

        private void PathfinderUpdate()
        {
            if (!isRunning)
            {
                return;
            }

            StartFindPath();
        }

        private void PathfinderComplete()
        {
            if (!isRunning)
            {
                return;
            }

            CompleteFindPath();
        }

        protected void OnEnable()
        {
            SearchGraphManager.OnSearchGraphDone += StartPathfinding;
            AIController.OnResetAI += ResetAlgorithm;
        }

        protected virtual void OnDisable()
        {
            SearchGraphManager.OnSearchGraphDone -= StartPathfinding;
            AIController.OnResetAI -= ResetAlgorithm;
            isRunning = false;

            jobHandle.Complete();
            expandedArray.Dispose();
            corridorList.Dispose();
            heapIndex.Dispose();

            lastGoal.Dispose();
            lastStart.Dispose();
            pathLength.Dispose();
            heuristic.Dispose();
            changed.Dispose();
            changes.Dispose();
        }

        protected virtual void StartPathfinding()
        {
            algTimer = new Stopwatch();
            searchGraphManager = SearchGraphSelector.Instance.SearchGraphManager;
            isRunning = true;

            newCorridor = new List<float3>();
            lastGoal = new NativeArray<int>(1, Allocator.Persistent);
            lastStart = new NativeArray<int>(2, Allocator.Persistent);
            pathLength = new NativeArray<float>(1, Allocator.Persistent);
            expandedArray = new NativeArray<int>(1, Allocator.Persistent);
            corridorList = new NativeList<float3>(Allocator.Persistent);
            changed = new NativeArray<bool>(1, Allocator.Persistent);
            heuristic = new NativeArray<double>(searchGraphManager.NodesCount, Allocator.Persistent);
            heapIndex = new NativeHashMap<int, NativeHeapIndex>(1000,Allocator.Persistent);

            IPathfinderJob job = Job;
            job.neighboursCountSize = searchGraphManager.neighbourCount;
            job.neighboursCount = searchGraphManager.neighbourCount.x * searchGraphManager.neighbourCount.y;

            Job = job;
        }

        private void StartFindPath()
        {
            if (!WaitOnMainThread)
            {
                CompleteFindPath();
            }
            if (!isRunningJob)
            {
                isRunningJob = true;

                PrepareJob();

                // algTimer.Restart();  //enable if you need to measure the time of each algorithm iteration
                algTimer.Start();
                ScheduleJob();
                resetAlgorithm = false;
            }

            if (WaitOnMainThread)
            {
                CompleteFindPath(); //enable if you need the main thread to wait for the algorithms (for example to measure the time of each algorithm iteration)
            }
        }

        private void CompleteFindPath()
        {
            if ((isRunningJob && jobHandle.IsCompleted) || WaitOnMainThread)  //disable the if-clause if you need to the main thread to wait for the algorithms (e.g. to measure the time of each algorithm iteration)
            {
                isRunningJob = false;
                jobHandle.Complete();

                algTimer.Stop();
                if (corridorList.Length > 0)
                {
                    newCorridor.Clear();
                    newCorridor.AddRange(corridorList.ToArray());
                    OnNewCorridorList(newCorridor);
                }

                // if (changed[0])
                // {
                //     count++;
                //     FileHandler.self.WriteDebugString(" algorithm time: " + algTimer.Elapsed.TotalMilliseconds +
                //                                       " : expanded: " + expandedArray[0]
                //                                       + ": path length: " + pathLength[0]
                //                                       + "\n");
                // }

                
                totalExpanded += expandedArray[0];
                changes.Dispose();
                DisposeAfterJob();

                if (count > PathfinderSetup.StopAtRunCount)
                {
                    Debug.Break();
                }
            }
        }

        protected virtual void PrepareJob()
        {
            expandedArray[0] = 0;
            corridorList.Clear();

            changes = new NativeArray<int>(searchGraphManager.ChangedNodes, Allocator.Persistent);
            changed[0] = false;

            IPathfinderJob job = Job;
            job.aiPosition = SearchGraphSelector.Instance.StartPosition;
            job.lookaheadPosition = ufoController.GetLookAheadPosition();
            job.goalPosition = SearchGraphSelector.Instance.GoalPosition;
            job.searchGraphOrigin = searchGraphManager.transform.position;
            job.cellSize = searchGraphManager.cellSize;
            job.searchSpace = new int3(searchGraphManager.searchLength, searchGraphManager.searchHeight, searchGraphManager.searchWidth);
            job.pathfinderSetup = PathfinderSetup;
            Job = job;

            if (resetAlgorithm)
            {
                ufoController.ResetCorridorMovement();
                count++;
                FileHandler.self.WriteDebugString(":total algorithm time: " + algTimer.ElapsedMilliseconds +
                                                  " : total expanded: " + totalExpanded+ "\n");
                algTimer.Restart();
                totalExpanded = 0;
            }
        }

        protected abstract void ScheduleJob();

        protected abstract void DisposeAfterJob();

        private void ResetAlgorithm()
        {
            resetAlgorithm = true;
        }
    }

    [Serializable]
    public struct PathfinderSetup
    {
        public float Epsilon;
        public bool WaitOnMainThread;
        public int StopAtRunCount;
    }
}