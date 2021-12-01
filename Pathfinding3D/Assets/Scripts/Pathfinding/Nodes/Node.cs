using System.Collections.Generic;
using JetBrains.Annotations;
using Priority_Queue;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public class Node : FastPriorityQueueNode
    {
        public double gValue; 

        public bool isInOpen,
            isInClosed,
            isInIncons,
            isChanged,
            isInOpenAfterChange,
            isNextNode,
            isLookaheadNode,
            isStartNode,
            isInTree,
            isInSubTree;

        public double[] priorityKey;
        public double rhsValue; 

        public Vector3 localPosition { get; set; }
        public Vector3 globalPosition { get; set; }
        public bool isFree { get; set; }
        public virtual List<Node> neighbours { get; set; }
        public virtual List<Node> predecessors { get; set; }
        public IDictionary<Node, float> pathCosts { get; set; }
        public Node parentNode { get; set; }

        public bool snapshotIsFree { get; set; }
        public Vector3 snapshotGlobalPosition { get; set; }
        public List<Node> snapshotNeighbours { get; set; }
        public List<Node> snapshotPredecessors { get; set; }
        public IDictionary<Node, float> snapshotPathCosts { get; set; }

        public bool publicIsFree { get; set; }
        public Vector3 publicGlobalPosition { get; set; }
        public List<Node> publicNeighbours { get; set; }
        public List<Node> publicPredecessors { get; set; }
        public IDictionary<Node, float> publicPathCosts { get; set; }

        public double heuristic { get; set; }
        public double plainHeuristic { get; set; }

        private bool isExpanded;
        public bool IsExpanded { get; set; }

        public Color ExpandedColor { get; set; }

        public void UpdateHeuristic(Vector3 goal, float inflate, bool updatePlain)
        {
            if (updatePlain)
            {
                UpdatePlainHeuristic(goal);
            }

            heuristic = plainHeuristic * inflate;
        }

        public void UpdatePlainHeuristic(Vector3 goal)
        {
            plainHeuristic = Vector3.Distance(goal, publicGlobalPosition);
        }

        public void Reset()
        {
            gValue = Mathf.Infinity;
            rhsValue = Mathf.Infinity;
            parentNode = null;

            isInClosed = false;
            isInIncons = false;
            isInOpen = false;
            isInOpenAfterChange = false;
            IsExpanded = false;

            priorityKey = new double[2];
            priorityKey[0] = Mathf.Infinity;
            priorityKey[1] = Mathf.Infinity;
        }

        public void ResetVisualization()
        {
            isInOpen = false;
            isInClosed = false;
            isExpanded = false;
        }
    }
}