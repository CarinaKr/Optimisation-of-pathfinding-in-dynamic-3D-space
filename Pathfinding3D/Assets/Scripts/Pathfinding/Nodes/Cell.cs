﻿using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public class Cell : Node
    {
        public Cell(Vector3 localPosition, Vector3 globalPosition, float size)
        {
            //do additional initialization steps here
            this.size = size;
            this.localPosition = localPosition;
            this.globalPosition = globalPosition;

            gValue = Mathf.Infinity;
            rhsValue = Mathf.Infinity;

            snapshotNeighbours = new List<Node>();
            snapshotPredecessors = new List<Node>();
            snapshotPathCosts = new Dictionary<Node, float>();

            publicNeighbours = new List<Node>();
            publicPredecessors = new List<Node>();
            publicPathCosts = new Dictionary<Node, float>();

            priorityKey = new double[2];
            priorityKey[0] = Mathf.Infinity;
            priorityKey[1] = Mathf.Infinity;
        }

        public float size { get; set; }
    }
}