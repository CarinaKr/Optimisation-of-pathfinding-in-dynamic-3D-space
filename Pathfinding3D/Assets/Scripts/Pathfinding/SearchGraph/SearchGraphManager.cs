using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.SearchGraph
{
    public abstract class SearchGraphManager : MonoBehaviour
    {
        public delegate void SearchGraphDone();
        public static event SearchGraphDone OnSearchGraphDone;
        
        public int searchWidth, searchHeight, searchLength;
        public float cellSize;
        [SerializeField] protected GameObject cellPrefab;
        [SerializeField] protected LayerMask obstacleLayer;
        [SerializeField] protected VisualizationHelper visualizationHelper;

        protected Stopwatch updateTimer;
        protected int3 searchSpace;
        protected float3 _cellSize;
        protected float3 _halfCellSize;
        protected QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Collide;
        protected Quaternion quaternionIdentity;
        protected Collider[] results = new Collider[1];
        protected HashSet<int> changedNodesSet;

        protected abstract SearchGraphSelector.SearchGraphType searchGraphType { get; }
        

        [HideInInspector] public int2 neighbourCount = new int2(2, 3);

        public int[] ChangedNodes
        {
            get
            {
                int[] array = changedNodesSet.ToArray();
                changedNodesSet.Clear();
                return array;
            }
        }

        public Color CurrentExpandedColor => visualizationHelper.ExpandedColor;
        public abstract int NodesCount { get; }

        protected abstract void CreateSearchGraph();
        protected abstract void UpdateSearch();

        protected virtual void Start()
        {
            searchSpace = new int3(searchLength, searchHeight, searchWidth);
            updateTimer = new Stopwatch();
            _cellSize = new Vector3(cellSize, cellSize, cellSize);
            _halfCellSize = _cellSize / 2;

            HybridManager hybridManager = this as HybridManager;
            if (hybridManager != null && hybridManager.IsBakingSearchGraph)
            {
                return;
            }
            CreateSearchGraph();
            if (OnSearchGraphDone != null)
            {
                OnSearchGraphDone();
            }
        }

        protected int3x2 Neighbours(int x, int y, int z, out int neighboursCount)
        {
            int3x2 neighbours = 0;
            neighboursCount = 0;
            int2 position = new int2();

            if (x > 0)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x - 1, y, z, searchSpace);
                neighboursCount++;
            }

            if (y > 0)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x, y - 1, z, searchSpace);
                neighboursCount++;
            }

            if (z > 0)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x, y, z - 1, searchSpace);
                neighboursCount++;
            }

            if (x < searchLength - 1)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x + 1, y, z, searchSpace);
                neighboursCount++;
            }

            if (y < searchHeight - 1)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x, y + 1, z, searchSpace);
                neighboursCount++;
            }

            if (z < searchWidth - 1)
            {
                position = CellGridManagerHelper.IndexToPosition(neighboursCount, neighbourCount.x, neighbourCount.y, ref position);
                neighbours[position.x][position.y] = CellGridManagerHelper.PositionToIndex(x, y, z + 1, searchSpace);
                neighboursCount++;
            }

            return neighbours;
        }
        protected int3x2 OrderedNeighbours(int x, int y, int z)
        {
            int3x2 neighbours = -1;

            neighbours[0][0] = CellGridManagerHelper.PositionToIndex(x + 1, y, z, searchSpace);
            neighbours[1][0] = CellGridManagerHelper.PositionToIndex(x - 1, y, z, searchSpace);
            neighbours[0][1] = CellGridManagerHelper.PositionToIndex(x, y + 1, z, searchSpace);
            neighbours[1][1] = CellGridManagerHelper.PositionToIndex(x, y - 1, z, searchSpace);
            neighbours[0][2] = CellGridManagerHelper.PositionToIndex(x, y, z + 1, searchSpace);
            neighbours[1][2] = CellGridManagerHelper.PositionToIndex(x, y, z - 1, searchSpace);

            return neighbours;
        }
    }
}