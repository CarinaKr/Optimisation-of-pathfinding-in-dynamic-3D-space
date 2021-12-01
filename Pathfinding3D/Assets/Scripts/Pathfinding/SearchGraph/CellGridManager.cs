using System.Collections.Generic;
using Pathfinding.Nodes;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.SearchGraph
{
    public class CellGridManager : SearchGraphManager
    {
        private bool[,,] oldFree, newFree;
        
        private List<int> freeBlockedChangedNodes;
        private CellNode[] updatedNodes;
        private CellNode tempNode;
        private int currentPredecessor;
        private int2 pos;
        private double3x2 pathCosts = new double3x2(math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL, math.INFINITY_DBL);
        
        protected override SearchGraphSelector.SearchGraphType searchGraphType => SearchGraphSelector.SearchGraphType.CELL_GRID;
        public CellNode[] Nodes { get; set; }
        public override int NodesCount => Nodes.Length;

        protected override void Start()
        {
            oldFree = new bool[searchLength, searchHeight, searchWidth];
            newFree = new bool[searchLength, searchHeight, searchWidth];
            Nodes = new CellNode[searchLength * searchHeight * searchWidth];
            

            freeBlockedChangedNodes = new List<int>();
            changedNodesSet = new HashSet<int>();
            quaternionIdentity = Quaternion.identity;
            
            base.Start();
        }

        private void Update()
        {
            UpdateSearch();
        }

        protected override void CreateSearchGraph()
        {
            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                int index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                CellNode node = CellObject.Create(cellPrefab, transform, new int3(x, y, z), cellSize, index);
                Nodes[index] = node;
            }

            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                int index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                CellNode tempNode = Nodes[index];
                tempNode.neighbours = Neighbours(x, y, z, out int neighboursCount);
                tempNode.orderedNeighbours = OrderedNeighbours(x, y, z);
                tempNode.neighboursCount = neighboursCount;
                tempNode.predecessors = tempNode.neighbours;
                Nodes[index] = tempNode;
            }

            CheckOccupation();
            UpdatePathCosts();
        }

        private CellNode[] updateNodes;
        private CellNode[] CheckOccupation()
        {
            updateNodes = Nodes;
            int index = 0;
            bool isFree = false;
            for (int z = 0; z < searchWidth; z++)
            {
                for (int y = 0; y < searchHeight; y++)
                {
                    for (int x = 0; x < searchLength; x++)
                    {
                        //check if cell is obstructed
                        index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                        oldFree[x, y, z] = Nodes[index].isFree;

                        isFree = Physics.OverlapBoxNonAlloc(Nodes[index].globalPosition, _halfCellSize, results, quaternionIdentity, obstacleLayer, triggerInteraction) <= 0;
                        updateNodes[index].isFree = isFree;
                        newFree[x, y, z] = isFree;
                    }
                }
            }

            return updateNodes;
        }

        private void UpdatePathCosts()
        {
            for (int i = 0; i < Nodes.Length; i++)
            {
                CellNode tempNode = Nodes[i];
                tempNode.pathCosts = UpdatePathCosts(i, out bool isAtObstacle);
                tempNode.isAtObstacle = isAtObstacle;
                Nodes[i] = tempNode;
            }
        }
        
        private double3x2 UpdatePathCosts(int nodeIndex, out bool isAtObstacle)
        {
            isAtObstacle = false;
            double pathCost;
            pathCosts = math.INFINITY_DBL;

            for (int i = 0; i < Nodes[nodeIndex].neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighbourCount, ref pos);
                int neighbour = Nodes[nodeIndex].neighbours[pos.x][pos.y];
                if (!Nodes[nodeIndex].isFree || !Nodes[neighbour].isFree)
                {
                    pathCost = math.INFINITY_DBL;
                    isAtObstacle = true;
                }
                else
                {
                    pathCost = cellSize;
                }
            
                pathCosts[pos.x][pos.y] = pathCost;
            }
            
            return pathCosts;
        }

        protected override void UpdateSearch()
        {
            updateTimer.Restart();
            freeBlockedChangedNodes.Clear();
            updatedNodes = Nodes;
            updatedNodes = CheckOccupation();

            for (int z = 0; z < searchWidth; z++) //check which nodes have been changed
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            { 
                if (newFree[x, y, z] != oldFree[x, y, z])
                {
                    freeBlockedChangedNodes.Add(CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace));
                }
            }
            
            foreach (int nodeIndex in freeBlockedChangedNodes) //update path costs of all changed nodes
            {
                if (!changedNodesSet.Contains(nodeIndex)) //make sure to check each node only once
                {
                    tempNode = Nodes[nodeIndex];
                    tempNode.pathCosts = UpdatePathCosts(nodeIndex, out bool isAtObstacle);
                    tempNode.isAtObstacle = isAtObstacle;
                    updatedNodes[nodeIndex] = tempNode;
                    changedNodesSet.Add(nodeIndex); //remove already checked nodes from copied list
                }
            
                for (int i = 0; i < Nodes[nodeIndex].neighboursCount; i++) //update path costs of all predecessors
                {
                    CellGridManagerHelper.IndexToPosition(i, neighbourCount, ref pos);
                    currentPredecessor = Nodes[nodeIndex].predecessors[pos.x][pos.y];
                    if (!changedNodesSet.Contains(currentPredecessor))
                    {
                        tempNode = Nodes[currentPredecessor];
                        tempNode.pathCosts = UpdatePathCosts(currentPredecessor, out bool isAtObstacle);
                        tempNode.isAtObstacle = isAtObstacle;
                        updatedNodes[currentPredecessor] = tempNode;
                        changedNodesSet.Add(currentPredecessor);
                    }
                }
            }
            
            Nodes = updatedNodes;
        }
    }
}