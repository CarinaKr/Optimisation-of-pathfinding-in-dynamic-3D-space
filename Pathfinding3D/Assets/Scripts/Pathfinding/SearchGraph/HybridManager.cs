using System.Collections;
using System.Collections.Generic;
using MiniJSON;
using Obstacles;
using Pathfinding.Nodes;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.LowLevel;
using UnityEngine.Rendering;
using Util;

namespace Pathfinding.SearchGraph
{
    public class HybridManager : SearchGraphManager
    {
        
        [SerializeField] private LayerMask staticObstaclesLayer;
        [SerializeField] private LayerMask dynamicObstaclesLayer;
        [SerializeField] private VisualizeMode visualizeMode;
        [SerializeField, ShowIf(nameof(visualizeMode), Nodes.VisualizeMode.Nodes)] private VisualizeNodes visualizeNodes;
        [SerializeField, ShowIf(nameof(visualizeMode), Nodes.VisualizeMode.Nodes)] private VisualizeNeighbours visualizeNeighbours;

        private DynamicObstacle[] dynamicObstacles;
        private bool[] blockedByStatic;
        private HybridNode[] allNodes;
        private HybridNodePathCosts[] allPathCosts;
        private int3[] normalizedDirections;
        private int3[] normalizedEdgeDirections;
        private int3[] normalizedVertexDirections;
        private int2x2[] splitEdgeOrderedSideNeighbours;
        private int2x3[] splitVertexOrderedSideNeighbours;
        private int2x3[] splitVertexOrderedEdgeNeighbours;
        private SerializedDictionary<int, List<int>> predecessors;
        private Dictionary<string, object> predDictionary;
        private List<int> dynamicNodes;
        private NativeArray<float3> basicAxis;
        private NativeMultiHashMap<int, float3> localObstacleVertices, localObstacleNormals, localObstacleEdges;
        private NativeList<float3> globalObstacleVertices, globalObstacleNormals, globalObstacleEdges;
        private NativeList<float3> allAxis;
        private NativeMultiHashMap<int, float3> globalNodeVerticesMap;
        private NativeArray<int> dynamicNodesNativeArray;
        private UpdateOverlapJob job;
        private UpdatePathCostsJob updatePathCostsJob;
        private NativeArray<bool> nodeIsBlocked;
        private NativeList<int> changedNodesList;
        private NativeArray<HybridNode> allHybridNodes;
        private NativeArray<HybridNodePathCosts> nodePathCosts;
        private int[] usedNodes;
        private JobHandle updatePathCostsJobHandle;
        private NativeArray<float4x4> obstaclesLocalToWorldMatrix;
        
        protected override SearchGraphSelector.SearchGraphType searchGraphType => SearchGraphSelector.SearchGraphType.HYBRID;
        [HideInInspector] public int2 edgesCount = new int2(3, 4);
        [HideInInspector] public int2 verticesCount = new int2(2, 4);
        [field: SerializeField, DebugField] public bool IsBakingSearchGraph { get; set; }
        
        public VisualizeNodes VisualizeNodes => visualizeNodes;
        public VisualizeNeighbours VisualizeNeighbours => visualizeNeighbours;
        public VisualizeMode VisualizeMode => visualizeMode;
        public NativeArray<HybridNode> AllNodes => allHybridNodes;
        public override int NodesCount => allNodes.Length;
        public int[] UsedNodes => usedNodes;
        public Dictionary<string, object> PredecessorDictionary => predDictionary;
        public NativeArray<HybridNodePathCosts> AllNodesPathCosts => nodePathCosts;

        protected override void Start()
        {
            allNodes = new HybridNode[searchLength * searchHeight * searchWidth];
            normalizedDirections = CellGridManagerHelper.GetNormalizedDirections();
            normalizedEdgeDirections = CellGridManagerHelper.GetNormalizedEdgeDirections();
            normalizedVertexDirections = CellGridManagerHelper.GetNormalizedVertexDirections();
            splitEdgeOrderedSideNeighbours = CellGridManagerHelper.GetSplitEdgeOrderedNeighbourIndices();
            splitVertexOrderedSideNeighbours = CellGridManagerHelper.GetSplitVertexOrderedNeighbourIndices();
            splitVertexOrderedEdgeNeighbours = CellGridManagerHelper.GetSplitVertexOrderedEdgesIndices();
            
            base.Start();

            if (IsBakingSearchGraph)
            {
                StartCoroutine(BakeSearchGraph());
            }
            else
            {
                PlayerLoopHelper.SetCustomGameLoop(5,typeof(SearchGraphManager), UpdateSearch, false);  //update
                PlayerLoopHelper.SetCustomGameLoop(6,typeof(SearchGraphManager), CompleteSearchGraphUpdate, false);  //pre late update
            }
        }

        public IEnumerator BakeSearchGraph()
        {
            IsBakingSearchGraph = true;
            predecessors = new SerializedDictionary<int, List<int>>();
            CreateCellGridSearchGraph();
            SetAllNodesNeighbours();
            CreateStaticSearchGraph();
            yield return StartCoroutine(CreateDynamicSearchGraph());
            UpdateNeighbours();
            SaveSearchGraph();
            InstantiateCellNodes();
#if UNITY_EDITOR
            EditorApplication.isPlaying = false;
#endif
        }
        
        protected override void CreateSearchGraph()
        {
            if (IsBakingSearchGraph)
            {
                return;
            }

            changedNodesSet = new HashSet<int>();
            FileHandler.self.ReadSaveFile(out string nodesString, out string predecessorsString);
            predDictionary = Json.Deserialize(predecessorsString) as Dictionary<string, object>;

            CreateCellGridSearchGraph();
            LoadSearchGraph(nodesString);

            BlockNodes(staticObstaclesLayer);
            InitUpdateJob();
            InstantiateCellNodes();
        }

        private void CreateCellGridSearchGraph()
        {
            allNodes = new HybridNode[searchLength * searchHeight * searchWidth];
            allPathCosts = new HybridNodePathCosts[allNodes.Length];
            searchSpace = new int3(searchLength, searchHeight, searchWidth);
            
            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                int index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                int3 localPosition = new int3(x, y, z);
                float3 globalPosition = (float3)transform.position + localPosition;
                HybridNode node = new HybridNode(localPosition, globalPosition, index);
                allNodes[index] = node;
                allPathCosts[index] = new HybridNodePathCosts(true);   
            }
        }
        
        private void CreateStaticSearchGraph()
        {
            blockedByStatic = new bool[allNodes.Length];
            CumulateBlockedNodes(staticObstaclesLayer, ref blockedByStatic);
            List<int> staticObstaclesEdges = GetEdgeNodes(blockedByStatic);
            for (int i = 0; i < staticObstaclesEdges.Count; i++)
            {
                allNodes[staticObstaclesEdges[i]].isStaticNode = true;
            }
        }

        private IEnumerator CreateDynamicSearchGraph()
        {
            bool[] blockedByDynamic = new bool[allNodes.Length];
            
            while (IsBakingSearchGraph)
            {
                CumulateBlockedNodes(dynamicObstaclesLayer, ref blockedByDynamic);
                yield return null;
            }

            for (int i = 0; i < blockedByDynamic.Length; i++)
            {
                if(!blockedByDynamic[i] || blockedByStatic[i])
                {
                    continue;
                }
                
                allNodes[i].isDynamicNode = true;
                
                SetAllDynamicNeighboursStatus(i);
            }
        }

        private void SetAllNodesNeighbours()
        {
            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                int index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                HybridNode tempNode = allNodes[index];
                
                //face neighbours
                tempNode.neighbours = Neighbours(x, y, z, out int neighboursCount);
                tempNode.orderedNeighbours = OrderedNeighbours(x, y, z);
                tempNode.neighboursCount = neighboursCount;
                
                //edge neighbours
                tempNode.edgeNeighbours = Edges(x, y, z, out int4x3 orderedEdges, out int edgesCounter);
                tempNode.orderedEdgeNeighbours = orderedEdges;
                tempNode.edgesCount = edgesCounter;
                
                //vertex neighbours
                tempNode.verticesNeighbours = Vertices(x, y, z, out int4x2 orderedVertices, out int verticesCounter);
                tempNode.orderedVerticesNeighbours = orderedVertices;
                tempNode.verticesCount = verticesCounter;
                
                allNodes[index] = tempNode;
            }
        }

        private int4x2 Vertices(int x, int y, int z, out int4x2 orderedVertices, out int counter)
        {
            orderedVertices = -1;
            int4x2 vertices = 0;
            counter = 0;
            int2 position = 0;
            int2 pos = 0;
            
            orderedVertices[0][0] = CellGridManagerHelper.PositionToIndex(x + 1, y + 1, z + 1, searchSpace);
            orderedVertices[1][0] = CellGridManagerHelper.PositionToIndex(x + 1, y + 1, z - 1, searchSpace);
            orderedVertices[0][1] = CellGridManagerHelper.PositionToIndex(x - 1, y + 1, z + 1, searchSpace);
            orderedVertices[1][1] = CellGridManagerHelper.PositionToIndex(x - 1, y + 1, z - 1, searchSpace);
            orderedVertices[0][2] = CellGridManagerHelper.PositionToIndex(x + 1, y - 1, z + 1, searchSpace);
            orderedVertices[1][2] = CellGridManagerHelper.PositionToIndex(x + 1, y - 1, z - 1, searchSpace);
            orderedVertices[0][3] = CellGridManagerHelper.PositionToIndex(x - 1, y - 1, z + 1, searchSpace);
            orderedVertices[1][3] = CellGridManagerHelper.PositionToIndex(x - 1, y - 1, z - 1, searchSpace);
            
            for (int i = 0; i < 8; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, verticesCount.x, verticesCount.y, ref position);
                if (orderedVertices[position.x][position.y] != -1)
                {
                    CellGridManagerHelper.IndexToPosition(counter, verticesCount.x, verticesCount.y, ref pos);
                    vertices[pos.x][pos.y] = orderedVertices[position.x][position.y];
                    counter++;
                }
            }

            return vertices;
        }
        
        private int4x3 Edges(int x, int y, int z, out int4x3 orderedEdges, out int counter)
        {
            orderedEdges = -1;
            int4x3 edges = 0;
            counter = 0;
            int2 position = 0;
            int2 pos = 0;
            
            orderedEdges[0][0] = CellGridManagerHelper.PositionToIndex(x + 1, y + 1, z, searchSpace);
            orderedEdges[1][0] = CellGridManagerHelper.PositionToIndex(x - 1, y + 1, z, searchSpace);
            orderedEdges[2][0] = CellGridManagerHelper.PositionToIndex(x, y + 1, z + 1, searchSpace);
            orderedEdges[0][1] = CellGridManagerHelper.PositionToIndex(x, y + 1, z - 1, searchSpace);
            
            orderedEdges[1][1] = CellGridManagerHelper.PositionToIndex(x + 1, y - 1, z, searchSpace);
            orderedEdges[2][1] = CellGridManagerHelper.PositionToIndex(x - 1, y - 1, z, searchSpace);
            orderedEdges[0][2] = CellGridManagerHelper.PositionToIndex(x, y - 1, z + 1, searchSpace);
            orderedEdges[1][2] = CellGridManagerHelper.PositionToIndex(x, y - 1, z - 1, searchSpace);
            
            orderedEdges[2][2] = CellGridManagerHelper.PositionToIndex(x + 1, y, z + 1, searchSpace);
            orderedEdges[0][3] = CellGridManagerHelper.PositionToIndex(x - 1, y, z + 1, searchSpace);
            orderedEdges[1][3] = CellGridManagerHelper.PositionToIndex(x + 1, y, z - 1, searchSpace);
            orderedEdges[2][3] = CellGridManagerHelper.PositionToIndex(x - 1, y, z - 1, searchSpace);

            for (int i = 0; i < 12; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, edgesCount.x, edgesCount.y, ref position);
                if (orderedEdges[position.x][position.y] != -1)
                {
                    CellGridManagerHelper.IndexToPosition(counter, edgesCount.x, edgesCount.y, ref pos);
                    edges[pos.x][pos.y] = orderedEdges[position.x][position.y];
                    counter++;
                }
            }
            
            return edges;
        }

        private void CumulateBlockedNodes(LayerMask layerMask, ref bool[] blockedNodes)
        {
            int index = 0;
            bool isFree = false;

            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                //check if cell is obstructed
                index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                isFree = Physics.OverlapBoxNonAlloc(allNodes[index].globalPosition, _halfCellSize, results, quaternionIdentity, layerMask, triggerInteraction) <= 0;
                blockedNodes[index] |= !isFree;
            }
        }

        private void BlockNodes(LayerMask layerMask)
        {
            int index = 0;
            bool isFree = false;

            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                //check if cell is obstructed
                index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                isFree = Physics.OverlapBoxNonAlloc(allNodes[index].globalPosition, _halfCellSize, results, quaternionIdentity, layerMask, triggerInteraction) <= 0;
                allPathCosts[index].isFree = isFree;
            }
        }
        
        private List<int> GetEdgeNodes(bool[] blockedNodes)
        {
            int index = 0;
            int2 pos = 0;
            
            List<int> edgeNodes = new List<int>();
            for (int z = 0; z < searchWidth; z++)
            for (int y = 0; y < searchHeight; y++)
            for (int x = 0; x < searchLength; x++)
            {
                index = CellGridManagerHelper.PositionToIndex(x, y, z, searchSpace);
                HybridNode tempNode = allNodes[index];

                if (blockedNodes[index])
                {
                    allPathCosts[index].isFree = false;
                    continue;
                }

                bool hasBlockedEdge = false;
                for (int i = 0; i < 12; i++)
                {
                    int3 freeSplitEdgeNeighbours = GetFreeSplitEdgeNeighbours(index, normalizedEdgeDirections[i], blockedNodes);
                    if (freeSplitEdgeNeighbours[2] == -1 &&  freeSplitEdgeNeighbours[0] != -1 && freeSplitEdgeNeighbours[1] != -1)
                    {
                        hasBlockedEdge = true;
                        break;
                    }
                }
                if (hasBlockedEdge)
                {
                    edgeNodes.Add(index);
                    continue;
                }

                for (int i = 0; i < 8; i++)
                {
                    int4 freeSplitEdgeNeighbours = GetFreeSplitVertexNeighbours(index, normalizedVertexDirections[i], blockedNodes);
                    if (freeSplitEdgeNeighbours[3] == -1 &&  freeSplitEdgeNeighbours[0] != -1 && freeSplitEdgeNeighbours[1] != -1 && freeSplitEdgeNeighbours[2] != -1)
                    {
                        edgeNodes.Add(index);
                        break;
                    }
                }
            }
            
            return edgeNodes;
        }

        //set all neighbours of dynamic nodes as dynamicNeighbour node if the node is not blocked by a static obstacle
        private void SetAllDynamicNeighboursStatus(int index)
        {
            int2 pos = 0;
            HybridNode tempNode = allNodes[index];
            
            for (int i = 0; i < tempNode.neighboursCount; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, neighbourCount, ref pos);
                allNodes[tempNode.neighbours[pos.x][pos.y]].isDynamicNeighbourNode = !blockedByStatic[tempNode.neighbours[pos.x][pos.y]];
            }

            for (int i = 0; i < 12; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, edgesCount, ref pos);
                if (tempNode.orderedEdgeNeighbours[pos.x][pos.y] != -1)
                {
                    allNodes[tempNode.orderedEdgeNeighbours[pos.x][pos.y]].isDynamicNeighbourNode = !blockedByStatic[tempNode.orderedEdgeNeighbours[pos.x][pos.y]];
                }
            }

            for (int i = 0; i < 8; i++)
            {
                CellGridManagerHelper.IndexToPosition(i, verticesCount, ref pos);
                if (tempNode.orderedVerticesNeighbours[pos.x][pos.y] != -1)
                {
                    allNodes[tempNode.orderedVerticesNeighbours[pos.x][pos.y]].isDynamicNeighbourNode = !blockedByStatic[tempNode.orderedVerticesNeighbours[pos.x][pos.y]];
                }
            }
        }

        private void UpdateNeighbours()
        {
            for (int i = 0; i < allNodes.Length; i++)
            {
                if (allNodes[i].isDynamicNode)
                {
                    UpdateDynamicNodesNeighbours(i);
                    continue;
                }
                
                if (allNodes[i].isStaticNode || allNodes[i].isDynamicNeighbourNode)
                {
                    UpdateNodesNeighbours(i);
                }
            }
        }

        private void UpdateDynamicNodesNeighbours(int index)
        {
            int2 neighbourPos, orderedNeighbourPos;
            int neighbour;
            HybridNode node = allNodes[index];
            HybridNodePathCosts nodePathCosts = allPathCosts[index];
            int count = 0;
            
            //remove blocked side neighbours
            int3x2 orderedNeighbours = -1;
            int3x2 neighbours = 0;
            double3x2 pathCosts = math.INFINITY_DBL;
            for (int i = 0; i < normalizedDirections.Length; i++)
            {
                orderedNeighbourPos = CellGridManagerHelper.IndexToPosition(i, neighbourCount);
                neighbour = node.orderedNeighbours[orderedNeighbourPos.x][orderedNeighbourPos.y];
                if (neighbour != -1 && allPathCosts[neighbour].isFree)
                {
                    neighbourPos = CellGridManagerHelper.IndexToPosition(count, neighbourCount);
                    neighbours[neighbourPos.x][neighbourPos.y] = neighbour;
                    pathCosts[neighbourPos.x][neighbourPos.y] = cellSize;
                    orderedNeighbours[orderedNeighbourPos.x][orderedNeighbourPos.y] = neighbour;
                    TryAddPredecessor(neighbour, index);
                    count++;
                }
            }
            nodePathCosts.pathCosts = pathCosts;
            node.neighboursCount = count;
            node.neighbours = neighbours;
            node.predecessors = neighbours;
            node.orderedNeighbours = orderedNeighbours;
            
            //remove blocked edges
            count = 0;
            int4x3 orderedEdgeNeighbours = -1;
            int4x3 edgeNeighbours = 0;
            double4x3 edgePathCosts = math.INFINITY_DBL;
            for (int i = 0; i < normalizedEdgeDirections.Length; i++)
            {
                int3 freeSplitEdgeNeighbours = GetFreeSplitEdgeNeighbours(index, normalizedEdgeDirections[i]);
                if (math.any(freeSplitEdgeNeighbours == -1))
                {
                    continue;
                }
                
                neighbourPos = CellGridManagerHelper.IndexToPosition(count, edgesCount);
                edgeNeighbours[neighbourPos.x][neighbourPos.y] = freeSplitEdgeNeighbours[2];
                edgePathCosts[neighbourPos.x][neighbourPos.y] = math.sqrt(2) * cellSize;
                orderedNeighbourPos = CellGridManagerHelper.IndexToPosition(i, edgesCount);
                orderedEdgeNeighbours[orderedNeighbourPos.x][orderedNeighbourPos.y] = freeSplitEdgeNeighbours[2];
                TryAddPredecessor(freeSplitEdgeNeighbours[2], index);
                count++;
            }
            nodePathCosts.edgePathCosts = edgePathCosts;
            node.edgesCount = count;
            node.edgeNeighbours = edgeNeighbours;
            node.edgePredecessors = edgeNeighbours;
            node.orderedEdgeNeighbours = orderedEdgeNeighbours;
            
            //remove blocked vertices
            count = 0;
            int4x2 orderedVerticesNeighbours = -1;
            int4x2 verticesNeighbours = 0;
            double4x2 verticesPathCosts = math.INFINITY_DBL;
            for (int i = 0; i < normalizedVertexDirections.Length; i++)
            {
                int4 freeSplitVertexNeighbours = GetFreeSplitVertexNeighbours(index, normalizedVertexDirections[i]);
                int3 freeSplitVertexEdgeNeighbours = GetFreeSplitVertexEdgeNeighbours(index, normalizedVertexDirections[i]);
                if (math.any(freeSplitVertexNeighbours == -1) || math.any(freeSplitVertexEdgeNeighbours == -1))
                {
                    continue;
                }
                
                neighbourPos = CellGridManagerHelper.IndexToPosition(count, verticesCount);
                verticesNeighbours[neighbourPos.x][neighbourPos.y] = freeSplitVertexNeighbours[3];
                verticesPathCosts[neighbourPos.x][neighbourPos.y] = math.sqrt(3) * cellSize;
                orderedNeighbourPos = CellGridManagerHelper.IndexToPosition(i, verticesCount);
                orderedVerticesNeighbours[orderedNeighbourPos.x][orderedNeighbourPos.y] = freeSplitVertexNeighbours[3];
                TryAddPredecessor(freeSplitVertexNeighbours[3], index);
                count++;
            }
            nodePathCosts.verticesPathCosts = verticesPathCosts;
            node.verticesCount = count;
            node.verticesNeighbours = verticesNeighbours;
            node.verticesPredecessors = verticesNeighbours;
            node.orderedVerticesNeighbours = orderedVerticesNeighbours;
            
            allNodes[index] = node;
            allPathCosts[index] = nodePathCosts;
        }
        
        private void UpdateNodesNeighbours(int index)
        {
            UpdateSideNeighbours(index);
            UpdateEdges(index);
            UpdateVertices(index);
        }

        private void UpdateSideNeighbours(int index)
        {
            int[] orderedNeighboursArray = new int[normalizedDirections.Length];
            for (int i = 0; i < normalizedDirections.Length; i++)
            {
                orderedNeighboursArray[i] = GetFirstUsedNode(index, normalizedDirections[i]);
            }

            int3x2 orderedNeighbours = -1;
            int3x2 neighbours = 0;
            double3x2 pathCosts = math.INFINITY_DBL;
            
            FillMatrices(orderedNeighboursArray, neighbourCount, out int count, out int[][] unorderedMatrix, out int[][] orderedMatrix);
            UpdatePathCosts(index, neighbourCount, unorderedMatrix, out double[][] pathCostsMatrix);
            for (int i = 0; i < unorderedMatrix.Length; i++)
            {
                for (int j = 0; j < unorderedMatrix[i].Length; j++)
                {
                    pathCosts[i][j] = pathCostsMatrix[i][j];
                    neighbours[i][j] = math.max(0,unorderedMatrix[i][j]);
                    orderedNeighbours[i][j] = orderedMatrix[i][j];
                }
            }

            TryAddPredecessorsRange(index, unorderedMatrix);
            allPathCosts[index].pathCosts = pathCosts;
            allNodes[index].orderedNeighbours = orderedNeighbours;
            allNodes[index].neighbours = neighbours;
            allNodes[index].neighboursCount = count;
        }

        private void UpdateEdges(int index)
        {
            int3 primaryDirection, secondaryDirection;
            int[] orderedEdgesArray = new int[normalizedEdgeDirections.Length];
            int stepCount = 0;
            
            for (int i = 0; i < normalizedEdgeDirections.Length; i++)
            {
                GetSplitDirections(normalizedEdgeDirections[i], out primaryDirection, out secondaryDirection);
                int3 splitEdgeNeighbours = GetFreeSplitEdgeNeighbours(index, normalizedEdgeDirections[i], primaryDirection, secondaryDirection);
                if (math.any(splitEdgeNeighbours == -1))
                {
                    orderedEdgesArray[i] = -1;
                    continue;
                }

                orderedEdgesArray[i] = GetFirstUsedNode(index, primaryDirection, secondaryDirection, ref stepCount);
            }

            int4x3 orderedEdges = -1;
            int4x3 edges = 0;
            double4x3 edgePathCosts = math.INFINITY_DBL;
            
            FillMatrices(orderedEdgesArray, edgesCount, out int count, out int[][] unorderedMatrix, out int[][] orderedMatrix);
            UpdatePathCosts(index, edgesCount, unorderedMatrix, out double[][] pathCostsMatrix);
            for (int i = 0; i < unorderedMatrix.Length; i++)
            {
                for (int j = 0; j < unorderedMatrix[i].Length; j++)
                {
                    edgePathCosts[i][j] = pathCostsMatrix[i][j];
                    edges[i][j] = math.max(0, unorderedMatrix[i][j]);
                    orderedEdges[i][j] = orderedMatrix[i][j];
                }
            }

            TryAddPredecessorsRange(index, unorderedMatrix);
            allPathCosts[index].edgePathCosts = edgePathCosts;
            allNodes[index].orderedEdgeNeighbours = orderedEdges;
            allNodes[index].edgeNeighbours = edges;
            allNodes[index].edgesCount = count;
        }

        private void UpdateVertices(int index)
        {
            int3 primaryDirection, secondaryDirection, tertiaryDirection;
            int[] orderedVerticesArray = new int[normalizedVertexDirections.Length];
            
            for (int i = 0; i < normalizedVertexDirections.Length; i++)
            {
                int4 splitVertexNeighbours = GetFreeSplitVertexNeighbours(index, normalizedVertexDirections[i]);
                int3 splitVertexEdgeNeighbours = GetFreeSplitVertexEdgeNeighbours(index, normalizedVertexDirections[i]);
                if (math.any(splitVertexNeighbours == -1) || math.any(splitVertexEdgeNeighbours == -1))
                {
                    orderedVerticesArray[i] = -1;
                    continue;
                }
                
                GetSplitDirections(normalizedVertexDirections[i], out primaryDirection, out secondaryDirection, out tertiaryDirection);
                orderedVerticesArray[i] = GetFirstUsedNode(index, primaryDirection, secondaryDirection, tertiaryDirection);
            }

            int4x2 orderedVertices = -1;
            int4x2 vertices = 0;
            double4x2 verticesPathCosts = math.INFINITY_DBL;

            FillMatrices(orderedVerticesArray, verticesCount, out int count, out int[][] unorderedMatrix, out int[][] orderedMatrix);
            UpdatePathCosts(index, verticesCount, unorderedMatrix, out double[][] pathCostsMatrix);
            for (int i = 0; i < unorderedMatrix.Length; i++)
            {
                for (int j = 0; j < unorderedMatrix[i].Length; j++)
                {
                    verticesPathCosts[i][j] = pathCostsMatrix[i][j];
                    vertices[i][j] = math.max(0, unorderedMatrix[i][j]);
                    orderedVertices[i][j] = orderedMatrix[i][j];
                }
            }

            TryAddPredecessorsRange(index, unorderedMatrix);
            allPathCosts[index].verticesPathCosts = verticesPathCosts;
            allNodes[index].orderedVerticesNeighbours = orderedVertices;
            allNodes[index].verticesNeighbours = vertices;
            allNodes[index].verticesCount = count;
        }

        private void FillMatrices(int[] orderedValues, int2 valueCount, out int count, out int[][] unorderedMatrix, out int[][] orderedMatrix)
        {
            unorderedMatrix = new int[valueCount.x][];
            orderedMatrix = new int[valueCount.x][];
            for (int i = 0; i < valueCount.x; i++)
            {
                unorderedMatrix[i] = new int[valueCount.y];
                orderedMatrix[i] = new int[valueCount.y];
                for (int j = 0; j < valueCount.y; j++)
                {
                    unorderedMatrix[i][j] = -1;
                }
            }

            count = 0;
            int loopCount = valueCount.x * valueCount.y;
            int2 orderedPosition, unorderedPosition;
            for (int i = 0; i < loopCount; i++)
            {
                orderedPosition = CellGridManagerHelper.IndexToPosition(i, valueCount);
                orderedMatrix[orderedPosition.x][orderedPosition.y] = orderedValues[i];
                if (orderedValues[i] != -1)
                {
                    unorderedPosition = CellGridManagerHelper.IndexToPosition(count, valueCount);
                    unorderedMatrix[unorderedPosition.x][unorderedPosition.y] = orderedValues[i];
                    count++;
                }
            }
        }

        private void UpdatePathCosts(int index, int2 valueCount, int[][] unorderedNeighbours, out double[][] pathCosts)
        {
            pathCosts = new double[valueCount.x][];
            for (int i = 0; i < valueCount.x; i++)
            {
                pathCosts[i] = new double[valueCount.y];
                for (int j = 0; j < valueCount.y; j++)
                {
                    pathCosts[i][j] = math.INFINITY_DBL;
                }
            }
            
            float3 nodePosition = allNodes[index].globalPosition;
            for (int i = 0; i < unorderedNeighbours.Length; i++)
            {
                for (int j = 0; j < unorderedNeighbours[i].Length; j++)
                {
                    if (unorderedNeighbours[i][j] != -1)
                    {
                        pathCosts[i][j] = CellGridManagerHelper.Distance(nodePosition, allNodes[unorderedNeighbours[i][j]].globalPosition);
                    }
                }
            }
        }

        private int GetFirstUsedNode(int index, int3 direction)
        {
            int stepCounter = 0;
            return GetFirstUsedNode(index, direction, ref stepCounter);
        }
        
        private int GetFirstUsedNode(int index, int3 direction, ref int stepCount)
        {
            stepCount++;
            index = (allNodes[index].localPosition + direction).PositionToIndex(searchSpace);
            if (index == -1 || !allPathCosts[index].isFree)
            {
                return -1;
            }

            if (allNodes[index].isStaticNode || allNodes[index].isDynamicNeighbourNode|| allNodes[index].isDynamicNode)
            {
                return index;
            }

            return GetFirstUsedNode(index, direction, ref stepCount);
        }

        private int GetFirstUsedNode(int index, int3 primaryDirection, int3 secondaryDirection, ref int secondaryStepCount)
        {
            secondaryStepCount++;
            index = (allNodes[index].localPosition + primaryDirection).PositionToIndex(searchSpace);
            if (index == -1 || !allPathCosts[index].isFree)
            {
                return -1;
            }
        
            int stepCount = 0;
            int usedNodeInSecondaryDirection = GetFirstUsedNode(index, secondaryDirection, ref stepCount);
            if (usedNodeInSecondaryDirection != -1)
            {
                return usedNodeInSecondaryDirection;
            }
            
            if (stepCount <= 1)
            {
                return -1;
            }
        
            return GetFirstUsedNode(index, primaryDirection, secondaryDirection, ref secondaryStepCount);
        }

        private int GetFirstUsedNode(int index, int3 primaryDirection, int3 secondaryDirection, int3 tertiaryDirection)
        {
            index = (allNodes[index].localPosition + primaryDirection).PositionToIndex(searchSpace);
            if (index == -1 || !allPathCosts[index].isFree)
            {
                return -1;
            }

            int secondaryStepCount = 0;
            int usedNodeInTertiaryDirection = GetFirstUsedNode(index, secondaryDirection, tertiaryDirection, ref secondaryStepCount);
            if (usedNodeInTertiaryDirection != -1)
            {
                return usedNodeInTertiaryDirection;
            }

            if (secondaryStepCount <= 1)
            {
                return -1;
            }
            
            return GetFirstUsedNode(index, primaryDirection, secondaryDirection, tertiaryDirection);
        }

        private void GetSplitDirections(int3 direction, out int3 primaryDirection, out int3 secondaryDirection)
        {
            primaryDirection = 0;
            secondaryDirection = 0;
            for (int i = 0; i < 3; i++)
            {
                
                int3 mask = 0;
                mask[i] = 1;
                if ((direction * mask).Equals(0))
                {
                    continue;
                }

                if (primaryDirection.Equals(0))
                {
                    primaryDirection = direction * mask;
                }
                else
                {
                    secondaryDirection = direction * mask;
                }
            }
        }

        private void GetSplitDirections(int3 direction, out int3 primaryDirection, out int3 secondaryDirection, out int3 tertiaryDirection)
        {
            primaryDirection = direction * new int3(1, 0, 0);
            secondaryDirection = direction * new int3(0, 1, 0);
            tertiaryDirection = direction * new int3(0, 0, 1);
        }

        private int3 GetFreeSplitEdgeNeighbours(int index, int3 direction, bool[] blockedNodes = null)
        {
            GetSplitDirections(direction, out int3 primaryDirection, out int3 secondaryDirection);
            return GetFreeSplitEdgeNeighbours(index, direction, primaryDirection, secondaryDirection, blockedNodes);
        }
        
        private int3 GetFreeSplitEdgeNeighbours(int index, int3 direction, int3 primaryDirection, int3 secondaryDirection, bool[] blockedNodes = null) 
        {
            int3 splitEdgeNeighbours = -1;
            int3 localPosition = allNodes[index].localPosition;
            
            splitEdgeNeighbours[0] = (localPosition + primaryDirection).PositionToIndex(searchSpace);
            splitEdgeNeighbours[1] = (localPosition + secondaryDirection).PositionToIndex(searchSpace);
            splitEdgeNeighbours[2] = (localPosition + direction).PositionToIndex(searchSpace);

            for (int i = 0; i < 3; i++)
            {
                if (splitEdgeNeighbours[i] != -1 && ((blockedNodes == null && !allPathCosts[splitEdgeNeighbours[i]].isFree) || (blockedNodes != null && blockedNodes[splitEdgeNeighbours[i]])))
                {
                    splitEdgeNeighbours[i] = -1;
                }
            }
            
            return splitEdgeNeighbours;
        }

        private int4 GetFreeSplitVertexNeighbours(int index, int3 direction, bool[] blockedNodes = null)
        {
            int4 splitVertexNeighbours = -1;
            int3 localPosition = allNodes[index].localPosition;
            
            GetSplitDirections(direction, out int3 primaryDirection, out int3 secondaryDirection, out  int3 tertiaryDirection);
            splitVertexNeighbours[0] = (localPosition + primaryDirection).PositionToIndex(searchSpace);
            splitVertexNeighbours[1] = (localPosition + secondaryDirection).PositionToIndex(searchSpace);
            splitVertexNeighbours[2] = (localPosition + tertiaryDirection).PositionToIndex(searchSpace);
            splitVertexNeighbours[3] = (localPosition + direction).PositionToIndex(searchSpace);
            
            for (int i = 0; i < 4; i++)
            {
                if (splitVertexNeighbours[i] != -1 && ((blockedNodes == null && !allPathCosts[splitVertexNeighbours[i]].isFree) || (blockedNodes != null && blockedNodes[splitVertexNeighbours[i]])))
                {
                    splitVertexNeighbours[i] = -1;
                }
            }
            
            return splitVertexNeighbours;
        }

        private int3 GetFreeSplitVertexEdgeNeighbours(int index, int3 direction)
        {
            int3 splitVertexEdgeNeighbours = -1;
            int3 localPosition = allNodes[index].localPosition;
            
            GetSplitDirections(direction, out int3 primaryDirection, out int3 secondaryDirection, out  int3 tertiaryDirection);
            splitVertexEdgeNeighbours[0] = (localPosition + primaryDirection + secondaryDirection).PositionToIndex(searchSpace);
            splitVertexEdgeNeighbours[1] = (localPosition + secondaryDirection + tertiaryDirection).PositionToIndex(searchSpace);
            splitVertexEdgeNeighbours[2] = (localPosition + primaryDirection + tertiaryDirection).PositionToIndex(searchSpace);
            
            for (int i = 0; i < 3; i++)
            {
                if (splitVertexEdgeNeighbours[i] != -1 && !allPathCosts[splitVertexEdgeNeighbours[i]].isFree)
                {
                    splitVertexEdgeNeighbours[i] = -1;
                }
            }
            
            return splitVertexEdgeNeighbours;
        }

        private void TryAddPredecessor(int index, int predecessor)
        {
            if (predecessors.TryGetValue(index, out List<int> pred))
            {
                pred.Add(predecessor);
            }
            else
            {
                List<int> addPred = new List<int>();
                addPred.Add(predecessor);
                predecessors.Add(index, addPred);
            }
        }

        private void TryAddPredecessorsRange(int index, int[][] unorderedNeighbours)
        {
            for (int i = 0; i < unorderedNeighbours.Length; i++)
            {
                for (int j = 0; j < unorderedNeighbours[i].Length; j++)
                {
                    if (unorderedNeighbours[i][j] != -1)
                    {
                        TryAddPredecessor(unorderedNeighbours[i][j], index);
                    }
                }
            }
        }

        private void SaveSearchGraph()
        {
            List<HybridNodeSaveState> safeNodes = new List<HybridNodeSaveState>();
            HybridNode node;
            for (int i = 0; i < allNodes.Length; i++)
            {
                node = allNodes[i];
                if (node.isDynamicNode || node.isStaticNode || node.isDynamicNeighbourNode)
                {
                    safeNodes.Add(new HybridNodeSaveState(node, allPathCosts[i]));
                }
            }

            string savePredecessors = Json.Serialize(predecessors);
            string saveNodes = JsonHelper.ArrayToJson(safeNodes.ToArray());
            FileHandler.self.WriteSaveFile(saveNodes+"\n"+savePredecessors);
        }

        private void LoadSearchGraph(string nodesString)
        {
            dynamicNodes = new List<int>();
            HybridNodeSaveState[] savedNodes = JsonHelper.JsonToArray<HybridNodeSaveState>(nodesString);
            globalNodeVerticesMap = new NativeMultiHashMap<int, float3>(8, Allocator.Persistent);
            usedNodes = new int[savedNodes.Length];

            for (int i = 0; i < savedNodes.Length; i++)
            {
                usedNodes[i] = savedNodes[i].index;
                allNodes[savedNodes[i].index].RecoverSaveState(savedNodes[i]);
                allPathCosts[savedNodes[i].index].RecoverSaveState(savedNodes[i]);
                
                if (savedNodes[i].isDynamicNode)
                {
                    dynamicNodes.Add(savedNodes[i].index);
                    allNodes[savedNodes[i].index].SetBasicProjections(transform);
                    
                    float3[] nodeVertices = CellGridManagerHelper.GlobalNodeVertices(allNodes[savedNodes[i].index].globalPosition, cellSize);
                    for (int j = 0; j < nodeVertices.Length; j++)
                    {
                        globalNodeVerticesMap.Add(dynamicNodes.Count-1, nodeVertices[j]);   
                    }
                }
            }

            dynamicNodesNativeArray = new NativeArray<int>(dynamicNodes.ToArray(), Allocator.Persistent);
        }

        private void InstantiateCellNodes()
        {
            for (int i = 0; i < allNodes.Length; i++)
            {
                if (allNodes[i].isDynamicNode || allNodes[i].isStaticNode || allNodes[i].isDynamicNeighbourNode)
                {
                    CellObject.CreateHybrid(cellPrefab, transform, allNodes[i].localPosition, cellSize, i);   
                }
            }
        }

        private void InitDynamicObstacles()
        {
            dynamicObstacles = FindObjectsOfType<DynamicObstacle>();
            localObstacleVertices = new NativeMultiHashMap<int, float3>(dynamicObstacles.Length, Allocator.Persistent); 
            localObstacleNormals = new NativeMultiHashMap<int, float3>(dynamicObstacles.Length, Allocator.Persistent);
            localObstacleEdges = new NativeMultiHashMap<int, float3>(dynamicObstacles.Length, Allocator.Persistent);
            for (int i = 0; i < dynamicObstacles.Length; i++)
            {
                float3[] localVertices = dynamicObstacles[i].LocalVertices;
                float3[] localEdges = dynamicObstacles[i].LocalEdges;
                float3[] localNormals = dynamicObstacles[i].LocalNormals;
                    
                for (int j = 0; j < localVertices.Length; j++)
                {
                    localObstacleVertices.Add(i, localVertices[j]);   
                }

                for (int j = 0; j < localEdges.Length; j++)
                {
                    localObstacleEdges.Add(i, localEdges[j]);
                }

                for (int j = 0; j < localNormals.Length; j++)
                {
                    localObstacleNormals.Add(i, localEdges[j]);
                }
            }
            
        }

        private void InitUpdateJob()
        {
            InitDynamicObstacles();
            globalObstacleVertices = new NativeList<float3>(Allocator.Persistent);
            globalObstacleNormals = new NativeList<float3>(Allocator.Persistent);
            globalObstacleEdges = new NativeList<float3>(Allocator.Persistent);
            changedNodesList = new NativeList<int>(Allocator.Persistent);
            allAxis = new NativeList<float3>(Allocator.Persistent);
            basicAxis = new NativeArray<float3>(3, Allocator.Persistent);
            allHybridNodes = new NativeArray<HybridNode>(allNodes, Allocator.Persistent);
            nodePathCosts = new NativeArray<HybridNodePathCosts>(allPathCosts, Allocator.Persistent);
            basicAxis[0] = transform.right;
            basicAxis[1] = transform.up;
            basicAxis[2] = transform.forward;
            
            job = new UpdateOverlapJob();
            job.dynamicNodes = dynamicNodesNativeArray;
            
            job.globalBasicAxis = basicAxis;
            job.localObstacleVertices = localObstacleVertices;
            job.localObstacleEdges = localObstacleEdges;
            job.localObstacleNormals = localObstacleNormals;
            job.globalNodeVerticesMap = globalNodeVerticesMap;
            job.allNodes = allHybridNodes;


            updatePathCostsJob = new UpdatePathCostsJob(cellSize);
            updatePathCostsJob.allNodes = allHybridNodes;
            updatePathCostsJob.usedNodesPathCosts = nodePathCosts;
            updatePathCostsJob.changedNodes = changedNodesList;
            updatePathCostsJob.dynamicNodes = dynamicNodesNativeArray;
            updatePathCostsJob.splitEdgeOrderedNeighbours = new NativeArray<int2x2>(splitEdgeOrderedSideNeighbours, Allocator.Persistent);
            updatePathCostsJob.splitEdgeNormalizedDirectionIndices = new NativeArray<int2>(CellGridManagerHelper.GetSplitEdgeNormalizedDirectionIndices(), Allocator.Persistent);
            
            updatePathCostsJob.splitVertexOrderedSideNeighbours = new NativeArray<int2x3>(splitVertexOrderedSideNeighbours, Allocator.Persistent);
            updatePathCostsJob.splitVertexOrderedEdgeNeighbours = new NativeArray<int2x3>(splitVertexOrderedEdgeNeighbours, Allocator.Persistent);
            updatePathCostsJob.splitVertexNormalizedDirectionIndices = new NativeArray<int3>(CellGridManagerHelper.GetSplitVertexNormalizedDirectionIndices(), Allocator.Persistent);
            updatePathCostsJob.splitVertexNormalizedEdgeDirectionIndices = new NativeArray<int3>(CellGridManagerHelper.GetSplitVertexNormalizedEdgeDirectionIndices(), Allocator.Persistent);

            updatePathCostsJob.normalizedDirections = new NativeArray<int3>(normalizedDirections, Allocator.Persistent);
            updatePathCostsJob.normalizedEdgeDirections = new NativeArray<int3>(normalizedEdgeDirections, Allocator.Persistent);
            updatePathCostsJob.normalizedVertexDirections = new NativeArray<int3>(normalizedVertexDirections, Allocator.Persistent);

            updatePathCostsJob.searchSpace = searchSpace;
        }

        protected override void UpdateSearch()
        {
            if (IsBakingSearchGraph || !Application.isPlaying)
            {
                return;
            }
            
            obstaclesLocalToWorldMatrix = new NativeArray<float4x4>(dynamicObstacles.Length, Allocator.Persistent);
            for (int i = 0; i < dynamicObstacles.Length; i++)
            {
                obstaclesLocalToWorldMatrix[i] =  dynamicObstacles[i].LocalToWorld;
            }
            job.dynamicObstaclesLocalToWorld = obstaclesLocalToWorldMatrix;

            nodeIsBlocked = new NativeArray<bool>(dynamicNodes.Count, Allocator.Persistent);
            job.nodeIsBlocked = nodeIsBlocked;
            JobHandle jobHandle = job.Schedule(obstaclesLocalToWorldMatrix.Length, 1);
            
            updatePathCostsJob.nodeIsBlocked = nodeIsBlocked;
            updatePathCostsJobHandle = updatePathCostsJob.Schedule(jobHandle);
            JobHandle.ScheduleBatchedJobs();
        }

        private void CompleteSearchGraphUpdate()
        {
            if (!Application.isPlaying)
            {
                return;
            }
            updatePathCostsJobHandle.Complete();
            
            changedNodesSet.UnionWith(changedNodesList.ToArray());

            obstaclesLocalToWorldMatrix.Dispose();
            nodeIsBlocked.Dispose();
        }

        private void OnDisable()
        {
            basicAxis.Dispose();
            dynamicNodesNativeArray.Dispose();
            localObstacleVertices.Dispose();
            localObstacleEdges.Dispose();
            localObstacleNormals.Dispose();
            globalObstacleVertices.Dispose();
            globalObstacleNormals.Dispose();
            globalObstacleEdges.Dispose();
            globalNodeVerticesMap.Dispose();
            allAxis.Dispose();
            changedNodesList.Dispose();
            allHybridNodes.Dispose();
            nodePathCosts.Dispose();
            updatePathCostsJob.splitEdgeOrderedNeighbours.Dispose();
            updatePathCostsJob.splitVertexOrderedEdgeNeighbours.Dispose();
            updatePathCostsJob.splitVertexOrderedSideNeighbours.Dispose();
            updatePathCostsJob.splitEdgeNormalizedDirectionIndices.Dispose();
            updatePathCostsJob.splitVertexNormalizedDirectionIndices.Dispose();
            updatePathCostsJob.splitVertexNormalizedEdgeDirectionIndices.Dispose();
            updatePathCostsJob.normalizedDirections.Dispose();
            updatePathCostsJob.normalizedEdgeDirections.Dispose();
            updatePathCostsJob.normalizedVertexDirections.Dispose();
            PlayerLoop.SetPlayerLoop(PlayerLoop.GetDefaultPlayerLoop());
        }
    }
}