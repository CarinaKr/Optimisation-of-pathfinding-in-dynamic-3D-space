using System;
using Pathfinding.SearchGraph;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.Nodes
{
    public class CellObject : MonoBehaviour
    {
        private int cellIndex;
        private Color gizmoColor;
        private Color transparent = new Color(0, 0, 0, 0);
        
        public int CellIndex => cellIndex;


        private void OnDrawGizmosSelected()
        {
            HybridManager hybridManager = SearchGraphSelector.Instance?.SearchGraphManager as HybridManager;
            if (hybridManager == null || hybridManager.AllNodes.Length == 0)
            {
                return;
            }
            
            HybridNode node = hybridManager.AllNodes[cellIndex];
            HybridNodePathCosts nodePathCosts = hybridManager.AllNodesPathCosts[cellIndex];

            if (hybridManager.VisualizeMode == VisualizeMode.Nodes)
            {
                VisualizeTagAndNeighbours(node, hybridManager);
            }
            else if (hybridManager.VisualizeMode == VisualizeMode.Blocked)
            {
                VisualizedBlocked(node, nodePathCosts, hybridManager);
            }
        }

        private void VisualizedBlocked(HybridNode node, HybridNodePathCosts nodePathCosts, HybridManager hybridManager)
        {
            Gizmos.color = nodePathCosts.isFree ? gizmoColor : Color.red;
            Gizmos.DrawWireCube(transform.position, new Vector3(1, 1, 1));
        }

        private void VisualizeTagAndNeighbours(HybridNode node, HybridManager hybridManager)
        {
            bool visualizeStatic = node.isStaticNode && (hybridManager.VisualizeNodes & VisualizeNodes.Static) != 0;
            bool visualizeDynamicNeighbour = node.isDynamicNeighbourNode && (hybridManager.VisualizeNodes & VisualizeNodes.DynamicNeighbours) != 0;
            bool visualizeDynamic = node.isDynamicNode && (hybridManager.VisualizeNodes & VisualizeNodes.Dynamic) != 0;
            gizmoColor = transparent;
            gizmoColor = visualizeStatic ? Color.yellow : gizmoColor;
            gizmoColor = visualizeDynamicNeighbour ? Color.cyan : gizmoColor;
            Gizmos.color = visualizeDynamic ? Color.magenta : gizmoColor;
            Gizmos.DrawWireCube(transform.position, new Vector3(1, 1, 1));

            if (!(visualizeStatic || visualizeDynamicNeighbour || visualizeDynamic))
            {
                return;
            }
            
            float3 start = node.globalPosition;
            if((hybridManager.VisualizeNeighbours & VisualizeNeighbours.Sides) != 0)
            {
                for (int i = 0; i < node.neighboursCount; i++)
                {
                    int2 pos = CellGridManagerHelper.IndexToPosition(i, hybridManager.neighbourCount);
                    float3 end = hybridManager.AllNodes[node.neighbours[pos.x][pos.y]].globalPosition;
                    Debug.DrawLine(start, end, Color.gray);
                }
            }

            if((hybridManager.VisualizeNeighbours & VisualizeNeighbours.Edges) != 0)
            {
                for (int i = 0; i < node.edgesCount; i++)
                {
                    int2 pos = CellGridManagerHelper.IndexToPosition(i, hybridManager.edgesCount);
                    float3 end = hybridManager.AllNodes[node.edgeNeighbours[pos.x][pos.y]].globalPosition;
                    Debug.DrawLine(start, end, Color.green);
                }
            }

            if((hybridManager.VisualizeNeighbours & VisualizeNeighbours.Vertices) != 0)
            {
                for (int i = 0; i < node.verticesCount; i++)
                {
                    int2 pos = CellGridManagerHelper.IndexToPosition(i, hybridManager.verticesCount);
                    float3 end = hybridManager.AllNodes[node.verticesNeighbours[pos.x][pos.y]].globalPosition;
                    Debug.DrawLine(start, end, Color.white);
                }
            }
        }

        public static CellNode Create(GameObject prefab, Transform parent, int3 position, float size, int index)
        {
            var newObject = Instantiate(prefab, parent);
            newObject.transform.localPosition = (float3)position * size;
            var cell = new CellNode(position, newObject.transform.position, index);
            var cellObject = newObject.GetComponent<CellObject>();
            cellObject.cellIndex = index;

            return cell;
        }

        public static void CreateHybrid(GameObject prefab, Transform parent, int3 position, float size, int index)
        {
            var newObject = Instantiate(prefab, parent);
            newObject.transform.localPosition = (float3)position * size;
            var cellObject = newObject.GetComponent<CellObject>();
            cellObject.cellIndex = index;
        }
    }

    [System.Flags]
    public enum VisualizeNodes
    {
        None = 0,
        Static = 1<<0,
        Dynamic = 1<<1,
        DynamicNeighbours = 1<<2,
        All = ~0,
    }
    
    [System.Flags]
    public enum VisualizeNeighbours
    {
        None = 0,
        Sides = 1<<0,
        Edges = 1<<1,
        Vertices = 1<<2,
        All = ~0,
    }
    
    public enum VisualizeMode
    {
        None,
        Nodes,
        Blocked,
    }
}