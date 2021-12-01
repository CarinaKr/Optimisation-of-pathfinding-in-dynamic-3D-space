using System;
using Pathfinding.Algorithms;
using UnityEngine;

namespace Pathfinding.SearchGraph
{
    public class PathfindingManager : MonoBehaviour
    {
        private enum PathfindingMethodCellGrid
        {
            A_STAR,
            THETA_STAR,
            MT_D_STAR_LITE,
            JPS_A_STAR,
        }
        
        private enum PathfindingMethodHybrid
        {
            A_STAR,
            THETA_STAR,
            MT_D_STAR_LITE,
            AA_MT_D_STAR_LITE
        }

        [SerializeField] private PathfindingMethodCellGrid pathfindingMethodCellGrid;
        [SerializeField] private PathfindingMethodHybrid pathfindingMethodHybrid;
        [SerializeField] private PathfinderSetup pathfinderSetup;

        
        private Pathfinder pathfinder;

        private void Awake()
        {
            Application.targetFrameRate = 300;
        }

        private void Start()
        {
            SearchGraphSelector.SearchGraphType searchGraphType = SearchGraphSelector.Instance.searchGraphType;
            if (searchGraphType == SearchGraphSelector.SearchGraphType.CELL_GRID)
            {
                SetCellGridPathfinder();
            }
            else
            {
                SetHybridPathfinder();
            }
            
            if (pathfinder != null)
            {
                pathfinder.PathfinderSetup = pathfinderSetup;
                pathfinder.gameObject.SetActive(true);
            }
        }

        private void SetCellGridPathfinder()
        {
            switch (pathfindingMethodCellGrid)
            {
                case PathfindingMethodCellGrid.A_STAR:
                    pathfinder = GetComponentInChildren<A_Star>(true);
                    break;
                case PathfindingMethodCellGrid.THETA_STAR:
                    pathfinder = GetComponentInChildren<Theta_Star>(true);   
                    break;
                case PathfindingMethodCellGrid.MT_D_STAR_LITE:
                    pathfinder = GetComponentInChildren<ReverseMTD_StarLite>(true);
                    break;
                case PathfindingMethodCellGrid.JPS_A_STAR:
                    pathfinder = GetComponentInChildren<A_StarJumpPoint>(true);
                    break;
            }
        }
        
        private void SetHybridPathfinder()
        {
            switch (pathfindingMethodHybrid)
            {
                case PathfindingMethodHybrid.A_STAR:
                    pathfinder = GetComponentInChildren<A_StarHybrid>(true);
                    break;
                case PathfindingMethodHybrid.THETA_STAR:
                    pathfinder = GetComponentInChildren<Theta_StarHybrid>(true);
                    break;
                case PathfindingMethodHybrid.MT_D_STAR_LITE:
                    pathfinder = GetComponentInChildren<MTD_StarLiteHybrid>(true);
                    break;
                case PathfindingMethodHybrid.AA_MT_D_STAR_LITE:
                    pathfinder = GetComponentInChildren<AA_MTD_StarLiteHybrid>(true);
                    break;
            }
        }
    }
}