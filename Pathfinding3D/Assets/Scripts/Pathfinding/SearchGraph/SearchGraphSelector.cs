using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding.SearchGraph
{
    public class SearchGraphSelector : MonoBehaviour
    {
        public enum SearchGraphType
        {
            CELL_GRID,
            HYBRID,
        }

        public static SearchGraphSelector Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindObjectOfType<SearchGraphSelector>();;
                }

                return _instance;
            }
            
        }

        public SearchGraphType searchGraphType;
        public Transform start, goal;
        
        public float3 StartPosition => start.position;
        public float3 GoalPosition => goal.position;

        public SearchGraphManager SearchGraphManager { private set; get; }

        private static SearchGraphSelector _instance;

        private void Awake()
        {
            if (_instance == null)
            {
                _instance = this;
            }

            if (_instance != this)
            {
                Destroy(gameObject);
            }
            
            switch (searchGraphType)
            {
                case SearchGraphType.CELL_GRID:
                    SearchGraphManager = GetComponentInChildren<CellGridManager>(true);
                    SearchGraphManager.gameObject.SetActive(true);
                    break;
                case SearchGraphType.HYBRID:
                    SearchGraphManager = GetComponentInChildren<HybridManager>(true);
                    SearchGraphManager.gameObject.SetActive(true);
                    break;
            }
        }
    }
}