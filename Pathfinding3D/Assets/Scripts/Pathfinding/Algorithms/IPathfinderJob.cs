
using Unity.Mathematics;

namespace Pathfinding.Algorithms
{
    public interface IPathfinderJob
    {
        public float3 aiPosition {get; set;}
        public float3 lookaheadPosition {get; set;}
        public float3 goalPosition {get; set;}
        public float3 searchGraphOrigin {get; set;}
        public float cellSize {get; set;}
        public int3 searchSpace {get; set;}
        public PathfinderSetup pathfinderSetup {get; set;}
        public int2 neighboursCountSize {get; set;}
        public int neighboursCount {get; set;}
        
    }
}
