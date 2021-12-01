using Pathfinding.Nodes;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Util;

namespace Pathfinding.SearchGraph
{
    [BurstCompile(FloatPrecision.Medium, FloatMode.Fast, CompileSynchronously = true)]
    public struct UpdateOverlapJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> dynamicNodes;
        [ReadOnly] public NativeMultiHashMap<int, float3> globalNodeVerticesMap;
        [NativeDisableContainerSafetyRestriction] public NativeArray<bool> nodeIsBlocked;
        public NativeArray<float4x4> dynamicObstaclesLocalToWorld;
        [ReadOnly] public NativeArray<float3> globalBasicAxis;
        [ReadOnly] public NativeMultiHashMap<int, float3> localObstacleVertices, localObstacleNormals, localObstacleEdges;
        [ReadOnly] public NativeArray<HybridNode> allNodes;

        public void Execute(int jobIndex)
        {
            //fill lists with global vertices, normals, and edges based on the local variables and the localToGlobal matrices of the dynamic obstacles 
            NativeList<float3> globalObstacleVertices = new NativeList<float3>(Allocator.Temp);
            NativeList<float3> globalObstacleNormals = new NativeList<float3>(Allocator.Temp);
            NativeList<float3> globalObstacleEdges = new NativeList<float3>(Allocator.Temp);
            LocalPointMapToGlobalList(jobIndex, localObstacleVertices, ref globalObstacleVertices);
            LocalVectorMapToGlobalList(jobIndex, localObstacleNormals, ref globalObstacleNormals);
            LocalVectorMapToGlobalList(jobIndex, localObstacleEdges, ref globalObstacleEdges);
            
            //because all nodes have the same normals and edges all possible separating axis can be collected here (once the obstacle is known)
            NativeList<float3> allAxis = new NativeList<float3>(Allocator.Temp);
            SeparatingAxisTheorem.CollectAxis(globalBasicAxis, globalObstacleNormals, globalObstacleEdges, ref allAxis);
            
            //as all axis are already known, the projections of the obstacle onto all these axis can already been done here
            NativeList<Projection> obstacleProjections = new NativeList<Projection>(Allocator.Temp);
            SeparatingAxisTheorem.ProjectOntoAxis(allAxis, globalObstacleVertices, ref obstacleProjections);
            
            for (int i = 0; i < dynamicNodes.Length; i++)
            {   
                //check overlap of the object with all dynamic nodes
                if (SeparatingAxisTheorem.IsOverlapping(obstacleProjections, allAxis, allNodes[dynamicNodes[i]], i, globalNodeVerticesMap)) 
                {
                    nodeIsBlocked[i] = true;
                }
            }
        }

        private void LocalPointMapToGlobalList(int index, NativeMultiHashMap<int, float3> map, ref NativeList<float3> list)
        {
            list.Clear();
            NativeMultiHashMap<int,float3>.Enumerator enumerator = map.GetValuesForKey(index);
            while (enumerator.MoveNext())
            {
                float4 local = new float4(enumerator.Current, 1);
                float3 global = math.mul(dynamicObstaclesLocalToWorld[index], local).xyz;
                list.Add(global);
            }
            enumerator.Dispose();
        }

        private void LocalVectorMapToGlobalList(int index, NativeMultiHashMap<int, float3> map, ref NativeList<float3> list)
        {
            list.Clear();
            NativeMultiHashMap<int,float3>.Enumerator enumerator = map.GetValuesForKey(index);
            while (enumerator.MoveNext())
            {
                float4 local = new float4(enumerator.Current, 0);
                float3 global = math.mul(dynamicObstaclesLocalToWorld[index], local).xyz;
                global = math.normalize(global);
                if (SeparatingAxisTheorem.AxisApproximatelyParallel(global, globalBasicAxis[0])
                    || SeparatingAxisTheorem.AxisApproximatelyParallel(global, globalBasicAxis[1])
                    || SeparatingAxisTheorem.AxisApproximatelyParallel(global, globalBasicAxis[2]))
                {
                    continue;
                }
                list.Add(global);
            }
            enumerator.Dispose();
        }

    }
}
