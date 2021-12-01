using System.Collections;
using System.Collections.Generic;
using Obstacles;
using Pathfinding.Nodes;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions.Must;
using UnityEngine.Profiling;

namespace Util
{
    public static class SeparatingAxisTheorem
    {
        // public static bool IsOverlapping(NativeArray<Projection> obstacleProjections, NativeList<float3> allAxis, NativeArray<float3> globalBasicAxis, HybridNode node, float halfSize, int verticesMapNodeIndex, NativeMultiHashMap<int, float3> verticesMap)
        public static bool IsOverlapping(NativeList<Projection> obstacleProjections, NativeList<float3> allAxis, HybridNode node, int verticesMapNodeIndex, NativeMultiHashMap<int, float3> nodesVerticesMap)
        {
            //the first three axis in allAxis are always the basic axis, which are the normals and edges of the nodes. For these axis each hybridNode already has projections stored
            // basicProjections[min/max][x/y/z]
            for (int i = 0; i < 3; i++)
            {
                if (ProjectionsNotOverlapping(node.basicProjections[0][i], node.basicProjections[1][i], obstacleProjections[i].min, obstacleProjections[i].max))
                {
                    return false;
                }
            }
            
            //for all other axis project the node onto the axis and check if it overlaps with the pre-computed obstacle projection
            for (int i = 3; i < allAxis.Length; i++)
            {
                Project(allAxis[i], verticesMapNodeIndex, nodesVerticesMap, out float nodeProjectionMin, out float nodeProjectionMax);
                if (ProjectionsNotOverlapping(nodeProjectionMin, nodeProjectionMax, obstacleProjections[i].min, obstacleProjections[i].max))
                {
                    return false;
                }
            }
            
            return true;
        }
        
        private static bool ProjectionsNotOverlapping(float minA, float maxA, float minB, float maxB)
        {
            return (minA < minB && maxA < minB) || (minB < minA && maxB < minA);
        }

        public static void ProjectOntoAxis(NativeList<float3> allAxis, NativeList<float3> vertices, ref NativeList<Projection> projections)
        {
            for (int i = 0; i < allAxis.Length; i++)
            {
                Project(allAxis[i], vertices, out float projectionMin, out float projectionMax);
                projections.Add(new Projection {min = projectionMin, max = projectionMax});
            }
        }
        

        private static void Project(float3 axis, NativeList<float3> vertices, out float projectionMin, out float projectionMax)
        {
            projectionMin = float.MaxValue;
            projectionMax = float.MinValue;
            for (int j = 0; j < vertices.Length; j++)
            {
                float projection = math.dot(axis, vertices[j]);
                projectionMin = math.min(projectionMin, projection);
                projectionMax = math.max(projectionMax, projection);
            }
        }
        
        public static void Project(float3 axis, float3[] vertices, out float projectionMin, out float projectionMax)
        {
            projectionMin = float.MaxValue;
            projectionMax = float.MinValue;
            for (int j = 0; j < vertices.Length; j++)
            {
                float projection = math.dot(axis, vertices[j]);
                projectionMin = math.min(projectionMin, projection);
                projectionMax = math.max(projectionMax, projection);
            }
        }
        
        private static void Project(float3 axis, int verticesMapIndex, NativeMultiHashMap<int, float3> verticesMap, out float projectionMin, out float projectionMax)
        {
            projectionMin = float.MaxValue;
            projectionMax = float.MinValue;

            NativeMultiHashMap<int,float3>.Enumerator enumerator = verticesMap.GetValuesForKey(verticesMapIndex);
            while (enumerator.MoveNext())
            {
                float projection = math.dot(axis, enumerator.Current);
                projectionMin = math.min(projectionMin, projection);
                projectionMax = math.max(projectionMax, projection);
            }
            enumerator.Dispose();
        }

        public static void CollectAxis(NativeArray<float3> globalBasicAxis, NativeList<float3> globalObstacleNormals, NativeList<float3> globalObstacleEdges, ref NativeList<float3> allAxis)
        {
            //all axis = normals object 1 + normals object 2 + all combinations from edges of obj 1 and 2
            allAxis.Clear();
            allAxis.AddRange(globalBasicAxis);
            allAxis.AddRange(globalObstacleNormals);

            for (int i = 0; i < globalObstacleEdges.Length; i++)
            {
                for (int j = 0; j < globalBasicAxis.Length; j++)
                {
                    float3 axis = math.cross(globalBasicAxis[j], globalObstacleEdges[i]);
                    axis = math.normalize(axis);
                    allAxis.Add(axis);
                }
            }
        }
        
        public static bool AxisApproximatelyParallel(float3 axisA, float3 axisB)
        {
            if (math.all(math.abs(axisA - axisB) < 0.001f))
            {
                return true;
            }

            if (math.all((math.abs(axisA - math.reflect(axisB, axisB))) < 0.001f))
            {
                return true;
            }
            
            return false;
        }
    }

    public struct Projection
    {
        public float min, max;
    }
}
