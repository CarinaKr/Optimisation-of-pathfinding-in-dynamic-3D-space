using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace Obstacles
{
    [RequireComponent(typeof(StaticMovement))]
    public class DynamicObstacle:MonoBehaviour
    {
        [SerializeField] private Collider hitCollider;

        private float3[] vertices, edges, normals;
        private Transform usedTransform;
        private MeshRenderer meshRenderer;

        public float3[] LocalVertices => vertices;
        public float3[] LocalEdges => edges;
        public float3[] LocalNormals => normals;
        public Transform UsedTransform => usedTransform;
        public Matrix4x4 LocalToWorld => (hitCollider!=null && hitCollider.enabled) || (hitCollider == null && meshRenderer.enabled) ? usedTransform.localToWorldMatrix : Matrix4x4.zero;
        
        private void Awake()
        {
            Mesh mesh = null;
            
            if (hitCollider != null)
            {
                usedTransform = hitCollider.transform;
                BoxCollider boxCollider = hitCollider as BoxCollider;
                if (boxCollider != null)
                {
                    FillVerticesBasedOnBoxCollider(boxCollider);
                    normals = new float3[3];
                    normals[0] = new float3(1, 0, 0);
                    normals[1] = new float3(0, 1, 0);
                    normals[2] = new float3(0, 0, 1);
                    edges = normals;
                    return;
                }
                
                MeshCollider meshCollider = hitCollider as MeshCollider;
                if (meshCollider != null)
                {
                    mesh = meshCollider.sharedMesh;
                }
            }
            else
            {
                usedTransform = transform;
                meshRenderer = GetComponent<MeshRenderer>();
                mesh = GetComponent<MeshFilter>().sharedMesh;
            }

            Vector3[] verts = mesh.vertices;
            vertices = RemoveDoubles(mesh.vertices);
            normals = RemoveDoubles(mesh.normals, true);
            edges = RemoveDoubles(GetEdges(mesh, verts), true);
        }

        private void FillVerticesBasedOnBoxCollider(BoxCollider boxCollider)
        {
            vertices = new float3[8];
            Vector3 sizeHalf = boxCollider.size / 2;
            Vector3 center = boxCollider.center;
                    
            vertices[0] = center + sizeHalf;
            vertices[1] = center - sizeHalf;

            vertices[2] = center + new Vector3(-sizeHalf.x, sizeHalf.y, sizeHalf.z);
            vertices[3] = center + new Vector3(-sizeHalf.x, sizeHalf.y, -sizeHalf.z);
            vertices[4] = center + new Vector3(sizeHalf.x, sizeHalf.y, -sizeHalf.z);
            
            vertices[5] = center + new Vector3(-sizeHalf.x, -sizeHalf.y, sizeHalf.z);
            vertices[6] = center + new Vector3(sizeHalf.x, -sizeHalf.y, sizeHalf.z);
            vertices[7] = center + new Vector3(sizeHalf.x, -sizeHalf.y, -sizeHalf.z);
        }

        private Vector3[] GetEdges(Mesh mesh, Vector3[] verts)
        {
            //get all edges based on triangles of the mesh
            int[] triangles = mesh.triangles;
            Dictionary<Vector3, int> triangleEdges = new Dictionary<Vector3, int>();
            List<Edge> edges = new List<Edge>();
            for (int t = 0; t < triangles.Length; t+=3)
            {
                Vector3 direction = math.normalize(verts[triangles[t + 1]] - verts[triangles[t]]);
                edges.Add(new Edge(direction, transform.TransformPoint(verts[triangles[t + 1]]), transform.TransformPoint(verts[triangles[t]])));
                if (!triangleEdges.ContainsKey(direction))
                {
                    triangleEdges.Add(direction, 0);
                }
                
                direction = verts[triangles[t + 2]] - verts[triangles[t + 1]];
                edges.Add(new Edge(direction, transform.TransformPoint(verts[triangles[t + 1]]), transform.TransformPoint(verts[triangles[t+2]])));
                if (!triangleEdges.ContainsKey(direction))
                {
                    triangleEdges.Add(direction, 0);
                }
                
                direction = verts[triangles[t]] - verts[triangles[t + 2]];
                edges.Add(new Edge(direction, transform.TransformPoint(verts[triangles[t + 2]]), transform.TransformPoint(verts[triangles[t]])));
                if (!triangleEdges.ContainsKey(direction))
                {
                    triangleEdges.Add(direction, 0);
                }
            }

            Vector3[] edgeKeys = triangleEdges.Keys.ToArray();
            for (int i = 0; i < edgeKeys.Length; i++)
            {
                float3 edge = edgeKeys[i];
                foreach (float3 normal in normals)
                {
                    if (math.abs(math.dot(normal, edge)) < 0.0001f)
                    {
                        triangleEdges[edge]++;
                    }
                }

                //remove edges that are orthogonal to less than two normals (these are edges across faces of the mesh)
                if (triangleEdges[edge] < 2)
                {
                    triangleEdges.Remove(edge);
                }
            }

            return triangleEdges.Keys.ToArray();
        }

        private float3[] RemoveDoubles(Vector3[] vectors, bool includeInverted = false)
        {
            HashSet<float3> hashSet = new HashSet<float3>();
            foreach (Vector3 vector in vectors)
            {
                if(includeInverted)
                {
                    Vector3 inverse = Vector3.Reflect(vector, vector);
                    if (!hashSet.Contains(inverse))
                    {
                        hashSet.Add(vector);
                    }
                }
                else
                {
                    hashSet.Add(vector);
                }
            }

            return hashSet.ToArray();
        }

        public float3[] GetGlobalVertexPositions()
        {
            float3[] globalVertices = new float3[vertices.Length];
            for (int i = 0; i < globalVertices.Length; i++)
            {
                float4 local = new float4(vertices[i], 1);
                globalVertices[i] = math.mul(LocalToWorld, local).xyz;
            }

            return globalVertices;
        }

        public float3[] GetGlobalEdges()
        {
            float3[] globalEdges = new float3[edges.Length];
            for (int i = 0; i < globalEdges.Length; i++)
            {
                globalEdges[i] = usedTransform.TransformVector(edges[i]);
            }

            return globalEdges;
        }
        
        public float3[] GetGlobalNormals()
        {
            float3[] globalNormals = new float3[normals.Length];
            for (int i = 0; i < globalNormals.Length; i++)
            {
                globalNormals[i] = usedTransform.TransformVector(normals[i]);
            }

            return globalNormals;
        }

        private void OnDrawGizmosSelected()
        {
            if (vertices == null)
            {
                return;
            }

            float3[] globalVertices = GetGlobalVertexPositions();
            Gizmos.color = Color.yellow;
            for (int i = 0; i < globalVertices.Length; i++)
            {
                Gizmos.DrawSphere(globalVertices[i], 0.5f);    
            }
        }
    }

    public struct Edge
    {
        public Vector3 id;
        public float3 start, end;

        public Edge(Vector3 id, float3 start, float3 end)
        {
            this.id = id;
            this.start = start;
            this.end = end;
        }
    }
}
