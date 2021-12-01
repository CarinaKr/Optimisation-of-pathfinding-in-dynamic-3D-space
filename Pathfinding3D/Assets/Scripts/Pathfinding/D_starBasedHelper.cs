using System.Collections.Generic;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Priority_Queue;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding
{
    public static class D_starBasedHelper
    {
        public static void AddToFastOpen(Node child, FastPriorityQueue<Node> fastOpenListPrim, float km)
        {
            double[] newKeyValue = CalculateKeyValue(child, km);
            if (newKeyValue[0] >= float.MaxValue)
            {
                return;
            }

            child.priorityKey = newKeyValue;
            if (!fastOpenListPrim.Contains(child))
            {
                fastOpenListPrim.Enqueue(child, (float) newKeyValue[0]);
            }
            else
            {
                fastOpenListPrim.UpdatePriority(child, (float) newKeyValue[0]);
            }
        }
        
        private static double[] CalculateKeyValue(Node child, float km)
        {
            double[] key = new double[2];
            if (child.gValue > child.rhsValue)
            {
                key[0] = child.rhsValue + child.heuristic + km;
                key[1] = child.rhsValue;
            }
            else
            {
                key[0] = child.gValue + child.plainHeuristic + km;
                key[1] = child.gValue;
            }

            return key;
        }
        
        public static Node GetNextNodeFast(FastPriorityQueue<Node> fastOpenListPrim)
        {
            if (fastOpenListPrim.Count == 0)
            {
                Debug.Log("empty open list");
                Debug.Log("no shortest path found");
            }

            if (fastOpenListPrim.Count == 0)
            {
                return null;
            }

            var samePriority = GetSamePrimKeys(fastOpenListPrim); //create List of all Nodes with same priority (check prim key)

            Node nextNode = samePriority[0];
            double minSecKey = nextNode.priorityKey[1];
            double currentSecKey = minSecKey;
            for (int i = 1; i < samePriority.Count; i++)
            {
                currentSecKey = samePriority[i].priorityKey[1];
                if (currentSecKey < minSecKey)
                {
                    minSecKey = currentSecKey;
                    nextNode = samePriority[i];
                }
            }

            return nextNode;
        }
        
        private static List<Node> GetSamePrimKeys(FastPriorityQueue<Node> fastOpenListPrim)
        {
            var samePrim = new List<Node>();
            float firstPriority = fastOpenListPrim.First.Priority;
            while (fastOpenListPrim.Count > 0)
            {
                if (fastOpenListPrim.First.Priority <= firstPriority)
                {
                    samePrim.Add(fastOpenListPrim.Dequeue());
                }
                else
                {
                    break;
                }
            }
            
            for (int i = 0; i < samePrim.Count; i++)
            {
                fastOpenListPrim.Enqueue(samePrim[i], samePrim[i].Priority);
            }

            if (fastOpenListPrim.Count == 0)
            {
                Debug.Log("empty open list");
            }

            return samePrim;
        }
        
        public static bool IsLowerKey<T>(int keyNodeA, double2 keyB, NativeArray<T> nodes) where T : struct, INode
        {
            bool isLower = false;
            double2 keyA = new double2();
            if (keyNodeA != -1)
            {
                keyA = nodes[keyNodeA].priorityKey;
            }
            else
            {
                return isLower;
            }

            if (keyA[0] < keyB[0])
            {
                isLower = true;
            }
            else if (keyA[0] == keyB[0])
            {
                if (keyA[1] < keyB[1])
                {
                    isLower = true;
                }
            }

            return isLower;
        }

        public static double CalculateRHS(int nodeIndexInPredecessor, CellNode predecessor, int2 neighboursCount)
        {
            int2 pos = new int2();
            CellGridManagerHelper.IndexToPosition(nodeIndexInPredecessor, neighboursCount, ref pos);
            return predecessor.pathCosts[pos.x][pos.y] + predecessor.gValue;    //calculate rhs-value based on predecessor
        }

        public static NativeArray<CellNode> ResetNodes(NativeArray<CellNode> nodes)
        {
            NativeArray<CellNode> updatedNodes = nodes;
            for (int i = 0; i < nodes.Length; i++)
            {
                CellNode tempNode = nodes[i];
                tempNode.gValue = math.INFINITY_DBL;
                tempNode.rhsValue = math.INFINITY_DBL;
                tempNode.parentNode = -1;

                double2 key = nodes[i].priorityKey;
                key[0] = math.INFINITY_DBL;
                key[1] = math.INFINITY_DBL;
                tempNode.priorityKey = key;

                tempNode.isInOpenList = false;
                updatedNodes[i] = tempNode;
            }
            

            return updatedNodes;
        }

        public static NativeHashSet<int> GetDiffTree(NativeHashSet<int> searchTree, NativeHashSet<int> subTree)
        {
            NativeHashSet<int> diffTree = new NativeHashSet<int>(0, Allocator.Temp);
            NativeArray<int> tempSearchTree = searchTree.ToNativeArray(Allocator.Temp);
            NativeArray<int> tempSubTree = subTree.ToNativeArray(Allocator.Temp);

            for (int i = 0; i < tempSearchTree.Length; i++)
            {
                diffTree.Add(tempSearchTree[i]);
            }
            
            for (int i = 0; i < tempSubTree.Length; i++)
            {
                diffTree.Remove(tempSubTree[i]);
            }

            return diffTree;
        }
    }
}
