// using System;
// using System.Collections.Generic;
// using System.Linq;
// using System.Threading.Tasks;
// using Pathfinding.Nodes;
// using Pathfinding.SearchGraph;
// using UnityEngine;
// using UnityEngine.Profiling;
// using UnityEngine.Serialization;
// using Util;
//
// namespace Pathfinding.Algorithms
// {
//     public class MTD_StarLite : D_starBased
//     {
//         [FormerlySerializedAs("useMT")] public bool useBasicMT = true; //Moving Target D* Lite optimization
//         private List<Node> copyChanges;
//
//         protected float km;
//         protected Node lastStart;
//
//         private HashSet<Node> searchTree, subTree, difTree;
//
//         protected override void StartPathfinding()
//         {
//             base.StartPathfinding();
//             searchTree = new HashSet<Node>();
//             subTree = new HashSet<Node>();
//             copyChanges = new List<Node>();
//         }
//
//         protected override void FindPath()
//         {
//             FileHandler.self.WriteString("MT-D* Lite with epsilon " + PathfinderSetup.Epsilon + " and Basic MT=" + useBasicMT + "\n");
//
//             // await Task.Delay((int) delay).ConfigureAwait(false);
//             //await new WaitForBackgroundThread();                  //should work as well, but is not reliable
//             algTimer.Restart();
//             publishTimer.Restart();
//             searchGraphManager.PublishLastScreenshot();
//             publishTimer.Stop();
//             timer.Restart();
//             startNode = searchGraphManager.GetStartNode();
//             lastStart = startNode;
//             searchGraphManager.UpdateHeuristic(startNode, PathfinderSetup.Epsilon);
//             goalNode = searchGraphManager.GetGoalNode();
//             goalNode.rhsValue = 0;
//             AddToFastOpen(goalNode);
//             ShortestPathLoop();
//
//             changes.Clear();
//             resetAlgorithm = false;
//
//             algTimer.Stop();
//             timer.Stop();
//             // await new WaitForUpdate();
//
//             //FileHandler.self.WriteString(":publishTime: " + publishTimer.Elapsed.Milliseconds);   //MEASURE data for static AI and destination
//             //FileHandler.self.WriteString(":only alg time: " + timer.ElapsedMilliseconds );
//             //FileHandler.self.WriteString(":algorithm time: " + algTimer.Elapsed.Milliseconds);
//             //FileHandler.self.WriteString(":expanded: " + expanded + "\n");
//             
//             while (isRunning)
//             {
//                 // while (changes.Count == 0 && isRunning)
//                 // {
//                 //     await Task.Delay(TimeSpan.FromSeconds(0.01));
//                 // }
//
//                 // await Task.Delay((int) delay).ConfigureAwait(false);
//                 //await new WaitForBackgroundThread();
//                 //algTimer.Restart();   //MEASURE data for static AI and destination
//                 algTimer.Start(); //MEASURE data for all dynamic
//                 startNode = ufoController.GetLookAheadNode(PathfinderSetup.Lookahead);
//
//                 bool runAlgorithm = false;
//                 if (resetAlgorithm)
//                 {
//                     searchTree.Clear();
//                     //FileHandler.self.WriteString("reset algorithm");
//                     algTimer.Stop(); //MEASURE data in all-dynamic environment
//                     //MEASURE data in all-dynamic environment
//                     // FileHandler.self.WriteString(":expanded: " + expanded);
//                     // FileHandler.self.WriteString(":total algorithm time: " + algTimer.ElapsedMilliseconds + "\n");
//                     expanded = 0;
//                     algTimer.Restart();
//
//                     ufoController.ResetCorridorMovement();
//                     publishTimer.Restart();
//                     searchGraphManager.PublishLastScreenshot();
//                     publishTimer.Stop();
//                     //FileHandler.self.WriteString("publish time: " + publishTimer.ElapsedMilliseconds+" ");    //MEASURE
//
//                     changes.Clear();
//
//                     fastOpenListPrim.Clear();
//                     km = 0;
//                     searchGraphManager.ResetNodes();
//                     startNode = searchGraphManager.GetStartNode();
//                     searchGraphManager.UpdateHeuristic(startNode, PathfinderSetup.Epsilon);
//                     goalNode = searchGraphManager.GetGoalNode();
//                     goalNode.rhsValue = 0;
//                     AddToFastOpen(goalNode);
//
//                     resetAlgorithm = false;
//                 }
//                 else if (changes.Count > 0)
//                 {
//                     publishTimer.Restart();
//                     isUpdatingNodes = true;
//                     copyChanges = new List<Node>(changes); //use copy
//                     changes.Clear(); //use copy
//                     searchGraphManager.PublishLastScreenshot( /*changes[0]*/copyChanges); //use copy
//                     publishTimer.Stop();
//                     //FileHandler.self.WriteString("publish time: " + publishTimer.ElapsedMilliseconds);    //MEASURE 
//
//                     if (startNode == null)
//                     {
//                         startNode = searchGraphManager.GetStartNode();
//                     }
//                     else if (!startNode.publicIsFree)
//                     {
//                         startNode = searchGraphManager.GetStartNode();
//                     }
//
//                     km += Vector3.Distance(lastStart.globalPosition, startNode.globalPosition) * PathfinderSetup.Epsilon;
//                     lastStart = startNode;
//
//                     searchGraphManager.UpdateHeuristic(startNode, PathfinderSetup.Epsilon);
//                     Node oldGoalNode = goalNode;
//                     goalNode = searchGraphManager.GetGoalNode();
//
//                     // if (searchGraphManager is WaypointManager && /*changes[0]*/copyChanges.Count > 0 || oldGoalNode != goalNode && !useMT) //use copy
//                     // {
//                     //     fastOpenListPrim.Clear();
//                     //     km = 0;
//                     //     searchGraphManager.ResetNodes();
//                     //     goalNode.rhsValue = 0;
//                     //     AddToFastOpen(goalNode);
//                     //     runAlgorithm = true;
//                     // }
//                     if (oldGoalNode != goalNode && useBasicMT)
//                     {
//                         goalNode.parentNode = null;
//                         changes.Add(oldGoalNode);
//                         runAlgorithm = true;
//
//                         UpdateChangedNodes(changes);
//                     }
//                     else if (oldGoalNode != goalNode && !useBasicMT)
//                     {
//                         runAlgorithm = true;
//                         goalNode.parentNode = null;
//                         subTree.Clear();
//                         FillSubTree(goalNode);
//                         if (PathfinderSetup.Visualize && subTree.Count > 100)
//                         {
//                             VisualizeSubTree(subTree.ToArray());
//                             Debug.Break();
//                         }
//                         difTree = new HashSet<Node>(searchTree.Except(subTree));
//                         foreach (Node node in difTree)
//                         {
//                             node.parentNode = null;
//                             node.rhsValue = Mathf.Infinity;
//                             node.gValue = Mathf.Infinity;
//                             if (fastOpenListPrim.Contains(node))
//                             {
//                                 fastOpenListPrim.Remove(node);
//                             }
//                         }
//
//                         foreach (Node node in difTree)
//                         {
//                             UpdateNode(node);
//                         }
//
//                         UpdateChangedNodes( /*changes[0]*/copyChanges);
//                     }
//                     else
//                     {
//                         UpdateChangedNodes( /*changes[0]*/copyChanges);
//                     }
//
//                     //changes.Clear();  //use copy
//                     isUpdatingNodes = false;
//                 }
//
//                 if (runAlgorithm)
//                 {
//                     if (PathfinderSetup.Visualize)
//                     {
//                         searchGraphManager.ResetVisualization();
//                     }
//
//                     ShortestPathLoop();
//                     FileHandler.self.WriteString("expanded: " + expanded + "\n");
//                     algTimer.Stop();
//                     runCounter++;
//                     if (runCounter == PathfinderSetup.StopAtRunCount)
//                     {
//                         Debug.Break();
//                     }
//                 }
//
//                 // await new WaitForUpdate();
//
//                 //MEASURE data for static AI and destination
//                 //FileHandler.self.WriteString(":publishTime: " + publishTimer.Elapsed.Milliseconds);
//                 //FileHandler.self.WriteString(": algorithm time: " + algTimer.Elapsed.Milliseconds + " : expanded: " + expanded + "\n");    //MEASURE
//                 //FileHandler.self.WriteString("expanded: " + expanded + "\n");
//             }
//         }
//
//         public void ShortestPathLoop()
//         {
//             Profiler.BeginSample("MT-D_lite");
//             expanded = 0; //MEASURE data for static AI and destination
//             pathFound = true;
//             if (startNode == null)
//             {
//                 startNode = searchGraphManager.GetStartNode();
//             }
//
//             if (startNode == null)
//             {
//                 Debug.Log("start node is null");
//             }
//
//             nextNode = GetNextNodeFast();
//             if (nextNode == null)
//             {
//                 Debug.Log("next node is null");
//             }
//
//             if (startNode.publicIsFree && nextNode != null)
//             {
//                 while ((startNode.gValue != startNode.rhsValue || IsLowerKey(nextNode, CalculateKeyValue(startNode))) &&
//                        isRunning)
//                 {
//                     expanded++;
//                     if (PathfinderSetup.Visualize)
//                     {
//                         nextNode.ExpandedColor = searchGraphManager.CurrentExpandedColor;
//                         nextNode.IsExpanded = true;
//                     }
//
//                     if (fastOpenListPrim.Count == 0)
//                     {
//                         Debug.Log("empty open list");
//                     }
//
//                     double[] oldKey = nextNode.priorityKey;
//                     double[] newKey = CalculateKeyValue(nextNode);
//                     if (oldKey[0] < newKey[0] || oldKey[0] == newKey[0] && oldKey[1] < newKey[1])
//                     {
//                         nextNode.priorityKey = CalculateKeyValue(nextNode);
//                         fastOpenListPrim.UpdatePriority(nextNode, (float) nextNode.priorityKey[0]);
//                     }
//                     else if (nextNode.gValue > nextNode.rhsValue) //overconsistent
//                     {
//                         nextNode.gValue = nextNode.rhsValue; //set g-value
//                         //openList.Remove(nextNode);
//                         if (fastOpenListPrim.Contains(nextNode))
//                         {
//                             fastOpenListPrim.Remove(nextNode);
//                         }
//
//                         foreach (Node pred in nextNode.publicPredecessors) //update rhs-value of all predecessors
//                         {
//                             UpdateNode(pred); //possible optimization: rhs=min(oldRHS, g(nextNode)+c(pred,nextNode))
//                         }
//                     }
//                     else //underconsistent
//                     {
//                         nextNode.gValue = Mathf.Infinity; //set g-value to infinite
//                         UpdateNode(nextNode); //update own rhs-value
//                         foreach (Node pred in nextNode.publicPredecessors) //update rhs-value of all predecessors
//                         {
//                             if (pred.parentNode == null || pred.parentNode == nextNode)
//                             {
//                                 UpdateNode(pred);
//                             }
//                         }
//                     }
//
//                     nextNode = GetNextNodeFast();
//                     if (nextNode == null)
//                     {
//                         pathFound = false;
//                     }
//                 }
//
//                 //Debug.Log("path found: " + pathFound);
//                 count = 0;
//                 corridor.Clear();
//                 if (pathFound)
//                 {
//                     FillCorridorByPathCost(startNode);
//                     DrawCorridor();
//                 }
//             }
//             else
//             {
//                 Debug.Log("start node is blocked: no shortest path found");
//             }
//
//             Profiler.EndSample();
//             //await new WaitForEndOfFrame();
//         }
//
//         protected void ResortOpenList()
//         {
//             var keys = new HashSet<Node>(fastOpenListPrim);
//             foreach (Node node in keys)
//             {
//                 node.priorityKey = CalculateKeyValue(node);
//                 fastOpenListPrim.UpdatePriority(node, (float) node.priorityKey[0]);
//             }
//         }
//
//         protected override void UpdateNode(Node node)
//         {
//             base.UpdateNode(node); //update rhs-value
//
//             if (node.rhsValue != node.gValue) //if node is inconsistent
//             {
//                 AddToFastOpen(node); //add to open list
//                 Profiler.BeginSample("add to search tree");
//                 if (!searchTree.Contains(node))
//                 {
//                     searchTree.Add(node);
//                     node.isInTree = true;
//                 }
//                 Profiler.EndSample();
//             }
//             else //if node is consistent
//             {
//                 if (fastOpenListPrim.Contains(node))
//                 {
//                     if (fastOpenListPrim.Count == 0)
//                     {
//                         Debug.Log("empty open list \n");
//                     }
//
//                     fastOpenListPrim.Remove(node); //remove from open list
//                 }
//                 
//                 Profiler.BeginSample("add to search tree");
//                 if (searchTree.Contains(node) && node.gValue == Mathf.Infinity)
//                 {
//                     searchTree.Remove(node);
//                     node.isInTree = false;
//                 }
//                 Profiler.EndSample();
//             }
//         }
//
//         protected override double[] CalculateKeyValue(Node child)
//         {
//             //keep for demonstrations
//             //double[] key = new double[2];                        
//             //key[1] = Mathf.Min((float)child.gValue, (float)child.rhsValue);
//             //key[0] = key[1] + child.heuristic + km;
//             //return key;
//
//             double[] key = new double[2];
//             if (child.gValue > child.rhsValue)
//             {
//                 key[0] = child.rhsValue + child.heuristic + km;
//                 key[1] = child.rhsValue;
//             }
//             else
//             {
//                 key[0] = child.gValue + child.plainHeuristic + km;
//                 key[1] = child.gValue;
//             }
//
//             return key;
//         }
//
//         private void FillSubTree(Node source)
//         {
//             subTree.Add(source);
//
//             foreach (Node pred in source.publicPredecessors)
//             {
//                 if (pred.parentNode == source && !subTree.Contains(pred))
//                 {
//                     FillSubTree(pred);
//                 }
//             }
//         }
//
//         private void FillSearchTree(Node source)
//         {
//             searchTree.Add(source);
//
//             foreach (Node pred in source.publicPredecessors)
//             {
//                 if (pred.parentNode == source && !searchTree.Contains(pred))
//                 {
//                     FillSearchTree(pred);
//                 }
//             }
//         }
//
//         private void VisualizeSubTree(Node[] nodes)
//         {
//             foreach (Node node in nodes)
//             {
//                 node.isInSubTree = true;
//             }
//         }
//     }
// }