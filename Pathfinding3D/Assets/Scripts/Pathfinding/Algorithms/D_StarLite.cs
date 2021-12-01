// using System;
// using System.Collections;
// using System.Collections.Generic;
// using System.Threading.Tasks;
// using Pathfinding.Nodes;
// using Pathfinding.SearchGraph;
// using UnityEngine;
// using UnityEngine.Profiling;
// using Util;
//
// namespace Pathfinding.Algorithms
// {
//     public class D_StarLite : D_starBased
//     {
//         public bool useMT; //Moving Target D* Lite optimization
//         private HashSet<Node> subTree;
//
//         protected float km;
//         protected Node lastStart;
//
//         protected override void StartPathfinding()
//         {
//             base.StartPathfinding();
//             subTree = new HashSet<Node>();
//         }
//
//         protected override void FindPath()
//         {
//             FileHandler.self.WriteString("D* Lite with epsilon " + PathfinderSetup.Epsilon + " and Basic MT=" + useMT + "\n");
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
//                 algTimer.Restart(); //MEASURE data for static AI and destination
//                 // algTimer.Start(); //MEASURE data for all dynamic
//                 startNode = ufoController.GetLookAheadNode(PathfinderSetup.Lookahead);
//                 bool runAlgorithm = false;
//
//                 if (resetAlgorithm)
//                 {
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
//
//                     resetAlgorithm = false;
//                 }
//                 else if (changes.Count > 0)
//                 {
//                     publishTimer.Restart();
//                     isUpdatingNodes = true;
//                     searchGraphManager.PublishLastScreenshot(changes);
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
//                     if ((searchGraphManager is WaypointManager && changes.Count > 0) || (oldGoalNode != goalNode && !useMT))
//                     {
//                         fastOpenListPrim.Clear();
//                         km = 0;
//                         searchGraphManager.ResetNodes();
//                         goalNode.rhsValue = 0;
//                         AddToFastOpen(goalNode);
//                         runAlgorithm = true;
//                     }
//                     else if (oldGoalNode != goalNode && useMT)
//                     {
//                         goalNode.parentNode = null;
//                         changes.Add(oldGoalNode);
//                         runAlgorithm = true;
//
//                         UpdateChangedNodes(changes);
//                     }
//                     else
//                     {
//                         UpdateChangedNodes(changes);
//                     }
//
//                     changes.Clear();
//                     isUpdatingNodes = false;
//                 }
//
//                 if(runAlgorithm)
//                 {
//                     if (PathfinderSetup.Visualize)
//                     {
//                         searchGraphManager.ResetVisualization();
//                     }
//                     Profiler.BeginSample("D star lite");
//                     ShortestPathLoop();
//                     Profiler.EndSample();
//                     algTimer.Stop();
//                     FileHandler.self.WriteString("expanded: " + expanded + "\n");
//                     runCounter++;
//                     if (runCounter == PathfinderSetup.StopAtRunCount)
//                     {
//                         Debug.Break();
//                     }
//                 }
//                 // await new WaitForUpdate();
//
//                 //MEASURE data for static AI and destination
//                 // FileHandler.self.WriteString(":publishTime: " + publishTimer.Elapsed.Milliseconds);
//                 // FileHandler.self.WriteString(": algorithm time: " + algTimer.Elapsed.Milliseconds + " : expanded: " + expanded); //MEASURE
//                 // FileHandler.self.WriteString(": path length: " + pathLength + "\n"); //MEASURE data for static AI and destination
//                 // FileHandler.self.WriteString("expanded: " + expanded + "\n");
//                 
//             }
//         }
//
//         public async Task ShortestPathLoop()
//         {
//             Profiler.BeginSample("setup");
//             expanded = 0; //MEASURE data for static AI and destination
//             HashSet<Node> expandedNodes = new HashSet<Node>();
//             pathFound = true;
//             if (startNode == null)
//             {
//                 startNode = searchGraphManager.GetStartNode();
//             }
//             Profiler.EndSample();
//
//             // Profiler.BeginSample("get next node");
//             nextNode = GetNextNodeFast();
//             // Profiler.EndSample();
//
//             if (startNode.publicIsFree && nextNode != null)
//             {
//                 Profiler.BeginSample("loop");
//                 while ((startNode.gValue != startNode.rhsValue || IsLowerKey(nextNode, CalculateKeyValue(startNode))) && isRunning)
//                 {
//                     expanded++;
//                     if (PathfinderSetup.Visualize)
//                     {
//                         nextNode.ExpandedColor = searchGraphManager.CurrentExpandedColor;
//                         nextNode.IsExpanded = true;
//                         if (expandedNodes.Contains(nextNode))
//                         {
//                             await new WaitForUpdate();
//                             nextNode.ExpandedColor = Color.black;
//                         }
//                     }
//                     expandedNodes.Add(nextNode);
//
//                     if (fastOpenListPrim.Count == 0)
//                     {
//                         Debug.Log("empty open list");
//                     }
//
//                     Profiler.BeginSample("calculate key value");
//                     double[] oldKey = nextNode.priorityKey;
//                     double[] newKey = CalculateKeyValue(nextNode);
//                     Profiler.EndSample();
//                     
//                     if (oldKey[0] < newKey[0] || (oldKey[0] == newKey[0] && oldKey[1] < newKey[1]))
//                     {
//                         Profiler.BeginSample("update key value");
//                         nextNode.priorityKey = CalculateKeyValue(nextNode);
//                         fastOpenListPrim.UpdatePriority(nextNode, (float) nextNode.priorityKey[0]);
//                         Profiler.EndSample();
//                     }
//                     else if (nextNode.gValue > nextNode.rhsValue) //overconsistent
//                     {
//                         nextNode.gValue = nextNode.rhsValue; //set g-value
//                         Profiler.BeginSample("remove from open list");
//                         if (fastOpenListPrim.Contains(nextNode))
//                         {
//                             fastOpenListPrim.Remove(nextNode);
//                         }
//                         Profiler.EndSample();
//
//                         Profiler.BeginSample("update rhs of predecessors");
//                         foreach (Node pred in nextNode.publicPredecessors) //update rhs-value of all predecessors
//                         {
//                             UpdateNode(pred); //possible optimization: rhs=min(oldRHS, g(nextNode)+c(pred,nextNode))
//                         }
//                         Profiler.EndSample();
//                     }
//                     else //underconsistent
//                     {
//                         nextNode.gValue = Mathf.Infinity; //set g-value to infinite
//                         
//                         Profiler.BeginSample("update own rhs");
//                         UpdateNode(nextNode); //update own rhs-value
//                         Profiler.EndSample();
//                         
//                         Profiler.BeginSample("update rhs of predecessors");
//                         foreach (Node pred in nextNode.publicPredecessors) //update rhs-value of all predecessors
//                         {
//                             if (pred.parentNode == null || pred.parentNode == nextNode)
//                             {
//                                 UpdateNode(pred);
//                             }
//                         }
//                         Profiler.EndSample();
//                     }
//
//                     Profiler.BeginSample("get next node");
//                     nextNode = GetNextNodeFast();
//                     Profiler.EndSample();
//                     if (nextNode == null)
//                     {
//                         pathFound = false;
//                     }
//                 }
//                 Profiler.EndSample();
//
//                 Profiler.BeginSample("create corridor");
//                 count = 0;
//                 corridor.Clear();
//                 if (pathFound)
//                 {
//                     FillCorridorByPathCost(startNode);
//                     DrawCorridor();
//                 }
//                 Profiler.EndSample();
//             }
//             else
//             {
//                 Debug.Log("start node is blocked: no shortest path found");
//             }
//
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
//             // Profiler.BeginSample("update node");
//             base.UpdateNode(node); //update rhs-value
//             // Profiler.EndSample();
//             
//             // Profiler.BeginSample("add or remove from list");
//             if (node.rhsValue != node.gValue) //if node is inconsistent
//             {
//                 // Profiler.BeginSample("add to open");
//                 AddToFastOpen(node); //add to open list
//                 // Profiler.EndSample();
//             }
//             else if (fastOpenListPrim.Contains(node)) //if node is consistent
//             {
//                 if (fastOpenListPrim.Count == 0)
//                 {
//                     Debug.Log("empty open list");
//                 }
//
//                 // Profiler.BeginSample("remove from open");
//                 fastOpenListPrim.Remove(node); //remove from open list
//                 // Profiler.EndSample();
//             }
//             // Profiler.EndSample();
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
//             // source.isInSubTree = true;
//
//             foreach (Node pred in source.publicPredecessors)
//             {
//                 if (pred.parentNode == source && !subTree.Contains(pred))
//                 {
//                     FillSubTree(pred);
//                 }
//             }
//         }
//     }
// }