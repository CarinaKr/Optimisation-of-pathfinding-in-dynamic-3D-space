This repository contains the results of the master thesis:

### "Optimisation of pathfinding in a dynamic 3D space" ###

developed at the University of Applied Sciences Hamburg.

In addition to a written paper the thesis includes a project, where the pathfinding algorithms A*, JPS, Theta*, (Basic) MT-D* Lite, and AAMT-D* Lite were implemented in an abstract test environment using a cell grid search graph and the hybrid search graph introduced in the thesis. For this project the game engine Unity 2020.3.3f1 (LTS) was used.

For a video demonstrating the results of the implemented pathfinding algorithms and search graphs see: https://youtu.be/RUUsUAHiFQA
For a quick start guide on how to configure and use the developed pathfinding system in Unity see: https://youtu.be/Z96w9YHGilk 

For a life demonstration of the pathfinding system implemented in the scope of this thesis, download the "Pathfinding3D" folder and open the scene "Pathfinding" in the Unity project or import the package "pathfindingInDynamic3DSpace" into Unity and open the scene "Pathfinding". 

Enable the Burst compiler and turn off the safety checks.

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/burstCompiler.png">
</p>

You can choose between a cell grid and the hybrid search graph as the "Search Graph Type"-field of the "SearchGraphSelector"-component of the "SearchGraphManager"-object:

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/searchGraphSelector_marked.png">
</p>

The pathfinding algorithm can be selected as the "Pathfinding Method" in the "PathfindingManager"-component of the "PathfindingManager"-object. On the same component you can also select the inflation factor used for the pathfinding algorithm and whether or not the algorithm is performed on the main thread. When using the hybrid search graph it is advised to uncheck the "Wait on Main Thread" box and run the pathfinding algorithms on worker threads. When using the cell grid search graph and JPS it is advised to run the algorithm on the main thread.

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/pathfindingManager_marked.png">
</p>

Expanding the "PathfindingManager" you can select each algorithm to adjust other variables, such as the Line-of-Sight offset multiplier, whether to use the "Fast" variation of some algorithms, or whether to use the Basic version for MT-D* Lite and AAMT-D* Lite.

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/thetaStarSetup_marked.png">
</p>
<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/mtStarSetup_marked.png">
</p>


Before using the hybrid search graph for pathfinding, it has to be bakes. Select the "HybridManager"-object and hit the "Bake Search Graph" button. The editor will switch to play mode automatically and start baking the search graph. When the dynamic obstacles have been in every possible state once (e.g. they have fully traversed their path or rotated 360°), hit the "Stop Baking Search Graph" button to complete the baking process. The editor will exit the play mode automatically and save the search graph. Now you can set the search graph type to HYBRID and press play to start the pathfinding.

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/hybridManager_marked.png">
</p>

The hybrid search graph has some options for its visualization. First, you can choose between "Nodes" or "Blocked" as the "VisualizeMode".
The "Blocked" mode will show all currently blocked nodes in red. 

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/blocked_marked.png">
</p>

The "Nodes" mode will show the nodes of the hybrid search graph in different colour based on their type. *static* nodes are yellow, *dynamic* nodes are magenta, and *dynamic neighbour* nodes are cyan. If you only want to see certain nodes, you can select those in the "Visualize Nodes" field. Additionally, you can visualize the neighbours between nodes by enabling the neighbour types in the "Visualize Neighbours" field. *side* neighbours are shown in grey, *edge* neighbours in green, and *vertex* neighbours in white.

<p align="center">
  <img width="460" src="https://github.com/CarinaKr/Optimisation-of-pathfinding-in-dynamic-3D-space/blob/main/readmeImages/nodes_marked.png">
</p>

Make sure you have selected the "HybridManager" game object and enabled Gizmos to see the visualization of the search graph! Also, be aware that by visualizing the search graph the performance of the pathfinding system is impacted negatively.

