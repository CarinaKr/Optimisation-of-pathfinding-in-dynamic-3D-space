using System;
using System.Collections.Generic;
using System.Text;
using Pathfinding.Algorithms;
using UnityEngine;
using UnityEngine.LowLevel;

namespace Util
{
    public static class PlayerLoopHelper
    {
        //based on: https://www.grizzly-machine.com/entries/maximizing-the-benefit-of-c-jobs-using-unitys-new-playerloop-api
        //and: https://forum.unity.com/threads/does-script-execution-order-work-with-custom-playerloops.1164328/
        public static void SetCustomGameLoop(int subSystemIndex, Type type, PlayerLoopSystem.UpdateFunction updateFunction, bool first)
        {
            PlayerLoopSystem playerLoop = PlayerLoop.GetCurrentPlayerLoop();
            PlayerLoopSystem playerLoopSystem = playerLoop.subSystemList[subSystemIndex];  

            // Convert the earlyUpdate subsystems to a list 
            List<PlayerLoopSystem> subsystemList = new List<PlayerLoopSystem>(playerLoopSystem.subSystemList);

            PlayerLoopSystem pathfinderSystem = new PlayerLoopSystem
            {
                type = type, 
                updateDelegate = updateFunction
            };

            // Add the new subsystem to the front of the update system
            subsystemList.Insert(first ? 0 : subsystemList.Count-1, pathfinderSystem);
            playerLoopSystem.subSystemList = subsystemList.ToArray();
            playerLoop.subSystemList[subSystemIndex] = playerLoopSystem;
        
            // Once done, tell Unity to use our modified loop
            PlayerLoop.SetPlayerLoop(playerLoop);
        }
        
        //source: https://blog.beardphantom.com/post/190674647054/unity-2018-and-playerloop
        public static void RecursivePlayerLoopPrint(PlayerLoopSystem def, StringBuilder sb, int depth)
        {
            if (depth == 0)
            {
                sb.AppendLine("ROOT NODE");
            }
            else if (def.type != null)
            {
                for (int i = 0; i < depth; i++)
                {
                    sb.Append("\t");
                }
                sb.AppendLine(def.type.Name);
            }
            if (def.subSystemList != null)
            {
                depth++;
                foreach (var s in def.subSystemList)
                {
                    RecursivePlayerLoopPrint(s, sb, depth);
                }
                depth--;
            }
        }
    }
}
