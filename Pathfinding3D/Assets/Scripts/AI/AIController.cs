using System.Collections.Generic;
using System.Linq;
using Pathfinding;
using Pathfinding.Algorithms;
using Pathfinding.Nodes;
using Pathfinding.SearchGraph;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Util;
using PathfindingHelper = Pathfinding.PathfindingHelper;

namespace AI
{
    public class AIController : MonoBehaviour
    {
        public delegate void ResetAI();

        [SerializeField] private Transform destination;
        [SerializeField] private bool drawCorridor;
        [SerializeField] private Color[] corridorColors;
        private List<float3> positionCorridor;
        private int corridorCounter;
        private bool corridorDone;
        private bool corridorDrawn = true;
        private AIMovement movement;

        private float3 nextPosition;
        private float3 lastPosition;
        private bool resetThisFrame;

        [SerializeField] private PathfindingHelper pathfindingHelper;

        public void Reset()
        {
            movement.Reposition();
            corridorDone = false;

            nextPosition = transform.position;
            positionCorridor.Clear();
            resetThisFrame = true;
            OnResetAI();
        }

        private void Awake()
        {
            movement = GetComponent<AIMovement>();
            positionCorridor = new List<float3>();

            movement.Reposition();
            nextPosition = transform.position;
            lastPosition = nextPosition;
        }

        private void Update()
        {
            if (Vector3.Distance(nextPosition, transform.position) < 0.1)
            {
                if (positionCorridor.Count > 1 && !corridorDone)
                {
                    positionCorridor.RemoveAt(0);
                    lastPosition = nextPosition;
                    nextPosition = positionCorridor[0];


                    if (positionCorridor.Count == 0)
                    {
                        corridorDone = true;
                    }
                }
                else if (pathfindingHelper.LineOfSight(transform.position, destination.position, true))
                {
                    lastPosition = nextPosition;
                    nextPosition = destination.position;
                    corridorDone = true;
                }
                else
                {
                    lastPosition = nextPosition;
                    nextPosition = transform.position;
                    corridorDone = false;
                }
            }

            movement.TargetPoint = nextPosition;
            

            if (drawCorridor && !corridorDrawn)
            {
                DrawCorridor();
                corridorDrawn = true;
            }
        }


        private void OnEnable()
        {
            Pathfinder.OnNewCorridorList += NewCorridorList;
        }

        private void OnDisable()
        {
            Pathfinder.OnNewCorridorList -= NewCorridorList;
        }

        public static event ResetAI OnResetAI;

        public void ResetCorridorMovement()
        {
            positionCorridor.Clear();
        }

        private void NewCorridorList(List<float3> newCorridor)
        {
            positionCorridor = newCorridor;
            corridorDrawn = false;

            // if (Vector3.Distance(positionCorridor[1], transform.position) < 0.1)
            if(positionCorridor.Count > 1 && ((nextPosition.Equals(positionCorridor[1]) && lastPosition.Equals(positionCorridor[0])) || Vector3.Distance(positionCorridor[1], transform.position) < 0.1))
            {
                positionCorridor.RemoveAt(0);
                lastPosition = nextPosition;
                nextPosition = positionCorridor[0];
            }
            
            if (!corridorDone)
            {
                nextPosition = positionCorridor[0];
            }
        }
        

        public float3 GetLookAheadPosition()
        {
            if (resetThisFrame)
            {
                resetThisFrame = false;
                return (float3) transform.position;
            }
            return nextPosition;
        }

        private void DrawCorridor()
        {
            for (var i = 0; i < positionCorridor.Count - 1; i++)
            {
                Debug.DrawLine(positionCorridor[i], positionCorridor[i + 1], corridorColors[corridorCounter % corridorColors.Length], 3f);
            }

            corridorCounter++;
        }
    }
}