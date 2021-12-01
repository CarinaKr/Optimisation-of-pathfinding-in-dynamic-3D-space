using UnityEngine;

namespace Obstacles
{
    public class StaticMovement : MonoBehaviour
    {
        [SerializeField] private bool usePatrolPoints;
        [SerializeField] private PatrolPointMarker[] patrolPoints;
        [SerializeField] private float moveSpeed, turnSpeed;
        [SerializeField] private Vector3 turnAround;
        [SerializeField] private bool forward, right, up;
        [SerializeField] private bool temporaryBlock;
        [SerializeField] private float switchTime;
        [SerializeField] private bool startActive;
        [SerializeField] private Transform AI;
        [SerializeField] private Vector3 boxMultiply;
        
        private Bounds box;
        private BoxCollider boxCollider;
        private bool isActive;
        private MeshRenderer meshRenderer;
        private int patrolCounter;

        private Vector3[] patrolPositions;
        private float switchTimeCounter;

        public bool Tagged { get; set; }

        private void Start()
        {
            Tagged = false;
            if (forward)
            {
                turnAround = transform.forward;
            }
            else if (right)
            {
                turnAround = transform.right;
            }
            else if (up)
            {
                turnAround = transform.up;
            }

            if (usePatrolPoints)
            {
                if (patrolPoints.Length == 0)
                {
                    //get patrol points from children
                    patrolPoints = GetComponentsInChildren<PatrolPointMarker>();
                }

                //save position of children in extra array since they will move along with the parent
                patrolPositions = new Vector3[patrolPoints.Length];
                for (int i = 0; i < patrolPoints.Length; i++)
                {
                    patrolPositions[i] = patrolPoints[i].transform.position;
                }
            }

            if (temporaryBlock)
            {
                boxCollider = GetComponent<BoxCollider>();
                box = boxCollider.bounds;
                box.extents = new Vector3(box.extents.x * boxMultiply.x, box.extents.y * boxMultiply.y, box.extents.z * boxMultiply.z);
                meshRenderer = GetComponentInChildren<MeshRenderer>();
                isActive = !startActive;
                ToggleActive();
            }
        }


        // Update is called once per frame
        private void Update()
        {
            Tagged = false;
            if (usePatrolPoints)
            {
                transform.position = Vector3.MoveTowards(transform.position, patrolPositions[patrolCounter],
                    moveSpeed * Time.deltaTime);
                if (Vector3.Distance(transform.position, patrolPositions[patrolCounter]) <= 0.5)
                {
                    patrolCounter = (patrolCounter + 1) % patrolPositions.Length;
                }
            }

            if (turnSpeed != 0)
            {
                transform.Rotate(turnAround, turnSpeed * Time.deltaTime);
            }

            if (temporaryBlock)
            {
                switchTimeCounter += Time.deltaTime;
                if (switchTime > 0 && switchTimeCounter >= switchTime)
                {
                    if (!box.Contains(AI.position))
                    {
                        ToggleActive();
                        switchTimeCounter = 0;
                    }
                }
            }
        }

        private void ToggleActive()
        {
            if (isActive)
            {
                boxCollider.enabled = false;
                meshRenderer.enabled = false;
                isActive = false;
            }
            else
            {
                boxCollider.enabled = true;
                meshRenderer.enabled = true;
                isActive = true;
            }
        }
    }
}