using UnityEngine;
using Util;

namespace AI
{
    public class AIMovement : MonoBehaviour
    {
        [SerializeField] private float moveSpeed;
        [SerializeField] private Transform startPoint;

        private Vector3 lastPosition;

        public Vector3 TargetPoint { private get; set; }
        public float DistanceTraveled { get; set; }

        private void Start()
        {
            lastPosition = new Vector3();
        }

        private void Update()
        {
            lastPosition = transform.position;

            if (TargetPoint.x < Mathf.Infinity)
            {
                transform.position = Vector3.MoveTowards(transform.position, TargetPoint, moveSpeed * Time.deltaTime);
                transform.LookAt(TargetPoint);
            }

            DistanceTraveled += Vector3.Distance(lastPosition, transform.position);
        }

        public void Reposition()
        {
            FileHandler.self.WriteDebugString("distanceTraveled:"+DistanceTraveled);
            DistanceTraveled = 0;
            if (startPoint != null)
            {
                transform.position = startPoint.position;
            }

            TargetPoint = Vector3.positiveInfinity;
        }
    }
}