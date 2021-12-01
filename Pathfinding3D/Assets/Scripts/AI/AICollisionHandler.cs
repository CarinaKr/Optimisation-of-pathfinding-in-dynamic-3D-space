using Obstacles;
using UnityEngine;
using Util;

namespace AI
{
    public class AICollisionHandler : MonoBehaviour
    {
        [SerializeField] private string[] obstacleTags;
        private StaticMovement currentMovement;
        private bool isCollided;

        private AIController ownController;

        private void Start()
        {
            ownController = GetComponent<AIController>();
        }

        private void Update()
        {
            isCollided = false;
        }

        private void OnTriggerEnter(Collider other)
        {
            for (var i = 0; i < obstacleTags.Length; i++)
            {
                if (other.transform.tag == obstacleTags[i])
                {
                    currentMovement = other.transform.GetComponentInParent<StaticMovement>();

                    if (isCollided || (currentMovement != null && currentMovement.Tagged))
                    {
                        return;
                    }

                    isCollided = true;
                    if (currentMovement != null)
                    {
                        currentMovement.Tagged = true;
                    }

                    // Debug.Break();
                    ownController.Reset();
                    Debug.Log("reset "+other.name);
                    // FileHandler.self.WriteDebugString("reset\n\n");
                }
            }


            if (other.transform.tag == "Player")
            {
                ownController.Reset();
                //count success
                Debug.Log("success");
                // FileHandler.self.WriteDebugString("success\n\n");
            }
        }
    }
}