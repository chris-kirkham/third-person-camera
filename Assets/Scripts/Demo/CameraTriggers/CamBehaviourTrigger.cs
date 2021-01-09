using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Changes the behaviour components (Follow target, look target, parameter preset) of ThirdPersonCameras which enter the trigger this script is attached to.
    /// </summary>
    public class CamBehaviourTrigger : MonoBehaviour
    {
        [Header("On trigger enter")]
        [SerializeField] private GameObject enterFollowTarget;
        [SerializeField] private GameObject enterLookTarget;
        [SerializeField] private CameraParams enterCamParams;

        [Header("On trigger exit")]
        [SerializeField] private bool revertToPreviousComponents = true;
        [SerializeField] private GameObject exitFollowTarget, exitLookTarget;
        [SerializeField] private CameraParams exitCamParams;

        //stores camera controller's existing components on trigger enter, to revert to on exit (if using)
        private GameObject currFollowTarget, currLookTarget;
        private CameraParams currCamParams;

        private void OnTriggerEnter(Collider other)
        {
            SharedCameraComponents camComponents = other.gameObject.GetComponent<CameraTriggerObject>()?.GetCameraComponents();
            if(camComponents != null)
            {
                //store camera's current components
                currFollowTarget = camComponents.followTarget;
                currLookTarget = camComponents.lookAtTarget;
                currCamParams = camComponents.GetCameraParams();

                //assign new components
                if (enterFollowTarget != null) camComponents.followTarget = enterFollowTarget;
                if (enterLookTarget != null) camComponents.lookAtTarget = enterLookTarget;
                if (enterCamParams != null) camComponents.SetCameraParams(enterCamParams);
            }
        }

        private void OnTriggerExit(Collider other)
        {
            SharedCameraComponents camComponents = other.gameObject.GetComponent<CameraTriggerObject>()?.GetCameraComponents();
            if (camComponents != null)
            {
                if(revertToPreviousComponents)
                {
                    if (currFollowTarget != null) camComponents.followTarget = currFollowTarget;
                    if (currLookTarget != null) camComponents.lookAtTarget = currLookTarget;
                    if (currCamParams != null) camComponents.SetCameraParams(currCamParams);
                }
                else
                {
                    if (exitFollowTarget != null) camComponents.followTarget = exitFollowTarget;
                    if (exitLookTarget != null) camComponents.lookAtTarget = exitLookTarget;
                    if (exitCamParams != null) camComponents.SetCameraParams(exitCamParams);
                }
            }
        }
    }
}