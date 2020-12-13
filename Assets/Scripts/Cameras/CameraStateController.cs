using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(CameraControllerInput))]
    public class CameraStateController : MonoBehaviour
    {
        private CameraControllerInput input;

        public enum CameraState { FollowingTarget, OrbitingTarget, TargetMovingTowardsCamera, TransitioningFromOrbitToFollow };
        private CameraState state = CameraState.FollowingTarget;

        public float movingTowardsCameraAngleRange = 15f;

        void Start()
        {
            input = GetComponent<CameraControllerInput>();
        }
        
        public CameraState UpdateCameraState(Camera cam, GameObject followTarget, ThirdPersonCameraController.CameraBehaviourMode camMode)
        {
            switch(state)
            {
                case CameraState.FollowingTarget:
                    if (input.OrbitAngleChanged && CanOrbit(camMode))
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (TargetIsMovingTowardsCamera(cam, followTarget))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    break;

                case CameraState.TargetMovingTowardsCamera:
                    if (input.OrbitAngleChanged && CanOrbit(camMode))
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (!TargetIsMovingTowardsCamera(cam, followTarget))
                    {
                        state = CameraState.FollowingTarget;
                    }
                    break;

                case CameraState.OrbitingTarget:
                    if (!input.OrbitAngleChanged || !CanOrbit(camMode))
                    {
                        state = CameraState.TransitioningFromOrbitToFollow;
                    }
                    break;

                case CameraState.TransitioningFromOrbitToFollow:
                    //check orbit-to-follow hold timer/transition timer
                    //check target moving towards camera?
                    throw new System.NotImplementedException();
                    break;

                default:
                    break;

            }

            return state;
        }

        public CameraState GetCameraState()
        {
            return state;
        }

        private bool TargetIsMovingTowardsCamera(Camera cam, GameObject followTarget)
        {
            throw new System.NotImplementedException();
            //return Vector3.Angle(cam.transform.forward, followTarget.transform.forward) >
        }

        private bool CanOrbit(ThirdPersonCameraController.CameraBehaviourMode camMode)
        {
            return (camMode == ThirdPersonCameraController.CameraBehaviourMode.Orbit || camMode == ThirdPersonCameraController.CameraBehaviourMode.FollowAndOrbit);
        }
    }
}