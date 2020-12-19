using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(CameraControllerInput))]
    public class CameraStateController : MonoBehaviour
    {
        private CameraControllerInput input;

        public float movingTowardsCameraAngleRange = 15f;
        [Min(0)] public float orbitToFollowTransitionDelay = 0f;
        private float orbitToFollowDelayCounter = 0f;

        private CameraState state = CameraState.FollowingTarget;

        private void Start()
        {
            input = GetComponent<CameraControllerInput>();
        }

        private void Update()
        {
            if (state != CameraState.TransitioningFromOrbitToFollow)
            {
                ResetOrbitToFollowDelayCounter();
            }
            else
            {
                orbitToFollowDelayCounter -= Time.deltaTime;
            }
        }

        public CameraState UpdateCameraState(Camera cam, GameObject followTarget, CameraParams camParams)
        {
            switch(state)
            {
                case CameraState.FollowingTarget:
                    if (input.OrbitAngleChanged && CanOrbit(camParams.camMode))
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (TargetIsMovingTowardsCamera(cam, followTarget) && TargetCanMoveTowardsCamera(camParams))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    break;

                case CameraState.TargetMovingTowardsCamera:
                    if (input.OrbitAngleChanged && CanOrbit(camParams.camMode))
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (!TargetIsMovingTowardsCamera(cam, followTarget))
                    {
                        state = CameraState.FollowingTarget;
                    }
                    break;

                case CameraState.OrbitingTarget:
                    if (!input.OrbitAngleChanged && CanFollow(camParams.camMode))
                    {
                        state = CameraState.TransitioningFromOrbitToFollow;
                    }
                    break;

                case CameraState.TransitioningFromOrbitToFollow:
                    //check orbit-to-follow hold timer/transition timer
                    //check target moving towards camera?
                    if(input.OrbitAngleChanged && CanOrbit(camParams.camMode))
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if(TargetIsMovingTowardsCamera(cam, followTarget) && TargetCanMoveTowardsCamera(camParams))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    else if(orbitToFollowDelayCounter <= 0)
                    {
                        state = CameraState.FollowingTarget;
                    }
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
            Vector3 camForwardFlat = new Vector3(cam.transform.forward.x, 0f, cam.transform.forward.z).normalized;
            Vector3 targetForwardFlat = new Vector3(followTarget.transform.forward.x, 0f, followTarget.transform.forward.z).normalized;
            return Mathf.Abs(Vector3.SignedAngle(camForwardFlat, targetForwardFlat, Vector3.up)) > (180 - movingTowardsCameraAngleRange);
        }

        /*
        private bool IsOrbiting(CameraBehaviourMode camMode)
        {
            return input.OrbitAngleChanged 
                && (camMode == CameraBehaviourMode.Orbit || camMode == CameraBehaviourMode.FollowAndOrbit);
        }
        */

        private bool CanOrbit(CameraBehaviourMode camMode)
        {
            return (camMode == CameraBehaviourMode.Orbit || camMode == CameraBehaviourMode.FollowAndOrbit);
        }

        private bool CanFollow(CameraBehaviourMode camMode)
        {
            return (camMode == CameraBehaviourMode.FollowAndOrbit || camMode == CameraBehaviourMode.Follow);
        }

        private bool TargetCanMoveTowardsCamera(CameraParams cameraParams)
        {
            return cameraParams.allowMoveTowardsCamera;
        }

        private void ResetOrbitToFollowDelayCounter()
        {
            orbitToFollowDelayCounter = orbitToFollowTransitionDelay;
        }
    }
}