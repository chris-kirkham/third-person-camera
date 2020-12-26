using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(CameraControllerInput))]
    [RequireComponent(typeof(SharedCameraComponents))]
    public class CameraStateController : MonoBehaviour
    {
        /* Components */
        private Camera cam;
        private CameraControllerInput input;
        private SharedCameraComponents components;

        /* State trackers */
        private CameraState state = CameraState.FollowingTarget;
        private float orbitToFollowDelayCounter = 0f;
        private float orbitToFollowTimeCounter = 0f;
        private float orbitToFollowRampUpMultiplier = 0f;

        private void Start()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            components = GetComponent<SharedCameraComponents>();
        }

        private void Update()
        {
            UpdateOrbitToFollowDelayCounter(Time.deltaTime);
            UpdateOrbitToFollowTransitionTimeCounter(Time.deltaTime);
        }

        public void UpdateCameraState()
        {
            switch(state)
            {
                case CameraState.FollowingTarget:
                    if (IsOrbiting())
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (TargetIsMovingTowardsCamera(cam, components.followTarget, components.cameraParams) && TargetCanMoveTowardsCamera(components.cameraParams))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    break;

                case CameraState.TargetMovingTowardsCamera:
                    if (IsOrbiting())
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (!TargetIsMovingTowardsCamera(cam, components.followTarget, components.cameraParams))
                    {
                        state = CameraState.FollowingTarget;
                    }
                    break;

                case CameraState.OrbitingTarget:
                    if (!input.OrbitAngleChanged && CanFollow())
                    {
                        state = CameraState.OrbitToFollow_HoldingOrbitAngle;
                    }
                    break;

                case CameraState.OrbitToFollow_HoldingOrbitAngle: 
                    if(IsOrbiting())
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if(TargetIsMovingTowardsCamera(cam, components.followTarget, components.cameraParams) && TargetCanMoveTowardsCamera(components.cameraParams))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    else if(orbitToFollowDelayCounter <= 0)
                    {
                        state = CameraState.OrbitToFollow_Transitioning;
                    }
                    break;
                case CameraState.OrbitToFollow_Transitioning:
                    if (IsOrbiting())
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if (TargetIsMovingTowardsCamera(cam, components.followTarget, components.cameraParams) && TargetCanMoveTowardsCamera(components.cameraParams))
                    {
                        state = CameraState.TargetMovingTowardsCamera;
                    }
                    else if(orbitToFollowTimeCounter <= 0)
                    {
                        state = CameraState.FollowingTarget;
                    }
                    break;

                default:
                    break;

            }
        }

        public CameraState GetCameraState()
        {
            return state;
        }

        private bool TargetIsMovingTowardsCamera(Camera cam, GameObject followTarget, CameraParams cameraParams)
        {
            Vector3 camForwardFlat = new Vector3(cam.transform.forward.x, 0f, cam.transform.forward.z).normalized;
            Vector3 targetForwardFlat = new Vector3(followTarget.transform.forward.x, 0f, followTarget.transform.forward.z).normalized;
            return Mathf.Abs(Vector3.SignedAngle(camForwardFlat, targetForwardFlat, Vector3.up)) > (180 - cameraParams.movingTowardsCameraAngleRange);
        }

        private bool IsOrbiting()
        {
            return input.OrbitAngleChanged && CanOrbit();
        }

        private bool CanOrbit()
        {
            CameraBehaviourMode camMode = components.cameraParams.camMode;
            return (camMode == CameraBehaviourMode.Orbit || camMode == CameraBehaviourMode.FollowAndOrbit);
        }

        private bool CanFollow()
        {
            CameraBehaviourMode camMode = components.cameraParams.camMode;
            return (camMode == CameraBehaviourMode.FollowAndOrbit || camMode == CameraBehaviourMode.Follow);
        }

        private bool TargetCanMoveTowardsCamera(CameraParams cameraParams)
        {
            return cameraParams.allowMoveTowardsCamera;
        }

        private void UpdateOrbitToFollowTransitionTimeCounter(float deltaTime)
        {
            if (state != CameraState.OrbitToFollow_Transitioning) //reset counter
            {
                orbitToFollowTimeCounter = components.cameraParams.orbitToFollowTransitionTime;
            }
            else //decrement counter
            {
                orbitToFollowTimeCounter -= deltaTime;
            }
        }

        private void UpdateOrbitToFollowTransitionRamp(float deltaTime)
        {
            if (state != CameraState.OrbitToFollow_Transitioning) //reset counter
            {
            }
            else //decrement counter
            {
            }
        }

        public float GetOrbitToFollowTransitionTimeCounter()
        {
            return orbitToFollowTimeCounter;
        }

        private void UpdateOrbitToFollowDelayCounter(float deltaTime)
        {
            if (state != CameraState.OrbitToFollow_HoldingOrbitAngle) //reset counter
            {
                orbitToFollowDelayCounter = components.cameraParams.orbitToFollowHoldTime;
            }
            else //decrement counter
            {
                orbitToFollowDelayCounter -= deltaTime;
            }
        }

        public float GetOrbitToFollowDelayTimeCounter()
        {
            return orbitToFollowDelayCounter;
        }
    }
}