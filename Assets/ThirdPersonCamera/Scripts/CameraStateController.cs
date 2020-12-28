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
        private CameraTargetOrientationState orientation = CameraTargetOrientationState.AwayFromCamera;
        private float orbitToFollowDelayCounter = 0f;

        /* Constants */
        private const float offsetNearEnoughMinDot = 0.9f;

        private void Start()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            components = GetComponent<SharedCameraComponents>();
        }

        private void Update()
        {
            UpdateOrbitToFollowDelayCounter(Time.deltaTime);
            //UpdateOrbitToFollowTransitionTimeCounter(Time.deltaTime);
            //UpdateOrbitToFollowInterpValue();
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
                    else if(orbitToFollowDelayCounter <= 0)
                    {
                        state = CameraState.FollowingTarget;
                        //state = CameraState.OrbitToFollow_Transitioning;
                    }
                    break;

                case CameraState.OrbitToFollow_Transitioning:
                    if (IsOrbiting())
                    {
                        state = CameraState.OrbitingTarget;
                    }
                    else if(IsCamOrientationNearEnoughDesiredFollow()) 
                    {
                        state = CameraState.FollowingTarget;
                    }
                    break;

                default:
                    break;
            }
        }

        public void UpdateTargetOrientationState()
        {
            switch(orientation)
            {
                case CameraTargetOrientationState.AwayFromCamera:
                    if(TargetIsMovingTowardsCamera() && TargetCanMoveTowardsCamera())
                    {
                        orientation = CameraTargetOrientationState.TowardsCamera;
                    }
                    break;

                case CameraTargetOrientationState.TowardsCamera:
                    if(!TargetIsMovingTowardsCamera() || !TargetCanMoveTowardsCamera())
                    {
                        orientation = CameraTargetOrientationState.AwayFromCamera;
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

        public CameraTargetOrientationState GetOrientationState()
        {
            return orientation;
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

        private bool TargetCanMoveTowardsCamera()
        {
            return components.cameraParams.allowMoveTowardsCamera;
        }

        private bool TargetIsMovingTowardsCamera()
        {
            Vector3 camForwardFlat = new Vector3(cam.transform.forward.x, 0f, cam.transform.forward.z).normalized;
            Vector3 followTargetForward = components.followTarget.transform.forward;
            Vector3 targetForwardFlat = new Vector3(followTargetForward.x, 0f, followTargetForward.z).normalized;
            return Mathf.Abs(Vector3.SignedAngle(camForwardFlat, targetForwardFlat, Vector3.up)) > (180 - components.cameraParams.movingTowardsCameraAngleRange);
        }

        private bool IsCamOrientationNearEnoughDesiredFollow()
        {
            CameraParams camParams = components.cameraParams;
            
            //get front or rear offset
            Vector3 desiredOffset = TargetIsMovingTowardsCamera() && TargetCanMoveTowardsCamera() ? camParams.desiredFrontOffset : camParams.desiredOffset;
            
            //transform offset from follow target local space, if necessary
            if (!camParams.useWorldSpaceOffset) desiredOffset = components.followTarget.transform.TransformDirection(desiredOffset);

            Vector3 followTargetToCam = cam.transform.position - components.followTarget.transform.position;
            return Vector3.Dot(desiredOffset.normalized, followTargetToCam.normalized) > offsetNearEnoughMinDot;
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