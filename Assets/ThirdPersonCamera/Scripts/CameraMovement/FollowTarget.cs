using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraStateController))]
    [RequireComponent(typeof(SharedCameraComponents))]
    [RequireComponent(typeof(OcclusionAvoidance))]
    [RequireComponent(typeof(CommonLocomotionFunctions))]
    public class FollowTarget : MonoBehaviour
    {
        //components
        private Camera cam;
        private CameraStateController stateController;
        private SharedCameraComponents components;
        private DebugDrawing debugDrawing;
        private OcclusionAvoidance occlusionAvoidance;
        private CommonLocomotionFunctions locomotion;

        //convenience variables
        private GameObject followTarget;
        private CameraParams camParams;

        private void Awake()
        {
            cam = GetComponent<Camera>();
            stateController = GetComponent<CameraStateController>();
            components = GetComponent<SharedCameraComponents>();
            debugDrawing = GetComponent<DebugDrawing>();
            occlusionAvoidance = GetComponent<OcclusionAvoidance>();
            locomotion = GetComponent<CommonLocomotionFunctions>();

            followTarget = components.followTarget;
            camParams = components.cameraParams;
        }

        public Vector3 GetTargetFollowPosition(float deltaTime, float virtualCameraSphereRadius)
        {
            //get desired offset based on whether target is moving towards or away from camera. If we add more camera-target orientation states,
            //will need to do this a better way
            Vector3 desiredOffset;
            float followSpeedHorz;
            float followSpeedVert;
            float followSpeedDistance;
            if (stateController.GetOrientationState() == CameraTargetOrientationState.TowardsCamera && camParams.allowMoveTowardsCamera)
            {
                desiredOffset = camParams.desiredFrontOffset;
                followSpeedHorz = camParams.frontFollowSpeedOrientation;
                followSpeedVert = camParams.frontFollowSpeedHeight;
                followSpeedDistance = camParams.frontFollowSpeedDistance;
            }
            else //moving away from camera
            {
                desiredOffset = camParams.desiredOffset;
                followSpeedHorz = camParams.followSpeedOrientation;
                followSpeedVert = camParams.followSpeedHeight;
                followSpeedDistance = camParams.followSpeedDistance;
            }

            //get desired height offset based on follow height mode
            if (camParams.followHeightMode == FollowHeightMode.AboveGround) desiredOffset.y = GetDesiredHeightAboveGroundOffset(virtualCameraSphereRadius);

            //get desired position from offset
            Vector3 desiredPos = GetDesiredFollowPositionWorld(desiredOffset);

            //shorten follow distance if necessary
            desiredPos = occlusionAvoidance.ShortenFollowDistanceToAvoidRearOcclusion(desiredPos, virtualCameraSphereRadius);

            //increase follow speed if avoiding occlusion
            if (camParams.avoidFollowTargetOcclusion)
            {
                float occlusionSpeedIncrease = occlusionAvoidance.GetOcclusionFollowSpeedIncrease();
                followSpeedHorz += occlusionSpeedIncrease;
                followSpeedVert += occlusionSpeedIncrease;
            }
            //interpolate between current and desired positions
            Vector3 newCamPos = locomotion.MoveCameraInterpolated(desiredPos, followSpeedHorz, followSpeedVert, deltaTime);

            //interpolate distance from target separately 
            float desiredDistance = desiredOffset.magnitude;
            Vector3 newOffset = newCamPos - followTarget.transform.position; //convert new cam position back to a world space offset from follow target
            float newDistance = Lerps.Smootherstep(newOffset.magnitude, desiredDistance, followSpeedDistance * deltaTime); //interpolate between current offset distance and desired distance
            return followTarget.transform.position + (newOffset.normalized * newDistance); //scale normalised offset by new distance and convert back to world position
        }

        /// <summary>Returns the desired world space follow position of the camera from the input offset, transforming it to world space if using local offset</summary>
        private Vector3 GetDesiredFollowPositionWorld(Vector3 desiredOffset)
        {
            if (camParams.useWorldSpaceOffset)
            {
                return followTarget.transform.position + desiredOffset;
            }
            else //offset is in target's local space; transform back to world space
            {
                return followTarget.transform.position + followTarget.transform.TransformDirection(desiredOffset);
            }
        }

        /// <summary>
        /// Finds the camera's desired height above the ground directly below the camera. Returns the cam params' fallback height if no valid ground found.
        /// </summary>
        /// <returns>
        /// The desired height above the ground, or the fallback height if no valid ground found. Either way, the value is returned as an offset from the target's height.
        /// </returns>
        private float GetDesiredHeightAboveGroundOffset(float virtualCameraSphereRadius)
        {
            if (Physics.SphereCast(cam.transform.position, virtualCameraSphereRadius, Vector3.down, out RaycastHit hit, camParams.maxGroundDistance, camParams.groundLayerMask) //if ground found
                && Vector3.Angle(Vector3.up, hit.normal) <= camParams.maxGroundSlopeAngle) //if ground angle is valid
            {
                float heightAboveGround = hit.point.y + camParams.desiredHeightAboveGround;
                return heightAboveGround - followTarget.transform.position.y;
            }
            else
            {
                return camParams.aboveGroundFallbackHeight;
            }
        }
    }
}