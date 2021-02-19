using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(SharedCameraComponents))]
    public class CommonLocomotionFunctions : MonoBehaviour
    {
        //components
        private Camera cam;
        private SharedCameraComponents components;

        //convenience variables
        GameObject followTarget;

        private void Awake()
        {
            cam = GetComponent<Camera>();
            components = GetComponent<SharedCameraComponents>();

            followTarget = components.followTarget;
        }

        public Vector3 ClampCameraMinDistance(float distance)
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.sqrMagnitude < distance * distance)
            {
                return followTarget.transform.position + (followTargetToCam.normalized * distance);
            }
            else
            {
                return cam.transform.position;
            }
        }

        /// <summary>Clamps the camera to its maximum distance from its target, as defined in the cam controller parameters.</summary>
        /// <returns>The distance-clamped camera position.</returns>
        public Vector3 ClampCameraMaxDistance(float distance)
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.sqrMagnitude > distance * distance)
            {
                return followTarget.transform.position + (followTargetToCam.normalized * distance);
            }
            else
            {
                return cam.transform.position;
            }
        }

        public Vector3 MoveCameraInterpolated(Vector3 desiredPos, float moveSpeed, float deltaTime)
        {
            return Lerps.Smootherstep(cam.transform.position, desiredPos, moveSpeed * deltaTime);
        }

        public Vector3 MoveCameraInterpolated(Vector3 desiredPos, float horizontalMoveSpeed, float verticalMoveSpeed, float deltaTime)
        {
            return Lerps.Smootherstep(cam.transform.position, desiredPos, horizontalMoveSpeed * deltaTime, verticalMoveSpeed * deltaTime);
        }
    }
}