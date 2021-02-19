using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraControllerInput))]
    [RequireComponent(typeof(CameraStateController))]
    [RequireComponent(typeof(SharedCameraComponents))]
    [RequireComponent(typeof(OcclusionAvoidance))]
    public class OrbitTarget : MonoBehaviour
    {
        //components
        private Camera cam;
        private CameraControllerInput input;
        private CameraStateController stateController;
        private SharedCameraComponents components;
        private DebugDrawing debugDrawing;
        private OcclusionAvoidance occlusionAvoidance;
        private CommonLocomotionFunctions locomotion;

        //trackers
        private Vector2 currentOrbitAngles = Vector2.zero;
        private Vector2 orbitHoldAngle = Vector2.zero;

        //convenience variables
        private GameObject followTarget;
        private CameraParams camParams;

        private void Awake()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            stateController = GetComponent<CameraStateController>();
            components = GetComponent<SharedCameraComponents>();
            debugDrawing = GetComponent<DebugDrawing>();
            occlusionAvoidance = GetComponent<OcclusionAvoidance>();
            locomotion = GetComponent<CommonLocomotionFunctions>();

            followTarget = components.followTarget;
            camParams = components.cameraParams;
        }

        private void Update()
        {
            UpdateCurrentOrbitAngles();
            UpdateOrbitHoldAngles(stateController.GetCameraState());
        }

        /// <summary>Gets the new (interpolated) camera orbit position.</summary>
        /// <param name="orbitAngles">Angle to orbit</param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        public Vector3 GetOrbitPosition(Vector3 orbitAngles, float deltaTime)
        {
            //get new camera position (this can/will change the distance from the target, but it is interpolated seperately next anyway)
            Vector3 desiredPos = GetDesiredOrbitPositionFromAngles(orbitAngles);
            Vector3 newCamPos = locomotion.MoveCameraInterpolated(desiredPos, camParams.orbitSpeed, deltaTime);

            //interpolate distance from target separately 
            Vector3 newOffset = newCamPos - followTarget.transform.position; //convert new cam position back to a world space offset from follow target
            float newDistance = Lerps.Smootherstep((cam.transform.position - followTarget.transform.position).magnitude, camParams.desiredOrbitDistance, camParams.orbitSpeedDistance * deltaTime); //interpolate between current offset distance and desired distance
            return followTarget.transform.position + newOffset.normalized * newDistance; //scale normalised offset by new distance and convert back to world position
        }

        /// <summary>Finds the camera's desired orbit position from its orbit angles and distance.</summary>
        private Vector3 GetDesiredOrbitPositionFromAngles(Vector2 orbitAngles)
        {
            Quaternion rotation = Quaternion.Euler(orbitAngles.x, orbitAngles.y, 0f);
            return followTarget.transform.position + rotation * (Vector3.back * camParams.desiredOrbitDistance);
        }

        /// <summary>Clamps an angle to within 360 degrees.</summary>
        private float Clamp360(float angle)
        {
            if (angle < 0)
            {
                do
                {
                    angle += 360;
                }
                while (angle < 0);

                return angle;
            }

            if (angle >= 360)
            {
                do
                {
                    angle -= 360;
                }
                while (angle >= 360);

                return angle;
            }

            return angle;
        }

        private Vector2 GetNewOrbitAngles(Vector2 currAngles, Vector2 orbitInput)
        {
            return new Vector2(Mathf.Clamp(currAngles.x + orbitInput.x, camParams.minOrbitYAngle, camParams.maxOrbitYAngle), Clamp360(currAngles.y + orbitInput.y));
        }

        /// <summary>Updates the current orbit angles tracker based on the current camera state.</summary>
        private void UpdateCurrentOrbitAngles()
        {
            if (stateController.GetCameraState() == CameraState.OrbitingTarget)
            {
                currentOrbitAngles = GetNewOrbitAngles(currentOrbitAngles, input.GetOrbitInput());
            }
            else
            {
                Vector2 euler = (Vector2)cam.transform.eulerAngles;
                if (euler.x > 180) euler.x -= 360; //make angles below horizontal negative so it's in line with orbit angles
                currentOrbitAngles = euler;
            }
        }

        /// <summary>Calculates new orbit angles based on the current angles and the orbit input.</summary>
        

        /// <summary> caches the current orbit angle for orbit-to-follow transition hold</summary>
        private void UpdateOrbitHoldAngles(CameraState state)
        {
            if (state != CameraState.OrbitToFollow_HoldingOrbitAngle) orbitHoldAngle = currentOrbitAngles;
        }
    }
}