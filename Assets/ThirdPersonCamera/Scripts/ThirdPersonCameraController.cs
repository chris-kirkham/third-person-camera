using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Lerps;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraControllerInput))]
    [RequireComponent(typeof(CameraStateController))]
    [RequireComponent(typeof(SharedCameraComponents))]
    public class ThirdPersonCameraController : MonoBehaviour
    {
        /* Components */
        #region components
        private Camera cam;
        private CameraControllerInput input;
        private CameraStateController stateController;
        private SharedCameraComponents components;

        //convenience references to components, updated at the start of each camera update
        private GameObject followTarget, lookAtTarget;
        private CameraParams camParams;
        #endregion

        #region member variables/trackers
        private float virtualCameraSphereRadius;
        
        private Vector2 currentOrbitAngles = Vector2.zero;
        private Vector2 orbitHoldAngle = Vector2.zero;
        private float timeInOcclusionMultiplier = 0f;
        private Vector3[] previousTargetPositions;
        #endregion

        private void Awake()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            stateController = GetComponent<CameraStateController>();
            components = GetComponent<SharedCameraComponents>();
            UpdateConvenienceComponentVars();

            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance();
        }

        private void Update()
        {
            if (camParams.updateFunction == CameraUpdateFunction.Update) UpdateCamera(Time.unscaledDeltaTime);    
        }

        private void LateUpdate()
        {
            if (camParams.updateFunction == CameraUpdateFunction.LateUpdate) UpdateCamera(Time.unscaledDeltaTime);
        }

        private void FixedUpdate()
        {
            if (camParams.updateFunction == CameraUpdateFunction.FixedUpdate) UpdateCamera(Time.fixedUnscaledDeltaTime);
        }

        /// <summary>
        /// Main camera update function, and the only one which moves the camera. Called in either Update, LateUpdate, or FixedUpdate, depending on setting.
        /// </summary>
        /// <param name="deltaTime">Time since the last camera update, in seconds.</param>
        private void UpdateCamera(float deltaTime)
        {
            /* Update relevant info */
            UpdateConvenienceComponentVars();
            UpdateStates();
            UpdateTrackers(deltaTime);

            /* Move camera */
            Vector3 initialClipPanePos = cam.GetNearClipPaneCentreWorld(); //cache camera's near clip pane centre before any movement for collision avoidance
            cam.transform.position = GetCameraMoveResult(stateController.GetCameraState(), stateController.GetOrientationState(), deltaTime);
            if (camParams.useMinDistance) cam.transform.position = ClampCameraMinDistance();
            if (camParams.useCamWhiskers) cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), deltaTime);
            if (camParams.avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionPullInResult(deltaTime);
            cam.transform.rotation = LookAtTarget(deltaTime);
            if (camParams.avoidCollisionWithGeometry) cam.transform.position = AvoidCollisions(initialClipPanePos, cam);
            if (camParams.useMaxDistance) cam.transform.position = ClampCameraMaxDistance();
        }

        #region general
        private Vector3 GetCameraMoveResult(CameraState state, CameraTargetOrientationState targetOrientation, float deltaTime)
        {
            switch (state)
            {
                case CameraState.FollowingTarget:
                    //return FollowTarget_DistanceRotation(followTarget.transform.TransformDirection(camParams.desiredOffset), camParams.followSpeed, camParams.frontFollowSpeed, deltaTime);
                    return FollowTarget(deltaTime);
                case CameraState.OrbitingTarget:
                    return OrbitTarget(currentOrbitAngles, deltaTime);
                case CameraState.OrbitToFollow_HoldingOrbitAngle:
                    return OrbitTarget(orbitHoldAngle, deltaTime);
                case CameraState.OrbitToFollow_Transitioning:
                    //TODO: Transition from orbit position to follow position
                    throw new System.NotImplementedException();
                default:
                    return cam.transform.position;
            }
        }

        //TODO: This produces jerky movement when running in circles, and causes collision avoidance to fail if you keep running towards the camera when it's backed into a wall
        private Vector3 ClampCameraMinDistance()
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.magnitude < camParams.minDistanceFromTarget)
            {
                return followTarget.transform.position + (followTargetToCam.normalized * camParams.minDistanceFromTarget);
            }
            else
            {
                return cam.transform.position;
            }
        }

        /// <summary>Clamps the camera to its maximum distance from its target, as defined in the cam controller parameters.</summary>
        /// <param name="cam">The current camera.</param>
        /// <returns>The distance-clamped camera position.</returns>
        private Vector3 ClampCameraMaxDistance()
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.magnitude > camParams.maxDistanceFromTarget)
            {
                return followTarget.transform.position + (followTargetToCam.normalized * camParams.maxDistanceFromTarget);
            }
            else
            {
                return cam.transform.position;
            }
        }

        /// <summary>Smoothly pulls the camera towards its follow target.</summary>
        /// <param name="camPos">The current camera position.</param>
        /// <param name="verticalSpeed">Speed to pull the camera to the target on the vertical (y) axis.</param>
        /// <param name="horizontalSpeed">Speed to pull the camera to the target on the horizontal (x, z) axes.</param>
        /// <param name="deltaTime">Time since the last camera update.</param>
        /// <returns>Camera position after moving towards target</returns>
        private Vector3 PullInCamera(Vector3 camPos, float verticalSpeed, float horizontalSpeed, float deltaTime)
        {
            Vector3 targetPos = followTarget.transform.position;
            float newX = Mathf.SmoothStep(camPos.x, targetPos.x, horizontalSpeed * deltaTime);
            float newY = Mathf.SmoothStep(camPos.y, targetPos.y, verticalSpeed * deltaTime);
            float newZ = Mathf.SmoothStep(camPos.z, targetPos.z, horizontalSpeed * deltaTime);

            return new Vector3(newX, newY, newZ);
        }

        /// <summary>
        /// Checks if the desired camera position would cause the camera to intersect with geometry to the rear;  
        /// if so, shortens the distance between the desired camera position and the target to avoid this  
        /// </summary>
        /// <param name="desiredPos">The desired camera position.</param>
        /// <returns>The closer camera position if an intersection was found, else the input position</returns>
        private Vector3 ShortenFollowDistanceToAvoidRearOcclusion(Vector3 desiredPos)
        {
            Vector3 desiredDir = desiredPos - followTarget.transform.position;
            if (Physics.SphereCast(followTarget.transform.position, virtualCameraSphereRadius, desiredDir, out RaycastHit hit, desiredDir.magnitude, camParams.occluderLayerMask))
            {
                return followTarget.transform.position + (desiredDir.normalized * hit.distance);
            }
            else
            {
                return desiredPos;
            }
        }

        private Vector3 MoveCameraInterpolated(Vector3 desiredPos, float moveSpeed, float deltaTime)
        {
            return Smootherstep(cam.transform.position, desiredPos, moveSpeed * deltaTime);
        }

        private Vector3 MoveCameraInterpolated(Vector3 desiredPos, float horizontalMoveSpeed, float verticalMoveSpeed, float deltaTime)
        {
            return Smoothstep(cam.transform.position, desiredPos, horizontalMoveSpeed * deltaTime, verticalMoveSpeed * deltaTime);
        }

        private void UpdateStates()
        {
            stateController.UpdateCameraState();
            stateController.UpdateTargetOrientationState();
        }

        private void UpdateTrackers(float deltaTime)
        {
            UpdateOcclusionTimeMultiplier(deltaTime);
            UpdateCurrentOrbitAngles();
            UpdateOrbitHoldAngles(stateController.GetCameraState());
            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance(); //update virtual size in case fov, clip pane distance etc. changed
        }
        #endregion              

        #region follow target
        private Vector3 FollowTarget_DistanceRotation(Vector3 desiredOffsetWorld, float rotationSpeed, float distanceFollowSpeed, float deltaTime)
        {
            //get shortened offset
            desiredOffsetWorld = ShortenFollowDistanceToAvoidRearOcclusion(followTarget.transform.position + desiredOffsetWorld) - followTarget.transform.position;

            //lerp between current and desired offsets with separate horizontal and vertical
            Vector3 currOffset = cam.transform.position - followTarget.transform.position;
            Vector3 currOffsetHorizontal = new Vector3(currOffset.x, 0, currOffset.z);
            float currOffsetY = currOffset.y;
            
            Vector3 desiredOffsetHorizontal = new Vector3(desiredOffsetWorld.x, 0, desiredOffsetWorld.z);
            float desiredOffsetY = desiredOffsetWorld.y;
            Vector3 lerpedOffsetHorizontal = Smoothstep(currOffsetHorizontal, desiredOffsetHorizontal, deltaTime * rotationSpeed);
            float lerpedOffsetY = Smoothstep(currOffsetY, desiredOffsetY, deltaTime * rotationSpeed);

            //lerp distance from target
            float currDist = currOffset.magnitude;
            float desiredDist = desiredOffsetWorld.magnitude;

            float lerpedDist = Smoothstep(currDist, desiredDist, deltaTime * distanceFollowSpeed);

            //get new cam position from lerped values
            Vector3 offset = new Vector3(lerpedOffsetHorizontal.x, lerpedOffsetY, lerpedOffsetHorizontal.z).normalized * lerpedDist;
            return followTarget.transform.position + offset;
        }

        private Vector3 FollowTarget(float deltaTime)
        {
            Vector3 desiredOffset;
            float followSpeed;
            if (stateController.GetOrientationState() == CameraTargetOrientationState.TowardsCamera && camParams.allowMoveTowardsCamera)
            {
                desiredOffset = camParams.desiredFrontOffset;
                followSpeed = camParams.frontFollowSpeed;
            }
            else //moving away from camera
            {
                desiredOffset = camParams.desiredOffset;
                followSpeed = camParams.followSpeed;
            }

            if(camParams.followHeightMode == FollowHeightMode.AboveGround) desiredOffset.y = GetDesiredHeightAboveGroundOffset();
            if(camParams.avoidFollowTargetOcclusion) followSpeed += GetOcclusionFollowSpeedIncrease();

            return MoveCameraInterpolated(ShortenFollowDistanceToAvoidRearOcclusion(GetDesiredFollowPosition(desiredOffset)), followSpeed, deltaTime);
        }

        private Vector3 GetDesiredFollowPosition(Vector3 desiredOffset)
        {
            if (camParams.useWorldSpaceOffset)
            {
                return followTarget.transform.position + desiredOffset;
            }
            else
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
        private float GetDesiredHeightAboveGroundOffset()
        {
            if(Physics.SphereCast(cam.transform.position, virtualCameraSphereRadius, Vector3.down, out RaycastHit hit, camParams.maxGroundDistance, camParams.groundLayerMask) //if ground found
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
        #endregion

        #region collision avoidance
        /// <summary>Avoids camera clipping with geometry by jumping the camera towards the follow target if a collision is detected.</summary>
        /// <param name="initialNearClipPanePos">The camera's near clip pane centre point at the start of the update tick.</param>
        /// <param name="updatedCam">The camera after other movement has been applied.</param>
        /// <returns>The new camera position after collision avoidance has been applied.</returns>
        private Vector3 AvoidCollisions(Vector3 initialNearClipPanePos, Camera updatedCam)
        {
            //!!!!
            //BIG TODO: This function fails when the sphere cast checks' (both the one in GetCollidersBetweenCameraPositions and the inner sphere checks that begin from followTargetPos)
            //have their starting sphere positions overlap geometry (see link below). This DOES HAPPEN when the camera's field of view is very wide and/or the near clip pane distance is large.
            //See this link for a possible solution (extending the start position 1 virtual size backwards), or consider using a BoxCast 
            //Description of sphere cast issue - https://forum.unity.com/threads/spherecastall-returns-0-0-0-for-all-raycasthit-points.428302/
            //Possible solution - https://forum.unity.com/threads/analyzing-and-optimizing-unity-sphere-capsule-casts.233328/
            //!!!!

            Vector3 followTargetPos = followTarget.transform.position;

            //Check if the path between the camera's initial position this tick and its updated position would cause it to intersect with any geometry
            //If so, cast a sphere back from follow target to camera's near clip pane, find the closest hit collider -which is also intersecting the camera's path-
            //to the follow target, and move the camera in front of that.
            HashSet<Collider> camColliders = GetCollidersBetweenCameraPositions(initialNearClipPanePos, updatedCam.GetNearClipPaneCentreWorld());
            if (camColliders.Count > 0) 
            {
                foreach (Collider c in camColliders) Debug.DrawLine(updatedCam.GetNearClipPaneCentreWorld(), c.transform.position, Color.yellow);
                
                Vector3 followTargetToNearClip = updatedCam.GetNearClipPaneCentreWorld() - followTargetPos;
                float minDistance = Mathf.Infinity;
                RaycastHit[] hits = Physics.SphereCastAll(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, followTargetToNearClip.magnitude, camParams.colliderLayerMask);
                foreach(RaycastHit hit in hits)
                {
                    if (camColliders.Contains(hit.collider) && hit.distance < minDistance) 
                    {
                        //note: the hit.distance of a RaycastHit from a sphere cast is the distance to the centre of the sphere that hit something,
                        //not the distance to the hit point: https://answers.unity.com/questions/882631/how-to-get-the-center-of-a-capsulecast.html
                        minDistance = hit.distance;
                        Debug.DrawLine(followTargetPos, hit.point, Color.red);
                    }
                }

                if(minDistance != Mathf.Infinity)
                {
                    Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * minDistance);
                    Debug.DrawLine(hitSphereCentre, hitSphereCentre - (updatedCam.transform.forward * updatedCam.nearClipPlane), Color.green);
                    return hitSphereCentre - (updatedCam.transform.forward * updatedCam.nearClipPlane); //move the camera's near clip pane centre to the centre of the hit sphere
                }
            }

            return updatedCam.transform.position;
        }

        ///<summary>
        ///Returns colliders between two camera positions as a HashSet, using the camera's virtual size to sphere cast.
        ///Intended use is to detect collisions by casting between the camera near clip position at the start of the update tick and its position after all movement
        ///except collision avoidance has been applied.
        ///</summary>
        ///<returns>All colliders hit by the sphere cast.</returns>
        private HashSet<Collider> GetCollidersBetweenCameraPositions(Vector3 initialNearClipPane, Vector3 updatedNearClipPane)
        {
            //debug drawing
            //Debug.DrawRay(initialNearClipPane, Vector3.up * virtualCameraSphereRadius, Color.blue);
            //Debug.DrawRay(updatedNearClipPane, Vector3.down * virtualCameraSphereRadius, Color.cyan);

            Vector3 initialToUpdatedNearClip = updatedNearClipPane - initialNearClipPane;
            RaycastHit[] hits = Physics.SphereCastAll(initialNearClipPane, virtualCameraSphereRadius, initialToUpdatedNearClip, initialToUpdatedNearClip.magnitude, camParams.colliderLayerMask);

            HashSet<Collider> colliders = new HashSet<Collider>();
            foreach (RaycastHit hit in hits) colliders.Add(hit.collider);
            return colliders;
        }
        #endregion

        #region cam whiskers
        //Casts "whisker" rays from the follow target
        private Vector3 GetCamWhiskerOffset()
        {
            Vector3 whiskerPushDir = Vector3.zero;
            Vector3 followTargetPos = followTarget.transform.position;
            float halfWhiskerSectorAngle = camParams.whiskerSectorAngle / 2;
            float angleInc = halfWhiskerSectorAngle / camParams.numWhiskers * 2;

            for (float angle = -halfWhiskerSectorAngle; angle <= halfWhiskerSectorAngle; angle += angleInc)
            {
                Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * -followTarget.transform.forward;
                Debug.DrawRay(followTargetPos, dir * camParams.whiskerLength, Color.HSVToRGB(Mathf.Abs(angle) / camParams.whiskerSectorAngle, 0.2f, 1f)); //visualise raycasts
                if (Physics.Raycast(followTargetPos, dir, camParams.whiskerLength)) whiskerPushDir -= new Vector3(dir.x, 0f, dir.z);
            }

            //zero local z (is there a better way to do this?)
            whiskerPushDir = cam.transform.InverseTransformDirection(whiskerPushDir);
            whiskerPushDir.z = 0;
            whiskerPushDir = cam.transform.TransformDirection(whiskerPushDir);
            Debug.DrawRay(followTargetPos, whiskerPushDir, Color.yellow);

            return whiskerPushDir / camParams.numWhiskers;
        }

        private Vector3 GetCamWhiskerResult(Vector3 camPos, Vector3 whiskerOffset, float deltaTime)
        {
            return Smoothstep(camPos, camPos + whiskerOffset, camParams.whiskerPushStrength * deltaTime);
        }
        #endregion

        #region occlusion
        /// <summary>Checks whether the camera's view of the target is occluded by level geometry by casting rays towards the target from each corner of the camera's near clip pane.</summary>
        /// <param name="clipPanePadding">Adds virtual padding to the camera's near clip pane size when casting rays. 0: actual near clip pane size; >0: increased clip pane size</param>
        /// <returns>True if the follow target is occluded by geometry on the occluder layer(s)</returns>
        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.transform.position;

            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, camParams.occluderLayerMask, clipPanePadding);
        }

        /// <summary> Pulls the camera towards the follow target if the follow target is occluded by geometry.</summary>
        /// <param name="deltaTime">Time since the last camera update.</param>
        /// <returns>The closer camera position if occluded, the current camera position if not.</returns>
        private Vector3 GetOcclusionPullInResult(float deltaTime)
        {
            Vector3 camPos = cam.transform.position;

            if (camParams.useTimeInOcclusionMultiplier)
            {
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical * timeInOcclusionMultiplier, camParams.occlusionPullInSpeedHorizontal * timeInOcclusionMultiplier, deltaTime);
            }
            else if(IsOccluded())
            {
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical, camParams.occlusionPullInSpeedHorizontal, deltaTime);
            }

            return camPos;
        }

        /// <summary>
        /// Gets the occlusion follow speed increase, taking into account the time-in-occlusion multiplier, if using.
        /// </summary>
        /// <returns>The occlusion follow speed increase.</returns>
        private float GetOcclusionFollowSpeedIncrease()
        {
            if(camParams.useTimeInOcclusionMultiplier)
            {
                return camParams.occlusionFollowSpeedIncrease * timeInOcclusionMultiplier;
            }
            else if(IsOccluded())
            {
                return camParams.occlusionFollowSpeedIncrease;
            }

            return 0f;
        }

        /// <summary>
        /// Updates the time-in-occlusion speed increase ramp variable. Note that this value is not the actual time spent in occlusion;
        /// it represents a ramp up/down value which increases up to a maximum the longer the target is occluded, and begins decreasing when the target
        /// is no longer occluded. 
        /// </summary>
        /// <param name="deltaTime">Time since the last update tick.</param>
        private void UpdateOcclusionTimeMultiplier(float deltaTime)
        {
            if (camParams.useTimeInOcclusionMultiplier)
            {
                if (IsOccluded(camParams.occlusionClipPanePadding))
                {
                    timeInOcclusionMultiplier = Mathf.Min(timeInOcclusionMultiplier + (deltaTime * camParams.timeInOcclusionRampUpSpeed), camParams.maxTimeInOcclusionMultiplier);
                }
                else
                {
                    timeInOcclusionMultiplier = Mathf.Max(timeInOcclusionMultiplier - (deltaTime * camParams.timeInOcclusionRampDownSpeed), 0f);
                }
            }
            else
            {
                timeInOcclusionMultiplier = 0f;
            }

            Debug.Log("timeInOcclusionMultiplier = " + timeInOcclusionMultiplier);
        }
        #endregion

        #region orbit
        private Vector3 OrbitTarget(Vector3 orbitAngles, float deltaTime)
        {
            Vector3 desiredPos = ShortenFollowDistanceToAvoidRearOcclusion(GetDesiredOrbitPositionFromAngles(orbitAngles));
            return MoveCameraInterpolated(desiredPos, camParams.orbitSpeed, deltaTime);
        }

        /// <summary>Calculates new orbit angles based on the current angles and the orbit input.</summary>
        private Vector2 GetNewOrbitAngles(Vector2 currAngles, Vector2 orbitInput)
        {
            return new Vector2(Mathf.Clamp(currAngles.x + orbitInput.x, camParams.minOrbitYAngle, camParams.maxOrbitYAngle), Clamp360(currAngles.y + orbitInput.y));
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
            
            if(angle >= 360)
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

        /// <summary>Finds the camera's desired orbit position from its orbit angles.</summary>
        private Vector3 GetDesiredOrbitPositionFromAngles(Vector2 orbitAngles)
        {
            Quaternion rotation = Quaternion.Euler(orbitAngles.x, orbitAngles.y, 0f);
            Debug.DrawLine(followTarget.transform.position + (rotation * camParams.desiredOffset), followTarget.transform.position + (Vector3.up * camParams.desiredOffset.y), Color.white);
            return followTarget.transform.position + (rotation * camParams.desiredOffset);
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

        /// <summary> caches the current orbit angle for orbit-to-follow transition hold</summary>
        private void UpdateOrbitHoldAngles(CameraState state)
        {
            if(state != CameraState.OrbitToFollow_HoldingOrbitAngle) orbitHoldAngle = currentOrbitAngles;
        }
        #endregion

        #region look at target
        /// <summary>
        /// Finds the rotation which will make the camera look at the look target (+ desired offset, if any)
        /// </summary>
        /// <returns>The desired look-at-target rotation</returns>
        private Quaternion GetDesiredLookAtTargetRotation()
        {
            Transform lookAtTransform = lookAtTarget.transform;
            return Quaternion.LookRotation((lookAtTransform.position + new Vector3(lookAtTransform.right.x, 0f, lookAtTransform.right.z).normalized * camParams.lookOffset) - cam.transform.position);
        }

        /// <summary>
        /// Calculates the camera's new look-at rotation.
        /// </summary>
        /// <param name="deltaTime">Time since the last camera update</param>
        /// <returns>The camera's new look-at-target rotation. If this is not interpolated, it will be the same as the desired look-at rotation</returns>
        private Quaternion LookAtTarget(float deltaTime)
        {
            if (camParams.interpolateTargetLookAt)
            {
                return Quaternion.Lerp(cam.transform.rotation, GetDesiredLookAtTargetRotation(), deltaTime * camParams.targetLookAtLerpSpeed);
            }
            else
            {
                return GetDesiredLookAtTargetRotation();
            }
        }
        #endregion

        #region utility
        /// <summary>
        /// Updates convenience references to the camera controller's follow/look targets and its parameter object
        /// </summary>
        private void UpdateConvenienceComponentVars()
        {
            followTarget = components.followTarget;
            lookAtTarget = components.lookAtTarget;
            camParams = components.cameraParams;
        }
        #endregion

        private void OnDrawGizmos()
        {
            if(cam != null) //cam will be null unless the game is running
            {
                //draw camera virtual size
                Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
                Gizmos.DrawWireSphere(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius);
            }
        }
    }
}