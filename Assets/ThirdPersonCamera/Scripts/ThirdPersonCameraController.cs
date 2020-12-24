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

        /* Private variables */
        private float virtualCameraSphereRadius;
        
        #region counters/trackers
        private Vector2 currentOrbitAngles = Vector2.zero;
        private float timeInOcclusionRamp = 0f;
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

        //all camera movement is done here
        private void FixedUpdate()
        {
            UpdateConvenienceComponentVars();
            UpdateOcclusionTimeRamp(Time.fixedDeltaTime);
            UpdateCurrentOrbitAngles();
            //Debug.Log("current orbit angles = " + currentOrbitAngles);
            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance(); //update virtual size in case fov, clip pane distance etc. changed
            stateController.UpdateCameraState();


            Vector3 initialClipPanePos = cam.GetNearClipPaneCentreWorld(); //cache camera's near clip pane centre before any movement for collision avoidance

            switch (stateController.GetCameraState())
            {
                case CameraState.FollowingTarget:
                    cam.transform.position = GetFollowPosition(GetDesiredFollowPosition(camParams.desiredOffset), camParams.followSpeed + GetOcclusionFollowSpeedIncrease(), Time.fixedDeltaTime);
                    break;
                case CameraState.TargetMovingTowardsCamera:
                    cam.transform.position = GetFollowPosition(GetDesiredFollowPosition(camParams.desiredFrontOffset), camParams.frontFollowSpeed + GetOcclusionFollowSpeedIncrease(), Time.fixedDeltaTime);
                    break;
                case CameraState.OrbitingTarget:
                    cam.transform.position = GetNewOrbitPosition(GetDesiredOrbitPositionFromAngles(currentOrbitAngles), Time.fixedDeltaTime);
                    break;
                case CameraState.OrbitToFollow_HoldingOrbitAngle:
                    //TODO: Hold orbit angle
                    break;
                case CameraState.OrbitToFollow_Transitioning:
                    //TODO: Transition from orbit position to follow position
                    cam.transform.position = GetDesiredTransitionFromOrbitToFollowPosition();
                    break;
            }

            if (camParams.useCamWhiskers) cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime);
            if (camParams.avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionPullInResult(Time.fixedDeltaTime);
            cam.transform.rotation = LookAtTarget(cam, Time.fixedDeltaTime);
            if (camParams.avoidCollisionWithGeometry) cam.transform.position = AvoidCollisions(initialClipPanePos, cam);
            if (camParams.useMaxDistance) cam.transform.position = ClampCameraMaxDistance(cam);
        }

        
        #region follow target
        ///<summary>
        ///Returns the camera position after interpolating between the current camera position and the desired follow position, 
        ///shortening the desired follow offset if it would be occluded. This is the same for both rear follow and front follow ("target moving towards camera" state, if using)
        ///</summary>
        ///<param name="state">The current camera state.</param>
        ///<param name="deltaTime">Time since the last camera update.</param>
        ///<returns>The camera's new follow position.</returns>
        private Vector3 GetFollowPosition(Vector3 desiredPos, float followSpeed, float deltaTime)
        {
            Debug.DrawLine(cam.transform.position, desiredPos, Color.red);

            Vector3 newCamPos = camParams.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos;
            Debug.DrawLine(cam.transform.position, newCamPos, Color.yellow);

            if(camParams.interpolateTargetFollow)
            {
                return Smoothstep(cam.transform.position, newCamPos, deltaTime * followSpeed);
            }
            else
            {
                return newCamPos;
            }
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
        /// Returns the camera's follow speed multiplier
        /// </summary>
        /// <returns>Speed multiplier for target follow interpolation</returns>
        private float GetFollowSpeed()
        {
            if (camParams.occlusionIncreaseFollowSpeedMultiplier > 0)
            {
                if(camParams.useTimeInOcclusionMultiplier && timeInOcclusionRamp > 0)
                {
                    return camParams.followSpeed * Mathf.Max(1, camParams.occlusionIncreaseFollowSpeedMultiplier * timeInOcclusionRamp);
                }
                else
                {
                    return camParams.followSpeed * Mathf.Max(1, camParams.occlusionIncreaseFollowSpeedMultiplier);
                }
            }
            else
            {
                return camParams.followSpeed;
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
            
            //Check if the camera's virtual size sphere is intersecting with geometry
            HashSet<Collider> overlapSphereColliders = GetCollidersBetweenCameraPositions(initialNearClipPanePos, updatedCam.GetNearClipPaneCentreWorld());
            if (overlapSphereColliders.Count > 0) //If so, cast a sphere back from follow target to camera's near clip pane
            {
                foreach (Collider c in overlapSphereColliders) Debug.DrawLine(updatedCam.GetNearClipPaneCentreWorld(), c.transform.position, Color.yellow);
                
                Vector3 avoidPositions = Vector3.zero;
                int colliderSphereHits = 0;
                Vector3 followTargetToNearClip = updatedCam.GetNearClipPaneCentreWorld() - followTargetPos;
                RaycastHit[] hits = Physics.SphereCastAll(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, followTargetToNearClip.magnitude, camParams.colliderLayerMask);

                foreach (RaycastHit hit in hits) //for each sphere hit, check if the hit is on a collider in our OverlapSphere colliders list; adjust camera position based on hit if so
                {
                    if (overlapSphereColliders.Contains(hit.collider)) 
                    {
                        Debug.Log("Hit collider: " + hit.collider);
                        Debug.DrawLine(followTargetPos, hit.point, Color.red);

                        //note: the hit.distance of a RaycastHit from a sphere cast is the distance to the centre of the sphere that hit something,
                        //not the distance to the hit point: https://answers.unity.com/questions/882631/how-to-get-the-center-of-a-capsulecast.html
                        Vector3 hitSphereCentre = followTargetPos + followTargetToNearClip.normalized * hit.distance;
                        avoidPositions += hitSphereCentre - updatedCam.transform.forward * updatedCam.nearClipPlane;
                        Debug.DrawLine(hitSphereCentre, hitSphereCentre - updatedCam.transform.forward * updatedCam.nearClipPlane, Color.green);
                        colliderSphereHits++;
                    }
                }

                if (colliderSphereHits > 0)
                {
                    return avoidPositions / colliderSphereHits; //return average of avoid vectors
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
            Debug.DrawRay(initialNearClipPane, Vector3.up * virtualCameraSphereRadius, Color.blue);
            Debug.DrawRay(updatedNearClipPane, Vector3.down * virtualCameraSphereRadius, Color.cyan);

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
        /// <summary> Pulls the camera towards the follow target if the follow target is occluded by geometry.</summary>
        /// <param name="deltaTime">Time since the last camera update.</param>
        /// <returns>The closer camera position if occluded, the current camera position if not.</returns>
        private Vector3 GetOcclusionPullInResult(float deltaTime)
        {
            Vector3 camPos = cam.transform.position;
            float maxDistance = Vector3.Distance(followTarget.transform.position, camPos);

            if (IsOccluded(camParams.occlusionClipPanePadding))
            {
                Debug.DrawRay(camPos, (followTarget.transform.position - camPos).normalized * maxDistance, Color.cyan);
                float timeInOcclusionMult = GetTimeInOcclusionMultiplier();
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical * timeInOcclusionMult, camParams.occlusionPullInSpeedHorizontal * timeInOcclusionMult, deltaTime);
            }

            return camPos;
        }

        /// <summary>
        /// Gets the follow speed increase for when the target is occluded, if using it. Also takes into account time in occlusion multiplier, if using.
        /// </summary>
        /// <returns>The occlusion follow speed increase (potentially multiplier by the time-in-occlusion ramp); 0 if target is not occluded.</returns>
        private float GetOcclusionFollowSpeedIncrease()
        {
            if (IsOccluded(camParams.occlusionClipPanePadding))
            {
                return camParams.occlusionIncreaseFollowSpeedMultiplier * GetTimeInOcclusionMultiplier();
            }
            else
            {
                return 0;
            }
        }



        /// <summary>Checks whether the camera's view of the target is occluded by level geometry by casting rays towards the target from each corner of the camera's near clip pane.</summary>
        /// <param name="clipPanePadding">Adds virtual padding to the camera's near clip pane size when casting rays. 0: actual near clip pane size; >0: increased clip pane size</param>
        /// <returns>True if the follow target is occluded by geometry on the occluder layer(s)</returns>
        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.transform.position;

            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, camParams.occluderLayerMask, clipPanePadding);
        }

        /// <summary>Gets the occlusion speed multiplier for time-in-occlusion speed ramp. If not using, returns 1. </summary>
        private float GetTimeInOcclusionMultiplier()
        {
            return camParams.useTimeInOcclusionMultiplier ? timeInOcclusionRamp : 1f;
        }

        /// <summary>
        /// Updates the time-in-occlusion speed increase ramp variable. Note that this value is not the actual time spent in occlusion;
        /// it represents a ramp up/down value which increases up to a maximum the longer the target is occluded, and begins decreasing when the target
        /// is no longer occluded. For this reason, it never goes below 1 (since we don't want the follow speed to decrease when out of occlusion)
        /// and never goes above the max value set in the camera params.
        /// </summary>
        /// <param name="deltaTime">Time since the last update tick.</param>
        private void UpdateOcclusionTimeRamp(float deltaTime)
        {
            if (camParams.useTimeInOcclusionMultiplier)
            {
                if (IsOccluded(camParams.occlusionClipPanePadding))
                {
                    timeInOcclusionRamp = Mathf.Min(timeInOcclusionRamp + deltaTime * camParams.timeInOcclusionRampUpSpeed, camParams.maxTimeInOcclusionMultiplier);
                }
                else
                {
                    timeInOcclusionRamp = Mathf.Max(timeInOcclusionRamp - deltaTime * camParams.timeInOcclusionRampDownSpeed, 1f);
                }
            }
            else
            {
                timeInOcclusionRamp = 1;
            }
            
        }
        #endregion

        #region orbit
        /// <summary>Calculates new orbit angles based on the current angles and the orbit input.</summary>
        private Vector2 GetNewOrbitAngles(Vector2 currAngles, Vector2 orbitInput)
        {
            return new Vector2(ClampOrbitAngle360(currAngles.x + orbitInput.x), Mathf.Clamp(currAngles.y + orbitInput.y, camParams.minOrbitYAngle, camParams.maxOrbitYAngle));
        }

        /// <summary>Clamps an angle to within 360 degrees.</summary>
        private float ClampOrbitAngle360(float angle)
        {
            if (angle < -360) return angle + 360;
            if (angle >= 360) return angle - 360;
            return angle;
        }

        /// <summary>Finds the camera's desired orbit position from its orbit angles.</summary>
        private Vector3 GetDesiredOrbitPositionFromAngles(Vector2 orbitAngles)
        {
            Quaternion rotation = Quaternion.Euler(orbitAngles.y, orbitAngles.x, 0f);
            Debug.DrawLine(followTarget.transform.position + rotation * camParams.desiredOffset, followTarget.transform.position + Vector3.up * camParams.desiredOffset.y, Color.white);
            return followTarget.transform.position + rotation * camParams.desiredOffset;
        }

        /// <summary>Interpolates between the camera's current position and the desired orbit position to find the camera's new orbit position.</summary>
        /// <param name="desiredPos">The desired orbit position</param>
        /// <param name="deltaTime">Time since the last camera update</param>
        /// <returns>The camera's new orbit position.</returns>
        private Vector3 GetNewOrbitPosition(Vector3 desiredPos, float deltaTime)
        {
            return Vector3.Lerp(cam.transform.position, camParams.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos, deltaTime * camParams.orbitSpeed);
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
                currentOrbitAngles = new Vector2(cam.transform.eulerAngles.y, cam.transform.eulerAngles.x);
            }
        }
        #endregion

        #region transition from orbit to follow
        private Vector3 GetDesiredOrbitToFollowHoldPosition()
        {
            throw new System.NotImplementedException();
        }

        private Vector3 GetDesiredTransitionFromOrbitToFollowPosition()
        {
            //TODO: check for moving towards camera here, so it can smoothly transition to that? (instead of just jumping out of the transition state, as it currently does)
            Vector3 desiredFollowPos = GetDesiredCamPosition(CameraState.FollowingTarget);

            float transitionTime = camParams.orbitToFollowTransitionTime;
            float currTime = stateController.GetOrbitToFollowTransitionTimeCounter();
            return Smoothstep(cam.transform.position, desiredFollowPos, (transitionTime - currTime) / transitionTime);
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
        /// <param name="cam">The current camera</param>
        /// <param name="deltaTime">Time since the last camera update</param>
        /// <returns>The camera's new look-at-target rotation. If this is not interpolated, it will be the same as the desired look-at rotation</returns>
        private Quaternion LookAtTarget(Camera cam, float deltaTime)
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

        #region general
        /// <summary>
        /// Returns the desired camera position based on the camera state.
        /// This is the "ideal" position the camera wants to be in, before taking into consideration obstacle/occlusion avoidance etc. 
        /// </summary>
        /// <param name="state">The current camera state.</param>
        /// <returns>The desired camera position based on the input camera state</returns>
        private Vector3 GetDesiredCamPosition(CameraState state)
        {
            switch (state)
            {
                case CameraState.OrbitingTarget:
                    return GetDesiredOrbitPositionFromAngles(currentOrbitAngles);
                case CameraState.TargetMovingTowardsCamera:
                    return GetDesiredFollowPosition(camParams.desiredFrontOffset);
                case CameraState.OrbitToFollow_HoldingOrbitAngle:
                    return GetDesiredOrbitPositionFromAngles(currentOrbitAngles); //hold orbit angles - state system won't allow updating them in this state
                case CameraState.OrbitToFollow_Transitioning:
                    return GetDesiredTransitionFromOrbitToFollowPosition();
                case CameraState.FollowingTarget:
                default:
                    return GetDesiredFollowPosition(camParams.desiredOffset);
            }
        }



        /// <summary>Clamps the camera to its maximum distance from its target, as defined in the cam controller parameters.</summary>
        /// <param name="cam">The current camera.</param>
        /// <returns>The distance-clamped camera position.</returns>
        private Vector3 ClampCameraMaxDistance(Camera cam)
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.magnitude > camParams.maxDistanceFromTarget)
            {
                return followTarget.transform.position + followTargetToCam.normalized * camParams.maxDistanceFromTarget;
            }

            return cam.transform.position;
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
        private Vector3 ShortenFollowDistanceToAvoidRearCollision(Vector3 desiredPos)
        {
            Vector3 desiredDir = desiredPos - followTarget.transform.position;
            if (Physics.SphereCast(followTarget.transform.position, virtualCameraSphereRadius, desiredDir, out RaycastHit hit, desiredDir.magnitude, camParams.colliderLayerMask))
            {
                /*
                Debug.DrawRay(followTarget.transform.position, (desiredDir.normalized * hit.distance), new Color(1, 1, 0));
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.up * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.right * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.down * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.left * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.forward * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.back * virtualCameraSphereRadius, Color.white);
                */
                return followTarget.transform.position + (desiredDir.normalized * hit.distance);
            }
            else
            {
                return desiredPos;
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