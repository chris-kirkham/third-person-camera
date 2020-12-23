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

        #region counters/trackers
        private Vector2 currentOrbitAngles = Vector2.zero;
        private float currentTimeInOcclusionMult = 0f;
        private Vector3[] previousTargetPositions;
        private float virtualCameraSphereRadius;
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
            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance(); //update virtual size in case fov, clip pane distance etc. changed
            UpdateConvenienceComponentVars();
            UpdateOcclusionEaseInOutCounters(Time.fixedDeltaTime);
            stateController.UpdateCameraState();

            Vector3 initialClipPanePos = cam.GetNearClipPaneCentreWorld(); //cache camera's near clip pane centre before any movement for collision avoidance

            if (stateController.GetCameraState() == CameraState.OrbitingTarget)
            {
                currentOrbitAngles = GetNewOrbitAngles(currentOrbitAngles, input.GetOrbitInput());
            }
            else
            {
                currentOrbitAngles = new Vector2(cam.transform.eulerAngles.y, cam.transform.eulerAngles.x);
            }
            //Debug.Log("current orbit angles = " + currentOrbitAngles);

            switch (stateController.GetCameraState())
            {
                case CameraState.FollowingTarget:
                case CameraState.TargetMovingTowardsCamera:
                    cam.transform.position = GetFollowPosition(stateController.GetCameraState(), Time.fixedDeltaTime);
                    break;
                case CameraState.OrbitingTarget:
                    cam.transform.position = OrbitTarget(GetDesiredOrbitPositionFromAngles(currentOrbitAngles), Time.fixedDeltaTime);
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
            if (camParams.avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionAvoidResult(Time.fixedDeltaTime);
            cam.transform.rotation = LookAtTarget(cam, Time.fixedDeltaTime);
            if (camParams.avoidCollisionWithGeometry) cam.transform.position = AvoidCollisions(initialClipPanePos, cam, followTarget.transform.position);
            if (camParams.useMaxDistance) cam.transform.position = ClampCameraMaxDistance(cam);
        }

        #region follow target
        //Returns the camera position after interpolating between the current camera position and the desired follow position, 
        //shortening the desired follow offset if it would be occluded
        private Vector3 GetFollowPosition(CameraState state, float deltaTime)
        {
            Vector3 desiredPos = GetDesiredCamPosition(state);
            Debug.DrawLine(cam.transform.position, desiredPos, Color.red);

            Vector3 newCamPos = camParams.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos;
            Debug.DrawLine(cam.transform.position, newCamPos, Color.yellow);

            if (camParams.useTimeInOcclusionMultiplier && currentTimeInOcclusionMult > 0)
            {
                if(camParams.interpolateTargetFollow)
                {
                    float t = deltaTime * camParams.followSpeed * Mathf.Max(1, camParams.occlusionIncreaseFollowSpeedMultiplier * currentTimeInOcclusionMult);
                    return Smoothstep(cam.transform.position, newCamPos, t);
                }
                else
                {
                    return newCamPos;
                }
            }
            else
            {
                if(camParams.interpolateTargetFollow)
                {
                    return Smoothstep(cam.transform.position, newCamPos, deltaTime * camParams.followSpeed);
                }
                else
                {
                    return newCamPos;
                }
            }
        }
        #endregion

        #region collision avoidance
        //Avoids camera clipping with geometry by jumping the camera forward if any of its near clip pane corners are inside geometry
        private Vector3 AvoidCollisions(Vector3 initialNearClipPanePos, Camera updatedCam, Vector3 followTargetPos)
        {
            //!!!!
            //BIG TODO: This function fails when the sphere cast checks' (both the one in GetCollidersBetweenCameraPositions and the inner sphere checks that begin from followTargetPos)
            //have their starting sphere positions overlap geometry (see link below). This DOES HAPPEN when the camera's field of view is very wide and/or the near clip pane distance is large.
            //See this link for a possible solution (extending the start position 1 virtual size backwards), or consider using a BoxCast 
            //Description of sphere cast issue - https://forum.unity.com/threads/spherecastall-returns-0-0-0-for-all-raycasthit-points.428302/
            //Possible solution - https://forum.unity.com/threads/analyzing-and-optimizing-unity-sphere-capsule-casts.233328/
            //!!!!

            //Check if the camera's virtual size sphere is intersecting with geometry
            //HashSet<Collider> overlapSphereColliders = new HashSet<Collider>(Physics.OverlapSphere(updatedCam.GetNearClipPaneCentreWorld(), virtualCameraSphereRadius, camParams.colliderLayerMask));
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

        //Returns colliders between two camera positions as a HashSet, using the camera's virtual size to sphere cast.
        //Intended use is to detect collisions by casting between the camera near clip position at the start of the update tick and its position after all movement
        //except collision avoidance has been applied
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
        //Takes the desired camera position and adjusts it so the follow target isn't occluded by objects in the scene.
        //If the follow target would be occluded if the camera moved to desiredPos, this function moves desiredPos towards the follow target.
        //If there are no occluders in the way of the desired position, returns desiredPos unmodified
        private Vector3 GetOcclusionAvoidResult(float deltaTime)
        {
            Vector3 camPos = cam.transform.position;
            float maxDistance = Vector3.Distance(followTarget.transform.position, camPos);

            if (IsOccluded(camParams.occlusionClipPanePadding))
            {
                Debug.DrawRay(camPos, (followTarget.transform.position - camPos).normalized * maxDistance, Color.cyan);
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical, camParams.occlusionPullInSpeedHorizontal, deltaTime);
            }

            return camPos;
        }

        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.transform.position;

            //Debug drawing
            //foreach (Vector3 clipPaneCorner in cam.GetNearClipPaneCornersWorld(clipPanePadding)) Debug.DrawRay(clipPaneCorner, dir, Color.white);
            
            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, camParams.occluderLayerMask, clipPanePadding);
        }

        private void UpdateOcclusionEaseInOutCounters(float deltaTime)
        {
            if (IsOccluded(camParams.occlusionClipPanePadding) && camParams.useTimeInOcclusionMultiplier)
            {
                currentTimeInOcclusionMult = Mathf.Min(currentTimeInOcclusionMult + deltaTime * camParams.timeInOcclusionRampUpSpeed, camParams.maxTimeInOcclusionMultiplier);
            }
            else
            {
                currentTimeInOcclusionMult = Mathf.Max(currentTimeInOcclusionMult - deltaTime * camParams.timeInOcclusionRampDownSpeed, 0f);
            }
        }
        #endregion

        #region orbit
        private Vector2 GetNewOrbitAngles(Vector2 currAngles, Vector2 orbitInput)
        {
            return new Vector2(ClampOrbitAngle(currAngles.x + orbitInput.x), Mathf.Clamp(currAngles.y + orbitInput.y, camParams.minOrbitYAngle, camParams.maxOrbitYAngle));
        }

        private float ClampOrbitAngle(float angle)
        {
            if (angle < -360) return angle + 360;
            if (angle >= 360) return angle - 360;
            return angle;
        }

        private Vector3 GetDesiredOrbitPositionFromAngles(Vector2 orbitAngles)
        {
            Quaternion rotation = Quaternion.Euler(orbitAngles.y, orbitAngles.x, 0f);
            Debug.DrawLine(followTarget.transform.position + rotation * camParams.desiredOffset, followTarget.transform.position + Vector3.up * camParams.desiredOffset.y, Color.white);
            return followTarget.transform.position + rotation * camParams.desiredOffset;
        }

        private Vector3 OrbitTarget(Vector3 desiredPos, float deltaTime)
        {
            return Vector3.Lerp(cam.transform.position, camParams.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos, deltaTime * camParams.orbitSpeed);
        }
        #endregion

        #region transition from orbit to follow
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
        //Returns the rotation which will make the camera look at the lookAtTarget
        private Quaternion GetDesiredLookAtTargetRotation()
        {
            Transform lookAtTransform = lookAtTarget.transform;
            return Quaternion.LookRotation((lookAtTransform.position + new Vector3(lookAtTransform.right.x, 0f, lookAtTransform.right.z).normalized * camParams.lookOffset) - cam.transform.position);
        }

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
        //Returns the desired target follow/orbit position based on the current camera state.
        //This is the "ideal" position the camera wants to be in, before taking into consideration obstacle/occlusion avoidance etc.
        private Vector3 GetDesiredCamPosition(CameraState state)
        {
            switch (state)
            {
                case CameraState.OrbitingTarget:
                    return GetDesiredOrbitPositionFromAngles(currentOrbitAngles);
                case CameraState.TargetMovingTowardsCamera:
                    if (camParams.useWorldSpaceOffset)
                    {
                        return followTarget.transform.position + camParams.desiredFrontOffset;
                    }
                    else
                    {
                        return followTarget.transform.position + followTarget.transform.TransformDirection(camParams.desiredFrontOffset);
                    }
                case CameraState.OrbitToFollow_HoldingOrbitAngle:
                    return GetDesiredOrbitPositionFromAngles(currentOrbitAngles); //hold orbit angles - state system won't allow updating them in this state
                case CameraState.OrbitToFollow_Transitioning:
                    return GetDesiredTransitionFromOrbitToFollowPosition();
                case CameraState.FollowingTarget:
                default:
                    if (camParams.useWorldSpaceOffset)
                    {
                        return followTarget.transform.position + camParams.desiredOffset;
                    }
                    else
                    {
                        return followTarget.transform.position + followTarget.transform.TransformDirection(camParams.desiredOffset);
                    }
            }
        }

        private Vector3 ClampCameraMaxDistance(Camera cam)
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.magnitude > camParams.maxDistanceFromTarget)
            {
                return followTarget.transform.position + followTargetToCam.normalized * camParams.maxDistanceFromTarget;
            }

            return cam.transform.position;
        }

        private Vector3 PullInCamera(Vector3 camPos, float speedY, float speedXZ, float deltaTime)
        {
            Vector3 targetPos = followTarget.transform.position;
            float newX = Mathf.SmoothStep(camPos.x, targetPos.x, speedXZ * deltaTime);
            float newY = Mathf.SmoothStep(camPos.y, targetPos.y, speedY * deltaTime);
            float newZ = Mathf.SmoothStep(camPos.z, targetPos.z, speedXZ * deltaTime);

            return new Vector3(newX, newY, newZ);
        }

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