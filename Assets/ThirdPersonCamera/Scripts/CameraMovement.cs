//debug preprocessor define
#if UNITY_EDITOR //true if running in the editor
    #define TPC_CAMERA_MOVEMENT_DEBUG //hopefully this is specific enough that nobody else has defined it in anything
#elif DEVELOPMENT_BUILD //true if "development build" flag is checked in build options
    #define TPC_CAMERA_MOVEMENT_DEBUG
#endif

using System.Collections.Generic;
using UnityEngine;
using static Lerps;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Main movement controller for the third-person camera. Handles camera movement.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraControllerInput))]
    [RequireComponent(typeof(CameraStateController))]
    [RequireComponent(typeof(SharedCameraComponents))]
    [RequireComponent(typeof(DebugDrawing))]
    public class CameraMovement : MonoBehaviour
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
        //virtual camera size
        private float virtualCameraSphereRadius;
        
        //orbit
        private Vector2 currentOrbitAngles = Vector2.zero;
        private Vector2 orbitHoldAngle = Vector2.zero;

        //occlusion avoidance
        private float timeInOcclusionMultiplier = 0f;
        private Vector3[] previousTargetPositions;

        //collision
        private float collisionMaxDistRelaxTime = 0f;
        #endregion

#if TPC_CAMERA_MOVEMENT_DEBUG
        private DebugDrawing debugInfo;
#endif

        private void Start()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            stateController = GetComponent<CameraStateController>();
            components = GetComponent<SharedCameraComponents>();
            UpdateConvenienceComponentVars();
            UpdateTrackers(Time.deltaTime);

#if TPC_CAMERA_MOVEMENT_DEBUG
            debugInfo = GetComponent<DebugDrawing>();
#endif
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

        #region general
        /// <summary>
        /// Main camera update function, and the only one which moves the camera. Called in either Update, LateUpdate, or FixedUpdate, depending on setting.
        /// </summary>
        /// <param name="deltaTime">Time since the last camera update, in seconds.</param>
        private void UpdateCamera(float deltaTime)
        {
            /* Update relevant info */
            UpdateConvenienceComponentVars();
            UpdateTrackers(deltaTime);

            /* Move camera */
            Vector3 initialClipPanePos = cam.GetNearClipPaneCentreWorld(); //cache camera's near clip pane centre before any movement for collision avoidance
            cam.transform.position = GetCameraMoveResult(stateController.GetCameraState(), deltaTime);
            if (camParams.useCamWhiskers)
            {
                cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), deltaTime);
            }
            
            if (camParams.avoidFollowTargetOcclusion)
            {
                cam.transform.position = GetOcclusionPullInResult(deltaTime);
            }
            
            if (camParams.useMinDistance)
            {
                cam.transform.position = ClampCameraMinDistance(camParams.minDistanceFromTarget);
            }

            //max distance clamp is dependant on whether the camera collided with geometry this tick
            Vector3 collisionAvoidPos = Vector3.zero;
            float maxDist = camParams.maxDistanceFromTarget;
            if (camParams.avoidCollisionWithGeometry)
            {
                collisionAvoidPos = AvoidCollisions(initialClipPanePos, cam);
            }
            
            if(collisionAvoidPos != cam.transform.position)
            {
                cam.transform.position = collisionAvoidPos;
            }
            
            if (camParams.useMaxDistance)
            {
                cam.transform.position = ClampCameraMaxDistance(maxDist);
            }

            cam.transform.rotation = LookAtTarget(deltaTime);

            //TEST
#if TPC_CAMERA_MOVEMENT_DEBUG
            Vector3 occlusionAvoidPos = GetOcclusionPullForwardPosition(cam, followTarget.transform.position, 1f);
            Debug.DrawLine(cam.transform.position, occlusionAvoidPos, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.up * cam.nearClipPlane, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.right * cam.nearClipPlane, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.down * cam.nearClipPlane, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.left * cam.nearClipPlane, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.forward * cam.nearClipPlane, Color.yellow);
            Debug.DrawRay(occlusionAvoidPos, Vector3.back * cam.nearClipPlane, Color.yellow);

            debugInfo.followTargetPos = followTarget.transform.position;
            debugInfo.camNearClipPanePos = cam.GetNearClipPaneCentreWorld();
            debugInfo.camVirtualSize = virtualCameraSphereRadius;
#endif
        }

        private Vector3 GetCameraMoveResult(CameraState state, float deltaTime)
        {
            switch (state)
            {
                case CameraState.FollowingTarget:
                    return FollowTarget(deltaTime);
                case CameraState.OrbitingTarget:
                    return OrbitTarget(currentOrbitAngles, deltaTime);
                case CameraState.OrbitToFollow_HoldingOrbitAngle:
                    return OrbitTarget(orbitHoldAngle, deltaTime);
                default:
                    return cam.transform.position;
            }
        }

        //TODO: This occasionally produces jerky movement when running in circles
        private Vector3 ClampCameraMinDistance(float distance)
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
        private Vector3 ClampCameraMaxDistance(float distance)
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
        /// <param name="desiredPos">The desired camera near clip pane position.</param>
        /// <returns>The closer camera position if an intersection was found, else the input position</returns>
        private Vector3 ShortenFollowDistanceToAvoidRearOcclusion(Vector3 desiredPos)
        {
            //cast a sphere with the camera's virtual size to check for rear occlusion
            Vector3 desiredDir = desiredPos - followTarget.transform.position;
            if (Physics.SphereCast(followTarget.transform.position, virtualCameraSphereRadius, desiredDir, out RaycastHit hit, desiredDir.magnitude, camParams.occluderLayerMask))
            {
#if TPC_CAMERA_MOVEMENT_DEBUG
                Debug.DrawRay(followTarget.transform.position, desiredDir.normalized * hit.distance, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.up * virtualCameraSphereRadius, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.down * virtualCameraSphereRadius, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.left * virtualCameraSphereRadius, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.right * virtualCameraSphereRadius, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.forward * virtualCameraSphereRadius, Color.magenta);
                Debug.DrawRay(followTarget.transform.position + desiredDir.normalized * hit.distance, Vector3.back * virtualCameraSphereRadius, Color.magenta);
#endif

                //remember to add near clip pane distance to camera distance
                return followTarget.transform.position + (desiredDir.normalized * hit.distance);
            }
            else
            {
                return desiredPos;
            }
        }

        private Vector3 MoveCameraInterpolated(Vector3 desiredPos, float moveSpeed, float deltaTime)
        {
            return Smoothstep(cam.transform.position, desiredPos, moveSpeed * deltaTime);
        }

        private Vector3 MoveCameraInterpolated(Vector3 desiredPos, float horizontalMoveSpeed, float verticalMoveSpeed, float deltaTime)
        {
            return Smoothstep(cam.transform.position, desiredPos, horizontalMoveSpeed * deltaTime, verticalMoveSpeed * deltaTime);
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
        private Vector3 FollowTarget(float deltaTime)
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
            if(camParams.followHeightMode == FollowHeightMode.AboveGround) desiredOffset.y = GetDesiredHeightAboveGroundOffset();
            
            //get desired position from offset
            Vector3 desiredPos = GetDesiredFollowPositionWorld(desiredOffset);

            //shorten follow distance if necessary
            desiredPos = ShortenFollowDistanceToAvoidRearOcclusion(desiredPos);

            //increase follow speed if avoiding occlusion
            if (camParams.avoidFollowTargetOcclusion)
            {
                float occlusionSpeedIncrease = GetOcclusionFollowSpeedIncrease();
                followSpeedHorz += occlusionSpeedIncrease;
                followSpeedVert += occlusionSpeedIncrease;
            }

            //interpolate between current and desired positions
            Vector3 newCamPos = MoveCameraInterpolated(desiredPos, followSpeedHorz, followSpeedVert, deltaTime);

            //interpolate distance from target separately 
            float desiredDistance = desiredOffset.magnitude;
            Vector3 newOffset = newCamPos - followTarget.transform.position; //convert new cam position back to a world space offset from follow target
            float newDistance = Smoothstep(newOffset.magnitude, desiredDistance, followSpeedDistance * deltaTime); //interpolate between current offset distance and desired distance
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
        /// <returns>The new camera position after collision avoidance has been applied. Returns updatedCam position if no collision detected.</returns>
        private Vector3 AvoidCollisions(Vector3 initialNearClipPanePos, Camera updatedCam)
        {
#if TPC_CAMERA_MOVEMENT_DEBUG
            debugInfo.initialNearClipPanePos = initialNearClipPanePos;
            debugInfo.updatedNearClipPanePos = updatedCam.GetNearClipPaneCentreWorld();
#endif
            //!!!!
            //BIG TODO: This function fails when the sphere cast checks' (both the one in GetCollidersBetweenCameraPositions and the inner sphere checks that begin from followTargetPos)
            //have their starting sphere positions overlap geometry (see link below). This DOES HAPPEN when the camera's field of view is very wide and/or the near clip pane distance is large.
            //See this link for a possible solution (extending the start position 1 virtual size backwards), or consider using a BoxCast 
            //Description of sphere cast issue - https://forum.unity.com/threads/spherecastall-returns-0-0-0-for-all-raycasthit-points.428302/
            //Possible solution - https://forum.unity.com/threads/analyzing-and-optimizing-unity-sphere-capsule-casts.233328/
            //!!!!

            //Check if the path between the camera's initial position this tick and its updated position would cause it to intersect with any geometry
            //If so, cast a sphere back from follow target to camera's near clip pane, find the closest hit collider -which is also intersecting the camera's path-
            //to the follow target, and move the camera in front of that.
            Vector3 followTargetPos = followTarget.transform.position;
            HashSet<Collider> camColliders = GetCollidersBetweenCameraPositions(initialNearClipPanePos, updatedCam.GetNearClipPaneCentreWorld());
            if (camColliders.Count > 0) 
            {
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
                        Debug.DrawRay(followTargetPos, followTargetToNearClip.normalized * hit.distance, Color.red);
                    }
                }

                if(minDistance != Mathf.Infinity)
                {
                    Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * minDistance);

#if TPC_CAMERA_MOVEMENT_DEBUG
                    debugInfo.minHitDistance = minDistance;
                    debugInfo.minHitSphereCentre = hitSphereCentre;
                    debugInfo.AddCamCollisionPosition(hitSphereCentre - (updatedCam.transform.forward * updatedCam.nearClipPlane));
#endif
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
            float overlapSphereInterval = 0.1f;
            Vector3 initialToUpdated = updatedNearClipPane - initialNearClipPane;
            Vector3 initialToUpdatedNorm = initialToUpdated.normalized;
            HashSet<Collider> overlaps = new HashSet<Collider>();

            //do overlap spheres at intervals along path from initial to updated near clip panes until collider(s) found
            for(float dist = 0; dist < initialToUpdated.magnitude; dist += overlapSphereInterval)
            {
                Debug.DrawRay(initialNearClipPane + (initialToUpdatedNorm * dist), Vector3.up * 0.1f, new Color(1, 1, 1, 0.5f));
                foreach (Collider c in Physics.OverlapSphere(initialNearClipPane + (initialToUpdatedNorm * dist), virtualCameraSphereRadius, camParams.colliderLayerMask))
                {
                    overlaps.Add(c);
                }

                if (overlaps.Count > 0) break; 
            }
            
            //final overlap sphere exactly at updated clip pane pos if no colliders found
            if(overlaps.Count == 0)
            {
                Debug.DrawRay(updatedNearClipPane, Vector3.up * 0.1f, new Color(1, 1, 1, 0.5f));
                foreach (Collider c in Physics.OverlapSphere(updatedNearClipPane, virtualCameraSphereRadius, camParams.colliderLayerMask))
                {
                    overlaps.Add(c);
                }
            }
            
            return overlaps;
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
        /// <summary>
        /// Returns the camera pull-forward position to avoid occlusion based on how much a <paramref name="checkRadius"/>-size sphere is occluded to the target.
        /// If the target is actually occluded (i.e. a sphere with the radius of the actual near clip pane diagonal is occluded), the function will return the position
        /// which jumps the camera just in front of the occluder.
        /// </summary>
        public Vector3 GetOcclusionPullForwardPosition(Camera cam, Vector3 followTargetPos, float checkRadius)
        {
            Vector3 nearClipPanePos = cam.GetNearClipPaneCentreWorld();
            float nearClipPaneRadius = cam.GetNearClipPaneCentreToCornerDistance();
            Vector3 nearClipToFollowTarget = followTargetPos - nearClipPanePos;
            RaycastHit hit;

            Debug.DrawRay(nearClipPanePos, nearClipToFollowTarget, Color.green);
            Debug.DrawRay(nearClipPanePos - cam.transform.right * checkRadius, nearClipToFollowTarget, Color.green);
            Debug.DrawRay(nearClipPanePos + cam.transform.up * checkRadius, nearClipToFollowTarget, Color.green);
            Debug.DrawRay(nearClipPanePos + cam.transform.right * checkRadius, nearClipToFollowTarget, Color.green);
            Debug.DrawRay(nearClipPanePos - cam.transform.up * checkRadius, nearClipToFollowTarget, Color.green);

            /*
            //initial spherecast of actual virtual camera size - this is necessary because, if the near clip pane is entirely occluded, 
            //the distance calculations in the next section could be > the clip pane radius, which would make it think the camera is not occluded
            if (Physics.SphereCast(nearClipPanePos, nearClipPaneRadius, nearClipToFollowTarget, out hit, nearClipToFollowTarget.magnitude, camParams.occluderLayerMask))
            {
                //Jump to other side of occluder - cast sphere from follow target to camera
                //(reusing nearClipToFollowTarget backwards here)
                if (Physics.SphereCast(followTargetPos, nearClipPaneRadius, -nearClipToFollowTarget, out hit, camParams.occluderLayerMask))
                {
                    return followTargetPos - (nearClipToFollowTarget.normalized * hit.distance) - cam.GetNearClipPaneDirectionWorld();
                }
            }
            */

            //cast checkRadius  
            if (Physics.SphereCast(nearClipPanePos, checkRadius, nearClipToFollowTarget, out hit, nearClipToFollowTarget.magnitude, camParams.occluderLayerMask))
            {
                Debug.Log("check spherecast true");
                Vector3 hitPointToRay = GetPerpendicularVectorFromHitPointToRay(nearClipPanePos, nearClipToFollowTarget, hit.point);
                float hitPointToNearClipDist = hitPointToRay.magnitude - nearClipPaneRadius;

                //normalised [0, 1] perpendicular-to-ray distance from hit point to near clip pane virtual sphere edge (dist == near clip size -> 0, dist == check radius -> 1).
                //values <0 are possible if target is occluded, but standard lerp function clamps it 
                float normalisedDist = hitPointToNearClipDist / (checkRadius - nearClipPaneRadius);

                //remember to put near clip pane centre at centre of sphere!!
                return Vector3.Lerp(cam.transform.position, (hit.point + hitPointToRay) /* should this be minus? */ - cam.GetNearClipPaneDirectionWorld(), normalisedDist);
            }

            return cam.transform.position;
        }

        /// <summary>
        /// Returns the distance, perpendicular to the ray direction, of the hit point to the ray,
        /// i.e. the vector rejection of the ray start -> hit point vector onto the ray vector (see "vector projection" on wikipedia) 
        /// </summary>
        /// <returns></returns>
        private Vector3 GetPerpendicularVectorFromHitPointToRay(Vector3 rayStart, Vector3 rayDir, Vector3 hitPoint)
        {
            Vector3 rayStartToHitPoint = hitPoint - rayStart;
            Debug.DrawRay(rayStart, rayStartToHitPoint, Color.blue);

            //https://www.wikiwand.com/en/Vector_projection
            //vector projection of rayStart->hitPoint onto ray
            Vector3 proj = (Vector3.Dot(rayStartToHitPoint, rayDir) / rayDir.sqrMagnitude) * rayDir;
            Debug.DrawRay(rayStart, proj, Color.red);

            //vector rejection of rayStart->hitPoint onto ray - this vector is perpendicular to the ray direction  
            Vector3 rej = rayStartToHitPoint - proj;
            Debug.DrawRay(rayStart + proj, rej, Color.cyan);
            Debug.DrawRay(rayStart + proj + rej, Vector3.up * 0.1f, Color.red);

            return -rej;
        }


        /// <summary>Checks whether the camera's view of the target is occluded by level geometry by casting rays towards the target from each corner of the camera's near clip pane.</summary>
        /// <param name="clipPanePadding">Adds virtual padding to the camera's near clip pane size when casting rays. 0: actual near clip pane size; >0: increased clip pane size</param>
        /// <returns>True if the follow target is occluded by geometry on the occluder layer(s)</returns>
        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.GetNearClipPaneCentreWorld();

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
            else
            {
                return 0f;
            }
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
        }

        /// <summary>
        /// Returns the camera position directly in front of any geometry occluding the player. Use this to jump the camera forward instantly to clear occlusion; if you want to
        /// smoothly move to a non-occluded position, use the other occlusion functions
        /// </summary>
        private Vector3 GetClosestUnoccludedCameraPosition(Camera cam)
        {
            Vector3 followTargetPos = followTarget.transform.position;
            Vector3 followTargetToNearClip = cam.GetNearClipPaneCentreWorld() - followTargetPos;
            if(Physics.SphereCast(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, out RaycastHit hit, followTargetToNearClip.magnitude, camParams.occluderLayerMask))
            {
                Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * hit.distance);
                return hitSphereCentre - (cam.transform.forward * cam.nearClipPlane); //move the camera's near clip pane centre to the centre of the hit sphere
            }

            return cam.transform.position;
        }

        private Vector3 GetClosestUnoccludedCameraPosition(Vector3 worldNearClipPanePos, Vector3 worldNearClipPaneDir)
        {
            Vector3 followTargetPos = followTarget.transform.position;
            Vector3 followTargetToNearClip = worldNearClipPanePos - followTargetPos;
            if (Physics.SphereCast(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, out RaycastHit hit, followTargetToNearClip.magnitude, camParams.occluderLayerMask))
            {
                Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * hit.distance);
                return hitSphereCentre - worldNearClipPaneDir; //move the camera's near clip pane centre to the centre of the hit sphere
            }

            return cam.transform.position;
        }
        #endregion

        #region orbit
        /// <summary>Gets the new (interpolated) camera orbit position.</summary>
        /// <param name="orbitAngles">Angle to orbit</param>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private Vector3 OrbitTarget(Vector3 orbitAngles, float deltaTime)
        {
            //get new camera position (this can/will change the distance from the target, but it is interpolated seperately next anyway)
            Vector3 desiredPos = GetDesiredOrbitPositionFromAngles(orbitAngles);
            Vector3 newCamPos = MoveCameraInterpolated(desiredPos, camParams.orbitSpeed, deltaTime);
            
            //interpolate distance from target separately 
            Vector3 newOffset = newCamPos - followTarget.transform.position; //convert new cam position back to a world space offset from follow target
            float newDistance = Smoothstep((cam.transform.position - followTarget.transform.position).magnitude, //interpolate between current offset distance and desired distance
                camParams.desiredOrbitDistance, camParams.orbitSpeedDistance * deltaTime); 
            return followTarget.transform.position + (newOffset.normalized * newDistance); //scale normalised offset by new distance and convert back to world position
        }

        /// <summary>Finds the camera's desired orbit position from its orbit angles and distance.</summary>
        private Vector3 GetDesiredOrbitPositionFromAngles(Vector2 orbitAngles)
        {
            Quaternion rotation = Quaternion.Euler(orbitAngles.x, orbitAngles.y, 0f);
            return followTarget.transform.position + (rotation * (Vector3.back * camParams.desiredOrbitDistance));
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

        /* functions to update orbit trackers */

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
        private Vector2 GetNewOrbitAngles(Vector2 currAngles, Vector2 orbitInput)
        {
            return new Vector2(Mathf.Clamp(currAngles.x + orbitInput.x, camParams.minOrbitYAngle, camParams.maxOrbitYAngle), Clamp360(currAngles.y + orbitInput.y));
        }

        /// <summary> caches the current orbit angle for orbit-to-follow transition hold</summary>
        private void UpdateOrbitHoldAngles(CameraState state)
        {
            if (state != CameraState.OrbitToFollow_HoldingOrbitAngle)
            {
                orbitHoldAngle = currentOrbitAngles;
            }
        }
        #endregion

        #region look at target
        /// <summary>
        /// Finds the rotation which will make the camera look at the look target (+ desired offset, if any)
        /// </summary>
        /// <returns>The desired look-at-target rotation</returns>
        private Quaternion GetDesiredLookAtTargetRotation()
        {
            var lookAtTransform = lookAtTarget.transform;
            var look = new Vector3(lookAtTransform.right.x, 0f, lookAtTransform.right.z).normalized * camParams.lookOffset;
            return Quaternion.LookRotation((lookAtTransform.position + look) - cam.transform.position, Vector3.up);
        }

        /// <summary>
        /// Calculates the camera's new look-at rotation.
        /// </summary>
        /// <param name="deltaTime">Time since the last camera update</param>
        /// <returns>The camera's new look-at-target rotation. If this is not interpolated, it will be the same as the desired look-at rotation</returns>
        private Quaternion LookAtTarget(float deltaTime)
        {
            //N.B. don't interpolate lookat if in orbit mode 
            if (stateController.GetCameraState() != CameraState.OrbitingTarget && camParams.interpolateTargetLookAt)
            {
                return Quaternion.Slerp(cam.transform.rotation, GetDesiredLookAtTargetRotation(), deltaTime * camParams.targetLookAtLerpSpeed);
            }
            else
            {
                return GetDesiredLookAtTargetRotation();
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
    }
}