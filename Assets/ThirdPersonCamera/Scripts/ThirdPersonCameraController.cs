using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Lerps;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraControllerInput))]
    [RequireComponent(typeof(CameraStateController))]
    public class ThirdPersonCameraController : MonoBehaviour
    {
        /* Components */
        #region components
        private Camera cam;
        private CameraControllerInput input;
        private CameraStateController stateController;

        public GameObject followTarget;
        public GameObject lookAtTarget;
        [InspectorName("Camera parameters")] public CameraParams p;
        #endregion

        #region counters/trackers
        private float currentTimeInOcclusionMult = 0f;
        private Vector3[] previousTargetPositions;
        private float virtualCameraSphereRadius;
        #endregion

        private void Awake()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            stateController = GetComponent<CameraStateController>();

            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance();
        }

        //all camera movement is done here
        private void FixedUpdate()
        {
            Vector3 initialClipPanePos = cam.transform.position;
            Debug.Log("initial clip pane pos: " + initialClipPanePos);
            UpdateOcclusionEaseInOutCounters(Time.fixedDeltaTime);
            stateController.UpdateCameraState(cam, followTarget, p);

            switch (stateController.GetCameraState())
            {
                case CameraState.FollowingTarget:
                case CameraState.TargetMovingTowardsCamera:
                    cam.transform.position = GetFollowPosition(stateController.GetCameraState(), Time.fixedDeltaTime);
                    break;
                case CameraState.OrbitingTarget:
                    cam.transform.position = OrbitTarget(Time.fixedDeltaTime, input.GetOrbitAngles());
                    break;
                case CameraState.TransitioningFromOrbitToFollow:
                    cam.transform.position = GetFollowPosition(stateController.GetCameraState(), Time.fixedDeltaTime);
                    break;
            }

            if (p.useCamWhiskers) cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime);
            if (p.avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionAvoidResult(Time.fixedDeltaTime);

            cam.transform.rotation = LookAtTarget(cam, Time.fixedDeltaTime);


            if (p.avoidCollisionWithGeometry)
            {
                cam.transform.position = AvoidCollisions_JumpForward(initialClipPanePos, cam);
            }

            //clamp camera distance to max distance
            if (p.useMaxDistance)
            {
                cam.transform.position = ClampCameraDistance(cam);
            }

        }

        #region follow target
        //Returns the desired target follow position based on the current camera state.
        //This is the "ideal" position the camera wants to be in, before taking into consideration obstacle/occlusion avoidance etc.
        private Vector3 GetDesiredFollowPosition(CameraState state)
        {
            switch(state)
            {
                case CameraState.TargetMovingTowardsCamera:
                    if(p.useWorldSpaceOffset)
                    {
                        return followTarget.transform.position + p.desiredFrontOffset;
                    }
                    else
                    {
                        return followTarget.transform.position + followTarget.transform.TransformDirection(p.desiredFrontOffset);
                    }
                case CameraState.FollowingTarget:
                default:
                    if(p.useWorldSpaceOffset)
                    {
                        return followTarget.transform.position + p.desiredOffset;
                    }
                    else
                    {
                        return followTarget.transform.position + followTarget.transform.TransformDirection(p.desiredOffset);
                    }
            }
        }

        //Returns the camera position after interpolating between the current camera position and the desired follow position, 
        //shortening the desired follow offset if it would be occluded
        private Vector3 GetFollowPosition(CameraState state, float deltaTime)
        {
            Vector3 desiredPos = GetDesiredFollowPosition(state);
            Debug.DrawLine(cam.transform.position, desiredPos, Color.red);

            Vector3 newCamPos = p.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos;

            Debug.DrawLine(cam.transform.position, newCamPos, Color.yellow);

            if (p.useTimeInOcclusionMultiplier && currentTimeInOcclusionMult > 0)
            {
                return Smoothstep(cam.transform.position, newCamPos, deltaTime * p.followSpeed * Mathf.Max(1, p.occlusionIncreaseFollowSpeedMultiplier * currentTimeInOcclusionMult));
            }
            else
            {
                return Smoothstep(cam.transform.position, newCamPos, deltaTime * p.followSpeed);
            }
        }
        #endregion

        #region collision avoidance
        //Avoids camera clipping with geometry by jumping the camera forward if any of its near clip pane corners are inside geometry
        private Vector3 AvoidCollisions_JumpForward(Vector3 initialNearClipPanePos, Camera updatedCam)
        {
            //Check if the camera's virtual size sphere is intersecting with geometry
            HashSet<Collider> overlapSphereColliders = new HashSet<Collider>(Physics.OverlapSphere(updatedCam.GetNearClipPaneCentreWorld(), virtualCameraSphereRadius, p.colliderLayerMask));
            //HashSet<Collider> overlapSphereColliders = GetCollidersBetweenCameraPositions(initialNearClipPanePos, updatedCam.GetNearClipPaneCentreWorld());
            if (overlapSphereColliders.Count > 0) //If so, cast a sphere back from follow target to camera's near clip pane
            {
                Vector3 avoidPositions = Vector3.zero;
                int colliderSphereHits = 0;
                Vector3 followTargetToNearClip = updatedCam.GetNearClipPaneCentreWorld() - followTarget.transform.position;
                RaycastHit[] hits = Physics.SphereCastAll(followTarget.transform.position, virtualCameraSphereRadius, followTargetToNearClip, followTargetToNearClip.magnitude, p.colliderLayerMask);
                
                foreach (RaycastHit hit in hits) //for each sphere hit, check if the hit is on a collider in our OverlapSphere colliders list; adjust camera position based on hit if so
                {
                    if (overlapSphereColliders.Contains(hit.collider)) 
                    {
                        Debug.DrawLine(followTarget.transform.position, hit.point, Color.green);
                        
                        //note: the hit.distance of a RaycastHit from a sphere cast is the distance to the centre of the sphere that hit something,
                        //not the distance to the hit point: https://answers.unity.com/questions/882631/how-to-get-the-center-of-a-capsulecast.html
                        Vector3 hitSphereCentre = followTarget.transform.position + followTargetToNearClip.normalized * hit.distance;
                        avoidPositions += hitSphereCentre - updatedCam.transform.forward * updatedCam.nearClipPlane;
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
            Vector3 initialToUpdatedNearClip = updatedNearClipPane - initialNearClipPane;
            RaycastHit[] hits = Physics.SphereCastAll(initialNearClipPane, virtualCameraSphereRadius, initialToUpdatedNearClip, initialToUpdatedNearClip.magnitude, p.colliderLayerMask);
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
            float halfWhiskerSectorAngle = p.whiskerSectorAngle / 2;
            float angleInc = halfWhiskerSectorAngle / p.numWhiskers * 2;

            for (float angle = -halfWhiskerSectorAngle; angle <= halfWhiskerSectorAngle; angle += angleInc)
            {
                Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * -followTarget.transform.forward;
                Debug.DrawRay(followTargetPos, dir * p.whiskerLength, Color.HSVToRGB(Mathf.Abs(angle) / p.whiskerSectorAngle, 0.2f, 1f)); //visualise raycasts
                if (Physics.Raycast(followTargetPos, dir, p.whiskerLength)) whiskerPushDir -= new Vector3(dir.x, 0f, dir.z);
            }

            //zero local z (is there a better way to do this?)
            whiskerPushDir = cam.transform.InverseTransformDirection(whiskerPushDir);
            whiskerPushDir.z = 0;
            whiskerPushDir = cam.transform.TransformDirection(whiskerPushDir);
            Debug.DrawRay(followTargetPos, whiskerPushDir, Color.yellow);

            return whiskerPushDir / p.numWhiskers;
        }

        private Vector3 GetCamWhiskerResult(Vector3 camPos, Vector3 whiskerOffset, float deltaTime)
        {
            return Smoothstep(camPos, camPos + whiskerOffset, p.whiskerPushStrength * deltaTime);
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

            if (IsOccluded(p.occlusionClipPanePadding))
            {
                //Debug.DrawRay(camPos, (followTarget.transform.position - camPos).normalized * maxDistance, Color.cyan);
                return PullInCamera(camPos, p.occlusionPullInSpeedVertical, p.occlusionPullInSpeedHorizontal, deltaTime);
            }

            return camPos;
        }

        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.transform.position;

            //Debug drawing
            //foreach (Vector3 clipPaneCorner in cam.GetNearClipPaneCornersWorld(clipPanePadding)) Debug.DrawRay(clipPaneCorner, dir, Color.white);
            
            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, p.occluderLayerMask, clipPanePadding);
        }

        private void UpdateOcclusionEaseInOutCounters(float deltaTime)
        {
            if (IsOccluded(p.occlusionClipPanePadding) && p.useTimeInOcclusionMultiplier)
            {
                currentTimeInOcclusionMult = Mathf.Min(currentTimeInOcclusionMult + deltaTime * p.timeInOcclusionRampUpSpeed, p.maxTimeInOcclusionMultiplier);
            }
            else
            {
                currentTimeInOcclusionMult = Mathf.Max(currentTimeInOcclusionMult - deltaTime * p.timeInOcclusionRampDownSpeed, 0f);
            }
        }
        #endregion

        #region orbit
        private Vector3 OrbitTarget(float deltaTime, Vector2 orbitAngles)
        {
            Vector2 clampedAngles = new Vector2(orbitAngles.x, Mathf.Clamp(orbitAngles.y, p.minOrbitYAngle, p.maxOrbitYAngle));
            Quaternion rotation = Quaternion.Euler(clampedAngles.y, clampedAngles.x, 0f);
            Vector3 desiredPos = followTarget.transform.position + rotation * p.desiredOffset;

            return Vector3.Lerp(cam.transform.position, p.avoidFollowTargetOcclusion ? ShortenFollowDistanceToAvoidRearCollision(desiredPos) : desiredPos, deltaTime * p.orbitSpeed);
        }
        #endregion

        #region look at target
        //Returns the rotation which will make the camera look at the lookAtTarget
        private Quaternion GetDesiredLookAtTargetRotation()
        {
            Transform lookAtTransform = lookAtTarget.transform;
            return Quaternion.LookRotation((lookAtTransform.position + new Vector3(lookAtTransform.right.x, 0f, lookAtTransform.right.z).normalized * p.lookOffset) - cam.transform.position);
        }

        private Quaternion LookAtTarget(Camera cam, float deltaTime)
        {
            if (p.interpolateTargetLookAt)
            {
                return Quaternion.Lerp(cam.transform.rotation, GetDesiredLookAtTargetRotation(), deltaTime * p.targetLookAtLerpSpeed);
            }
            else
            {
                return GetDesiredLookAtTargetRotation();
            }
        }
        #endregion

        #region general
        private Vector3 ClampCameraDistance(Camera cam)
        {
            Vector3 followTargetToCam = cam.transform.position - followTarget.transform.position;
            if (followTargetToCam.magnitude > p.maxDistanceFromTarget)
            {
                return followTarget.transform.position + followTargetToCam.normalized * p.maxDistanceFromTarget;
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
            if (Physics.SphereCast(followTarget.transform.position, virtualCameraSphereRadius, desiredDir, out RaycastHit hit, desiredDir.magnitude, p.colliderLayerMask))
            {
                Debug.DrawRay(followTarget.transform.position, (desiredDir.normalized * hit.distance), new Color(1, 1, 0));
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.up * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.right * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.down * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.left * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.forward * virtualCameraSphereRadius, Color.white);
                Debug.DrawRay(followTarget.transform.position + (desiredDir.normalized * hit.distance), Vector3.back * virtualCameraSphereRadius, Color.white);

                return followTarget.transform.position + (desiredDir.normalized * hit.distance);
            }
            else
            {
                return desiredPos;
            }
        }
        #endregion

        private void OnDrawGizmos()
        {
            if (UnityEditor.EditorApplication.isPlaying)
            {
                //draw camera virtual size
                Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
                Gizmos.DrawWireSphere(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius);
            }
        }

    }
}