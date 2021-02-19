using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(SharedCameraComponents))]
    public class OcclusionAvoidance : MonoBehaviour
    {
        //components
        private Camera cam;
        private SharedCameraComponents components;

        //trackers
        private float timeInOcclusionMultiplier = 0f;
        private Vector3[] previousTargetPositions;

        //convenience variables
        private GameObject followTarget;
        private CameraParams camParams;

        private void Awake()
        {
            cam = GetComponent<Camera>();
            components = GetComponent<SharedCameraComponents>();
            camParams = components.cameraParams;
        }

        private void Update()
        {
            UpdateOcclusionTimeMultiplier(Time.deltaTime);
        }

        /// <summary> Pulls the camera towards the follow target if the follow target is occluded by geometry.</summary>
        /// <param name="deltaTime">Time since the last camera update.</param>
        /// <returns>The closer camera position if occluded, the current camera position if not.</returns>
        public Vector3 GetOcclusionPullInResult(float deltaTime)
        {
            Vector3 camPos = cam.transform.position;

            if (camParams.useTimeInOcclusionMultiplier)
            {
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical * timeInOcclusionMultiplier, camParams.occlusionPullInSpeedHorizontal * timeInOcclusionMultiplier, deltaTime);
            }
            else if (IsOccluded())
            {
                return PullInCamera(camPos, camParams.occlusionPullInSpeedVertical, camParams.occlusionPullInSpeedHorizontal, deltaTime);
            }

            return camPos;
        }

        /// <summary>
        /// Gets the occlusion follow speed increase, taking into account the time-in-occlusion multiplier, if using.
        /// </summary>
        /// <returns>The occlusion follow speed increase.</returns>
        public float GetOcclusionFollowSpeedIncrease()
        {
            if (camParams.useTimeInOcclusionMultiplier)
            {
                return camParams.occlusionFollowSpeedIncrease * timeInOcclusionMultiplier;
            }
            else if (IsOccluded())
            {
                return camParams.occlusionFollowSpeedIncrease;
            }
            else
            {
                return 0f;
            }
        }

        /// <summary>
        /// Returns the camera position directly in front of any geometry occluding the player. Use this to jump the camera forward instantly to clear occlusion; if you want to
        /// smoothly move to a non-occluded position, use the other occlusion functions
        /// </summary>
        public Vector3 GetClosestUnoccludedCameraPosition(Camera cam, float virtualCameraSphereRadius)
        {
            Vector3 followTargetPos = followTarget.transform.position;
            Vector3 followTargetToNearClip = cam.GetNearClipPaneCentreWorld() - followTargetPos;
            if (Physics.SphereCast(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, out RaycastHit hit, followTargetToNearClip.magnitude, camParams.occluderLayerMask))
            {
                Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * hit.distance);
                return hitSphereCentre - (cam.transform.forward * cam.nearClipPlane); //move the camera's near clip pane centre to the centre of the hit sphere
            }

            return cam.transform.position;
        }

        public Vector3 GetClosestUnoccludedCameraPosition(Vector3 worldNearClipPanePos, Vector3 worldNearClipPaneDir, float virtualCameraSphereRadius)
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

        /// <summary>
        /// Checks if the desired camera position would cause the camera to intersect with geometry to the rear;  
        /// if so, shortens the distance between the desired camera position and the target to avoid this  
        /// </summary>
        /// <param name="desiredPos">The desired camera near clip pane position.</param>
        /// <returns>The closer camera position if an intersection was found, else the input position</returns>
        public Vector3 ShortenFollowDistanceToAvoidRearOcclusion(Vector3 desiredPos, float virtualCameraSphereRadius)
        {
            //cast a sphere with the camera's virtual size to check for rear occlusion
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

        /// <summary>Checks whether the camera's view of the target is occluded by level geometry by casting rays towards the target from each corner of the camera's near clip pane.</summary>
        /// <param name="clipPanePadding">Adds virtual padding to the camera's near clip pane size when casting rays. 0: actual near clip pane size; >0: increased clip pane size</param>
        /// <returns>True if the follow target is occluded by geometry on the occluder layer(s)</returns>
        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.GetNearClipPaneCentreWorld();

            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, camParams.occluderLayerMask, clipPanePadding);
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

            //initial spherecast of actual virtual camera size - this is necessary because, if the near clip pane is entirely occluded, 
            //the distance calculations in the next section could be > the clip pane radius, which would make it think the camera is not occluded
            if (Physics.SphereCast(nearClipPanePos, nearClipPaneRadius, nearClipToFollowTarget, out hit, camParams.occluderLayerMask))
            {
                //Jump to other side of occluder - cast sphere from follow target to camera
                //(reusing nearClipToFollowTarget backwards here)
                if(Physics.SphereCast(followTargetPos, nearClipPaneRadius, -nearClipToFollowTarget, out hit, camParams.occluderLayerMask))
                {
                    return followTargetPos - (nearClipToFollowTarget.normalized * hit.distance) - cam.GetNearClipPaneDirectionWorld();
                }
            }

            //cast checkRadius  
            if (Physics.SphereCast(nearClipPanePos, checkRadius, nearClipToFollowTarget, out hit, camParams.occluderLayerMask))
            {
                Vector3 hitPointToRay = GetPerpendicularVectorFromHitPointToRay(nearClipPanePos, nearClipToFollowTarget, hit.point);
                float hitPointToNearClipDist = hitPointToRay.magnitude - nearClipPaneRadius;

                //normalised [0, 1] perpendicular-to-ray distance from hit point to near clip pane virtual sphere edge (dist == near clip size -> 0, dist == check radius -> 1).
                //values <0 are possible if target is occluded, but standard lerp function clamps it 
                float normalisedDist = hitPointToNearClipDist / (checkRadius - nearClipPaneRadius);

                //remember to put near clip pane centre at centre of sphere!!
                return Vector3.Lerp(cam.transform.position, hit.point + hitPointToRay /* should this be minus? */ - cam.GetNearClipPaneDirectionWorld(), normalisedDist);
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

            //https://www.wikiwand.com/en/Vector_projection
            //vector projection of rayStart->hitPoint onto ray
            Vector3 proj = (Vector3.Dot(rayStartToHitPoint, rayDir) / rayDir.sqrMagnitude) * rayDir;

            //vector rejection of rayStart->hitPoint onto ray - this vector is perpendicular to the ray direction  
            Vector3 rej = rayStartToHitPoint - proj;

            //float perpDistanceFromNearClipPane = rej.magnitude - cam.GetNearClipPaneCentreToCornerDistance();

            return rej;
        }
    }
}