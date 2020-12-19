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

            if (p.interpolateTargetLookAt)
            {
                cam.transform.rotation = Quaternion.Lerp(cam.transform.rotation, GetTargetLookAtRotation(), Time.fixedDeltaTime * p.targetLookAtLerpSpeed);
            }
            else
            {
                cam.transform.rotation = GetTargetLookAtRotation(); 
            }

            //cam.transform.rotation = GetTargetLookAtRotation(GetCamWhiskerResult(lookAtTarget.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime));

            if (p.avoidCollisionWithGeometry)
            {
                cam.transform.position = AvoidCollisions_JumpForward(cam);
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

            Vector3 newCamPos = ShortenFollowDistanceToAvoidRearCollision(desiredPos);

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
        private Vector3 AvoidCollisions_JumpForward(Camera cam)
        {
            //TODO: Sometimes causes jittering back and forward 
            //(Seems to be: camera jumps forward, then doesn't see a collision until it's moved behind the object, then sees a collision and jumps forward...)
            //possibly due to only checking the first hit collider?

            //Check if the camera's virtual size sphere is intersecting with geometry
            Collider[] overlapSphereCollider = new Collider[1];
            if (Physics.OverlapSphereNonAlloc(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius, overlapSphereCollider) > 0)
            {
                //If so, cast a sphere back from follow target to camera's near clip pane
                Vector3 followTargetToNearClip = cam.GetNearClipPaneCentreWorld() - followTarget.transform.position;
                RaycastHit[] hits = Physics.SphereCastAll(followTarget.transform.position, virtualCameraSphereRadius, followTargetToNearClip, followTargetToNearClip.magnitude, p.colliderLayerMask);
                foreach(RaycastHit hit in hits)
                {
                    if(hit.collider == overlapSphereCollider[0]) 
                    {
                        Debug.DrawLine(followTarget.transform.position, hit.point, Color.green);
                        Vector3 hitSphereCentre = followTarget.transform.position + followTargetToNearClip.normalized * hit.distance;
                        Debug.DrawRay(hitSphereCentre, Vector3.up, Color.blue);
                        return hitSphereCentre - cam.transform.forward * cam.nearClipPlane;
                    }
                }
            }

            /*
            //OLD METHOD - CHECK CLIP PANE CORNERS
            //Check if the camera's virtual size sphere is intersecting with geometry
            Collider[] colliders = new Collider[1];
            if (Physics.OverlapSphereNonAlloc(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius, colliders) > 0)
            {
                //check if any of the camera's near clip pane corners are inside geometry
                Vector3[] nearClipPaneCorners = cam.GetNearClipPaneCornersWorld();
                float longestDistInsideGeometry = 0f;
                int longestDistCornerIndex = -1;
                Vector3 longestDistHitPoint = Vector3.zero;
                RaycastHit hit;
                for (int i = 0; i < 4; i++)
                {
                    Vector3 followTargetToClipCorner = nearClipPaneCorners[i] - followTarget.transform.position;
                    if (colliders[0].Raycast(new Ray(followTarget.transform.position, followTargetToClipCorner), out hit, followTargetToClipCorner.magnitude))
                    {
                        Debug.DrawLine(followTarget.transform.position, hit.point, new Color(1, 0.5f, 1));
                        float dist = Vector3.Distance(hit.point, nearClipPaneCorners[i]);
                        if (dist > longestDistInsideGeometry)
                        {
                            longestDistInsideGeometry = dist;
                            longestDistCornerIndex = i;
                            longestDistHitPoint = hit.point;
                        }
                    }
                }

                //if any clip pane corners are inside geometry, move the camera such that the point deepest inside geometry is touching its ray hit point 
                if (longestDistCornerIndex != -1)
                {
                    Debug.DrawRay(longestDistHitPoint, cam.transform.position - nearClipPaneCorners[longestDistCornerIndex], new Color(1, 0, 1));
                    return longestDistHitPoint + (cam.transform.position - nearClipPaneCorners[longestDistCornerIndex]);
                }
            }
            */

            return cam.transform.position;
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

            return Vector3.Lerp(cam.transform.position, ShortenFollowDistanceToAvoidRearCollision(desiredPos), deltaTime * p.orbitSpeed);
        }
        #endregion

        #region look at target
        //Returns the rotation which will make the camera look at the lookAtTarget
        private Quaternion GetTargetLookAtRotation()
        {
            Transform lookAtTransform = lookAtTarget.transform;
            return Quaternion.LookRotation((lookAtTransform.position + new Vector3(lookAtTransform.right.x, 0f, lookAtTransform.right.z).normalized * p.lookOffset) - cam.transform.position);
        }
        #endregion

        #region general
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
            if (Physics.Linecast(followTarget.transform.position, desiredPos, out RaycastHit hit, p.colliderLayerMask))
            {
                //return hit.point + (hit.normal * virtualCameraSphereRadius);
                return hit.point;
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