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
        #endregion

        /* Inspector parameters */
        #region camera mode
        public enum CameraBehaviourMode { Follow, Orbit, FollowAndOrbit };
        public CameraBehaviourMode camMode = CameraBehaviourMode.Follow;
        #endregion

        #region target follow parameters
        [Header("Target following")]
        public Vector3 desiredOffset = Vector3.back;
        public enum FollowHeightMode { AboveTarget, AboveGround };
        public FollowHeightMode followHeightMode = FollowHeightMode.AboveTarget; //TODO: IMPLEMENT
                                                                                 
        //public Vector3 minOffset = Vector3.back;
        //public Vector3 maxOffset = Vector3.back;
        public bool interpolateTargetFollowing = true;
        public float followSpeed = 1f;
        #endregion

        #region target look parameters
        [Header("Target look")]
        public bool interpolateTargetLookAt = true;
        public float targetLookAtLerpSpeed = 1f;
        #endregion

        #region occlusion parameters
        [Header("Occlusion avoidance")]
        public bool avoidFollowTargetOcclusion = true;
        public float occlusionPullInSpeedHorizontal = 1f;
        public float occlusionPullInSpeedVertical = 1f;
        public float occlusionIncreaseFollowSpeedMultiplier = 1f;
        public float occlusionClipPanePadding = 0f;
        public bool preserveCameraHeight = true;

        //parameters for easing in/out of occlusion avoidance speedups
        public bool useTimeInOcclusionMultiplier = true;
        private float currentTimeInOcclusionMult = 0f;
        public float maxTimeInOcclusionMultiplier = 10f;
        public float timeInOcclusionRampUpSpeed = 1f;
        public float timeInOcclusionRampDownSpeed = 1f;
        #endregion

        #region camera whisker parameters
        [Header("Camera whiskers")]
        public bool useCamWhiskers = true;
        public float whiskerPushStrength = 1f;
        [Min(2)] public int numWhiskers = 4;
        public float whiskerLength = 1f;
        [Range(0, 360)] public float whiskerSectorAngle = 180;
        #endregion

        #region collision avoidance parameters
        [Header("Collision avoidance")]
        public bool avoidCollisionWithGeometry = true;
        public float collisionDetectionDistance = 1f;
        public float collisionPullInSpeed = 1f;
        #endregion

        #region orbit parameters
        [Header("Target orbit")]
        public bool orbit = true;
        public float orbitSpeed = 10f;
        public float orbitSensitivity = 10f;
        public float minOrbitYAngle = -45f;
        public float maxOrbitYAngle = 45f;

        //tracks current mouse x and y angles
        private float mouseX = 0f;
        private float mouseY = 0f;
        #endregion

        //previous target positions array
        public bool usePreviousTargetPositionsForCameraPullIn = true;
        private Vector3[] previousTargetPositions;

        //virtual camera size
        private float virtualCameraSphereRadius;

        /* Layer masks */
        private int occluderLayers;
        private int colliderLayers;

        private void Awake()
        {
            cam = GetComponent<Camera>();
            input = GetComponent<CameraControllerInput>();
            stateController = GetComponent<CameraStateController>();

            virtualCameraSphereRadius = cam.GetNearClipPaneCentreToCornerDistance();

            occluderLayers = LayerMask.GetMask("LevelGeometrySolid");
            colliderLayers = LayerMask.GetMask("LevelGeometrySolid");
        }

        //all camera movement is done here
        private void FixedUpdate()
        {
            Vector3 currentCamPos = cam.transform.position;
            Vector3[] currentCamNearClipPaneCorners = cam.GetNearClipPaneCornersWorld();

            if (IsOccluded(occlusionClipPanePadding))
            {
                currentTimeInOcclusionMult = Mathf.Min(currentTimeInOcclusionMult + Time.fixedDeltaTime * timeInOcclusionRampUpSpeed, maxTimeInOcclusionMultiplier);
            }
            else
            {
                currentTimeInOcclusionMult = Mathf.Max(currentTimeInOcclusionMult - Time.fixedDeltaTime * timeInOcclusionRampDownSpeed, 0f);
            }

            if (orbit)
            {
                cam.transform.position = OrbitTarget(Time.fixedDeltaTime, input.GetOrbitAngles());
            }
            else
            {
                cam.transform.position = GetFollowPosition(Time.fixedDeltaTime);
            }

            if (avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionAvoidResult(Time.fixedDeltaTime);
            if (useCamWhiskers) cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime);

            if (interpolateTargetLookAt)
            {
                cam.transform.rotation = Quaternion.Lerp(cam.transform.rotation, GetTargetLookAtRotation(lookAtTarget.transform.position), Time.fixedDeltaTime * targetLookAtLerpSpeed);
            }
            else
            {
                cam.transform.rotation = GetTargetLookAtRotation(lookAtTarget.transform.position);
            }
            //cam.transform.rotation = GetTargetLookAtRotation(GetCamWhiskerResult(lookAtTarget.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime));

            if (avoidCollisionWithGeometry)
            {
                cam.transform.position = AvoidCollisions_JumpForward(cam);
            }
        }

        #region follow target
        //Returns the desired target follow position as specified by the target's position and the target follow offset params.
        //This is the "ideal" position the camera wants to be in, before taking into consideration obstacle/occlusion avoidance etc.
        private Vector3 GetDesiredFollowPosition(Vector3 desiredOffset)
        {
            return followTarget.transform.position + followTarget.transform.TransformDirection(desiredOffset);
        }

        //Returns the camera position after interpolating between the current camera position and the desired follow position, 
        //shortening the desired follow offset if it would be occluded
        private Vector3 GetFollowPosition(float deltaTime)
        {
            Vector3 desiredPos = GetDesiredFollowPosition(desiredOffset);
            Debug.DrawLine(cam.transform.position, desiredPos, Color.red);

            Vector3 newCamPos;
            if (Physics.Linecast(followTarget.transform.position, desiredPos, out RaycastHit hit, colliderLayers))
            {
                //newCamPos = hit.point + (hit.normal * COLLISION_SPHERE_CHECK_RADIUS);
                newCamPos = hit.point;
            }
            else
            {
                newCamPos = desiredPos;
            }

            Debug.DrawLine(cam.transform.position, newCamPos, Color.yellow);

            if (useTimeInOcclusionMultiplier && currentTimeInOcclusionMult > 0)
            {
                return Smoothstep(cam.transform.position, newCamPos, deltaTime * followSpeed * Mathf.Max(1, occlusionIncreaseFollowSpeedMultiplier * currentTimeInOcclusionMult));
            }
            else
            {
                return Smoothstep(cam.transform.position, newCamPos, deltaTime * followSpeed);
            }
        }
        #endregion

        #region collision avoidance
        //Avoids camera clipping with geometry by jumping the camera forward if any of its near clip pane corners are inside geometry
        private Vector3 AvoidCollisions_JumpForward(Camera cam)
        {
            //Check if the camera's virtual size sphere is intersecting with geometry
            Collider[] colliders = new Collider[1];
            //if (Physics.OverlapSphereNonAlloc(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius, colliders) > 0)
            colliders = Physics.OverlapSphere(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius);
            if (colliders.Length > 0)
            {
                foreach(Collider c in colliders)
                {
                    Debug.DrawLine(cam.transform.position, c.transform.position, new Color(1, 0.25f, 1));
                }

                //check if any of the camera's near clip pane corners are inside geometry
                Vector3[] nearClipPaneCorners = cam.GetNearClipPaneCornersWorld();
                float longestDistInsideGeometry = 0f;
                int longestDistCornerIndex = -1;
                Vector3 longestDistHitPoint = Vector3.zero;
                RaycastHit hit;
                foreach(Collider c in colliders)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Vector3 followTargetToClipCorner = nearClipPaneCorners[i] - followTarget.transform.position;
                        if (c.Raycast(new Ray(followTarget.transform.position, followTargetToClipCorner), out hit, followTargetToClipCorner.magnitude))
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
                }

                /*
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
                */

                //if any clip pane corners are inside geometry, move the camera such that the point deepest inside geometry is touching its ray hit point 
                if (longestDistCornerIndex != -1)
                {
                    Debug.DrawRay(longestDistHitPoint, cam.transform.position - nearClipPaneCorners[longestDistCornerIndex], new Color(1, 0, 1));
                    return longestDistHitPoint + (cam.transform.position - nearClipPaneCorners[longestDistCornerIndex]);
                }
            }

            return cam.transform.position;
        }
        #endregion

        #region cam whiskers
        //Casts "whisker" rays from the follow target
        private Vector3 GetCamWhiskerOffset()
        {
            Vector3 whiskerPushDir = Vector3.zero;
            Vector3 followTargetPos = followTarget.transform.position;
            float halfWhiskerSectorAngle = whiskerSectorAngle / 2;
            float angleInc = halfWhiskerSectorAngle / numWhiskers * 2;

            for (float angle = -halfWhiskerSectorAngle; angle <= halfWhiskerSectorAngle; angle += angleInc)
            {
                Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * -followTarget.transform.forward;
                Debug.DrawRay(followTargetPos, dir * whiskerLength, Color.HSVToRGB(Mathf.Abs(angle) / whiskerSectorAngle, 0.2f, 1f)); //visualise raycasts
                if (Physics.Raycast(followTargetPos, dir, whiskerLength)) whiskerPushDir -= new Vector3(dir.x, 0f, dir.z);
            }

            //zero local z (is there a better way to do this?)
            whiskerPushDir = cam.transform.InverseTransformDirection(whiskerPushDir);
            whiskerPushDir.z = 0;
            whiskerPushDir = cam.transform.TransformDirection(whiskerPushDir);
            Debug.DrawRay(followTargetPos, whiskerPushDir, Color.yellow);

            return whiskerPushDir / numWhiskers;
        }

        private Vector3 GetCamWhiskerResult(Vector3 camPos, Vector3 whiskerOffset, float deltaTime)
        {
            return Smoothstep(camPos, camPos + whiskerOffset, whiskerPushStrength * deltaTime);
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

            if (IsOccluded(occlusionClipPanePadding))
            {
                Debug.DrawRay(camPos, (followTarget.transform.position - camPos).normalized * maxDistance, Color.cyan);
                return PullInCamera(camPos, occlusionPullInSpeedVertical, occlusionPullInSpeedHorizontal, deltaTime);
            }

            return camPos;
        }

        private bool IsOccluded(float clipPanePadding = 0f)
        {
            Vector3 dir = followTarget.transform.position - cam.transform.position;

            //Debug drawing
            foreach (Vector3 clipPaneCorner in cam.GetNearClipPaneCornersWorld(clipPanePadding))
            {
                Debug.DrawRay(clipPaneCorner, dir, Color.white);
            }
            //? if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers)) ...
            return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, occluderLayers, clipPanePadding);
        }
        #endregion

        #region orbit
        private Vector3 OrbitTarget(float deltaTime, Vector2 orbitAngles)
        {
            Vector2 clampedAngles = new Vector2(orbitAngles.x, Mathf.Clamp(orbitAngles.y, minOrbitYAngle, maxOrbitYAngle));
            Quaternion rotation = Quaternion.Euler(clampedAngles.y, clampedAngles.x, 0f);
            Vector3 desiredPos = followTarget.transform.position + rotation * desiredOffset;

            return Vector3.Lerp(cam.transform.position, ShortenFollowDistanceToAvoidRearCollision(desiredPos), deltaTime * orbitSpeed);
        }

        private float Orbit_ClampMouseAngle(float mouseAngle)
        {
            if (mouseAngle < -360) return mouseAngle + 360;
            if (mouseAngle >= 360) return mouseAngle - 360;
            return mouseAngle;
        }
        #endregion

        #region look at target
        //Returns the rotation which will make the camera look at the lookAtTarget
        private Quaternion GetTargetLookAtRotation(Vector3 lookAtTargetPos)
        {
            return Quaternion.LookRotation(lookAtTargetPos - cam.transform.position);
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
            if (Physics.Linecast(followTarget.transform.position, desiredPos, out RaycastHit hit, colliderLayers))
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
                Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
                Gizmos.DrawWireSphere(cam.transform.position + cam.transform.forward * cam.nearClipPlane, virtualCameraSphereRadius);
            }
        }

    }
}