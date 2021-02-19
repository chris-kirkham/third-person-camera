//debug preprocessor define
#if UNITY_EDITOR //true if running in the editor
    #define TPC_CAMERA_MOVEMENT_DEBUG //hopefully this is specific enough that nobody else has defined it in anything
#elif DEVELOPMENT_BUILD //true if "development build" flag is checked in build options
    #define TPC_CAMERA_MOVEMENT_DEBUG
#endif

using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [RequireComponent(typeof(SharedCameraComponents))]
    [RequireComponent(typeof(DebugDrawing))]
    public class CollisionAvoidance : MonoBehaviour
    {
        //components
        private SharedCameraComponents components;
        private DebugDrawing debugDrawing;

        //trackers
        private float collisionMaxDistRelaxTime = 0f;

        //convenience variables
        private GameObject followTarget;
        private CameraParams camParams;

        private void Awake()
        {
            components = GetComponent<SharedCameraComponents>();
            debugDrawing = GetComponent<DebugDrawing>();
            
            camParams = components.cameraParams;
        }

        private void Update()
        {
            //UpdateMaxDistRelaxTime(Time.deltaTime);
        }

        /// <summary>Avoids camera clipping with geometry by jumping the camera towards the follow target if a collision is detected.</summary>
        /// <param name="initialNearClipPanePos">The camera's near clip pane centre point at the start of the update tick.</param>
        /// <param name="updatedCam">The camera after other movement has been applied.</param>
        /// <param name="virtualCameraSphereRadius">The radius of the camera's virtual size sphere.</param>
        /// <returns>The new camera position after collision avoidance has been applied. Returns updatedCam position if no collision detected.</returns>
        public Vector3 AvoidCollisions(Vector3 initialNearClipPanePos, Camera updatedCam, float virtualCameraSphereRadius)
        {
#if TPC_CAMERA_MOVEMENT_DEBUG
            debugDrawing.initialNearClipPanePos = initialNearClipPanePos;
            debugDrawing.updatedNearClipPanePos = updatedCam.GetNearClipPaneCentreWorld();
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
            HashSet<Collider> camColliders = GetCollidersBetweenCameraPositions(initialNearClipPanePos, updatedCam.GetNearClipPaneCentreWorld(), virtualCameraSphereRadius);
            if (camColliders.Count > 0)
            {
                Vector3 followTargetToNearClip = updatedCam.GetNearClipPaneCentreWorld() - followTargetPos;
                float minDistance = Mathf.Infinity;
                RaycastHit[] hits = Physics.SphereCastAll(followTargetPos, virtualCameraSphereRadius, followTargetToNearClip, followTargetToNearClip.magnitude, camParams.colliderLayerMask);
                foreach (RaycastHit hit in hits)
                {
                    if (camColliders.Contains(hit.collider) && hit.distance < minDistance)
                    {
                        //note: the hit.distance of a RaycastHit from a sphere cast is the distance to the centre of the sphere that hit something,
                        //not the distance to the hit point: https://answers.unity.com/questions/882631/how-to-get-the-center-of-a-capsulecast.html
                        minDistance = hit.distance;
                        Debug.DrawRay(followTargetPos, followTargetToNearClip.normalized * hit.distance, Color.red);
                    }
                }

                if (minDistance != Mathf.Infinity)
                {
                    Vector3 hitSphereCentre = followTargetPos + (followTargetToNearClip.normalized * minDistance);

#if TPC_CAMERA_MOVEMENT_DEBUG
                    debugDrawing.minHitDistance = minDistance;
                    debugDrawing.minHitSphereCentre = hitSphereCentre;
                    debugDrawing.AddCamCollisionPosition(hitSphereCentre - (updatedCam.transform.forward * updatedCam.nearClipPlane));
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
        private HashSet<Collider> GetCollidersBetweenCameraPositions(Vector3 initialNearClipPane, Vector3 updatedNearClipPane, float virtualCameraSphereRadius)
        {
            float overlapSphereInterval = 0.1f;
            Vector3 initialToUpdated = updatedNearClipPane - initialNearClipPane;
            Vector3 initialToUpdatedNorm = initialToUpdated.normalized;
            HashSet<Collider> overlaps = new HashSet<Collider>();

            //do overlap spheres at intervals along path from initial to updated near clip panes until collider(s) found
            for (float dist = 0; dist < initialToUpdated.magnitude; dist += overlapSphereInterval)
            {
                Debug.DrawRay(initialNearClipPane + (initialToUpdatedNorm * dist), Vector3.up * 0.1f, new Color(1, 1, 1, 0.5f));
                foreach (Collider c in Physics.OverlapSphere(initialNearClipPane + (initialToUpdatedNorm * dist), virtualCameraSphereRadius, camParams.colliderLayerMask))
                {
                    overlaps.Add(c);
                }

                if (overlaps.Count > 0) break;
            }

            //final overlap sphere exactly at updated clip pane pos if no colliders found
            if (overlaps.Count == 0)
            {
                Debug.DrawRay(updatedNearClipPane, Vector3.up * 0.1f, new Color(1, 1, 1, 0.5f));
                foreach (Collider c in Physics.OverlapSphere(updatedNearClipPane, virtualCameraSphereRadius, camParams.colliderLayerMask))
                {
                    overlaps.Add(c);
                }
            }

            return overlaps;
        }
    }
}