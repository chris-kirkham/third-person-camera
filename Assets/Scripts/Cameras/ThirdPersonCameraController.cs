using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Lerps;

[RequireComponent(typeof(Camera))]
public class ThirdPersonCameraController : MonoBehaviour
{
    //Components
    private Camera cam;
    public GameObject followTarget;
    public GameObject lookAtTarget;
    
    //Inspector parameters
    [Header("Target following")]
    public Vector3 desiredOffset = Vector3.back;
    public Vector3 minOffset = Vector3.back;
    public Vector3 maxOffset = Vector3.back;
    public bool interpolateTargetFollowing = true;
    public float followSpeed = 1f;

    [Header("Target look")]
    public bool interpolateTargetLookAt = true;
    public float targetLookAtLerpSpeed = 1f;

    [Header("Occlusion avoidance")]
    public bool avoidFollowTargetOcclusion = true;
    public float occlusionPullInSpeed = 1f;
    public float occlusionIncreaseFollowSpeedMultiplier = 1f;
    public float occlusionClipPanePadding = 0f;
    public bool preserveCameraHeight = true;

    //parameters for easing in/out of occlusion avoidance speedups
    public bool useTimeInOcclusionMultiplier = true;
    private float currentTimeInOcclusionMult = 0f;
    public float maxTimeInOcclusionMultiplier = 10f;
    public float timeInOcclusionRampUpSpeed = 1f;
    public float timeInOcclusionRampDownSpeed = 1f;

    [Header("Camera whiskers")]
    public bool useCamWhiskers = true;
    public float whiskerPushStrength = 1f;
    [Min(2)] public int numWhiskers = 4;
    public float whiskerLength = 1f;
    [Range(0, 360)] public float whiskerSectorAngle = 180;

    [Header("Collision avoidance")]
    public bool avoidCollisionWithGeometry = true;
    public float collisionDetectionDistance = 1f;
    public float collisionPullInSpeed = 1f;

    [Header("Target orbit")]
    public bool orbit = true;
    public float orbitSpeed = 10f;
    public float orbitSensitivity = 10f;
    public float minOrbitYAngle = -45f;
    public float maxOrbitYAngle = 45f;

    //tracks current mouse x and y angles
    private float mouseX = 0f;
    private float mouseY = 0f;

    //Layer masks
    private int occluderLayers;
    private int colliderLayers;

    private void Awake()
    {
        cam = GetComponent<Camera>();

        occluderLayers = LayerMask.GetMask("LevelGeometrySolid");
        colliderLayers = LayerMask.GetMask("LevelGeometrySolid");
    }

    //all camera movement is done here
    private void FixedUpdate()
    {
        Vector3 currentCamPos = cam.transform.position;
        Vector3[] currentCamNearClipPaneCorners = cam.GetNearClipPaneCornersWorld();

        if(IsOccluded(occlusionClipPanePadding))
        {
            currentTimeInOcclusionMult = Mathf.Min(currentTimeInOcclusionMult + Time.fixedDeltaTime * timeInOcclusionRampUpSpeed, maxTimeInOcclusionMultiplier);
        }
        else
        {
            currentTimeInOcclusionMult = Mathf.Max(currentTimeInOcclusionMult - (Time.fixedDeltaTime * timeInOcclusionRampDownSpeed), 0f);
        }

        if(orbit)
        {
            cam.transform.position = OrbitTarget(Time.fixedDeltaTime);
        }
        else
        {
            cam.transform.position = GetFollowPosition(Time.fixedDeltaTime);
        }
        
        if(avoidFollowTargetOcclusion) cam.transform.position = GetOcclusionAvoidResult(Time.fixedDeltaTime);
        if(useCamWhiskers) cam.transform.position = GetCamWhiskerResult(cam.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime);
        
        if(interpolateTargetLookAt)
        {
            cam.transform.rotation = Quaternion.Lerp(cam.transform.rotation, GetTargetLookAtRotation(lookAtTarget.transform.position), Time.fixedDeltaTime * targetLookAtLerpSpeed);
        }
        else
        {
            cam.transform.rotation = GetTargetLookAtRotation(lookAtTarget.transform.position);
        }
        //cam.transform.rotation = GetTargetLookAtRotation(GetCamWhiskerResult(lookAtTarget.transform.position, GetCamWhiskerOffset(), Time.fixedDeltaTime));

        //if (avoidCollisionWithGeometry) 
        if (avoidCollisionWithGeometry)
        {
            cam.transform.position = AvoidCollisions(cam, collisionDetectionDistance, Time.fixedDeltaTime);
            //cam.transform.position = AvoidCollisionsContinuous(currentCamPos, currentCamNearClipPaneCorners, cam.transform.position, cam.GetNearClipPaneCornersWorld());
        }
    }

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
        if(Physics.Linecast(followTarget.transform.position, desiredPos, out RaycastHit hit, colliderLayers))
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

#region collision avoidance
    //Takes a camera position and avoids obstacles, if any, by instantly moving the camera position in front of the closest obstacle.
    private Vector3 AvoidCollisions(Camera cam, float collisionCheckDistance, float deltaTime)
    {
        Collider[] overlapColliders = new Collider[1];

        Vector3 camPos = cam.transform.position;
        Vector3 nearClipPaneCentre = camPos + cam.transform.forward * cam.nearClipPlane;
        Vector3 halfExtents = cam.GetNearClipPaneHalfExtents();
        Vector3 newCamPos = camPos;
        bool collisionHit = false;
        //RaycastHit hit;
        if (AvoidCollisionsRaycast(ref newCamPos, cam.transform.up, nearClipPaneCentre, halfExtents, collisionCheckDistance)) collisionHit = true; //up
        if (AvoidCollisionsRaycast(ref newCamPos, cam.transform.right, nearClipPaneCentre, halfExtents, collisionCheckDistance)) collisionHit = true; //right
        if (AvoidCollisionsRaycast(ref newCamPos, -cam.transform.up, nearClipPaneCentre, halfExtents, collisionCheckDistance)) collisionHit = true; //down
        if (AvoidCollisionsRaycast(ref newCamPos, -cam.transform.right, nearClipPaneCentre, halfExtents, collisionCheckDistance)) collisionHit = true; //left
        if (AvoidCollisionsRaycast(ref newCamPos, cam.transform.forward, nearClipPaneCentre, halfExtents, collisionCheckDistance)) collisionHit = true; //forward

        //if one of the collision raycasts hit, check if the camera is also occluded by the colliding geometry; if so, move it in front of the geometry;
        //else, pull camera forward to try to avoid the collision
        if (collisionHit)
        {
            if(Physics.Linecast(followTarget.transform.position, nearClipPaneCentre, out RaycastHit occlusionHit, colliderLayers))
            {
                Debug.Log("Camera collision solver: camera inside geometry");
                newCamPos = occlusionHit.point;
            }
            else //pull camera forward to try avoid collision
            {
                Vector3 targetPos = preserveCameraHeight ?
                    new Vector3(followTarget.transform.position.x, cam.transform.position.y, followTarget.transform.position.z)
                    : followTarget.transform.position;

                //Debug.DrawLine(camPos, targetPos, Color.black);
                newCamPos = Smoothstep(camPos, targetPos, deltaTime * collisionPullInSpeed);
            }
        }

        Debug.DrawLine(camPos, newCamPos, Color.green);
        return Smoothstep(camPos, newCamPos, 0.2f);
    }

    //Raycast function for collision avoidance; the main collision function casts six of these in each of the camera's cardinal directions.
    //If the raycast hits a collider on the collision layer(s), updates the ref camera position to avoid the collision, sets the ref RaycastHit to the collision hit,
    //and returns true, else returns false
    private bool AvoidCollisionsRaycast(ref Vector3 camPos, Vector3 dir, Vector3 nearClipPaneCentre, Vector3 nearClipPaneHalfExtents, float collisionCheckDistance)
    {
        if (Physics.Raycast(nearClipPaneCentre, dir, out RaycastHit hit, collisionCheckDistance, colliderLayers))
        {
            //Debug.DrawRay(nearClipPaneCentre, dir * (collisionCheckDistance + nearClipPaneHalfExtents.y), Color.green);
            camPos -= dir * (collisionCheckDistance - hit.distance);
            return true;
        }

        return false;
    }

    private Vector3 AvoidCollisionsContinuous(Vector3 currentCamPos, Vector3[] currentCamNearClipPaneCorners, Vector3 nextCamPos, Vector3[] nextCamNearClipPaneCorners)
    {
        float shortestDistanceToCollision = Mathf.Infinity;
        int shortestDistanceCornerIndex = -1;
        RaycastHit shortestDistanceHit = new RaycastHit();
        for(int i = 0; i < 4; i++)
        {
            Debug.DrawLine(currentCamNearClipPaneCorners[i], nextCamNearClipPaneCorners[i], new Color(1, 0, 1));
            if(Physics.Linecast(currentCamNearClipPaneCorners[i], nextCamNearClipPaneCorners[i], out RaycastHit hit, colliderLayers))
            {
                float dist = hit.distance;
                if(dist < shortestDistanceToCollision)
                {
                    shortestDistanceToCollision = dist;
                    shortestDistanceCornerIndex = i;
                    shortestDistanceHit = hit;
                }
            }
        }

        if(shortestDistanceCornerIndex != -1)
        {
            Vector3 cornerToCamPos = currentCamPos - currentCamNearClipPaneCorners[shortestDistanceCornerIndex];
            return shortestDistanceHit.point + cornerToCamPos;
        }

        return nextCamPos;
    }
    #endregion

#region cam whiskers
    //Casts "whisker" rays from the follow target
    private Vector3 GetCamWhiskerOffset()
    {
        Vector3 whiskerPushDir = Vector3.zero;
        Vector3 followTargetPos = followTarget.transform.position;
        float halfWhiskerSectorAngle = whiskerSectorAngle / 2;
        float angleInc = (halfWhiskerSectorAngle / numWhiskers) * 2;

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

    //Takes the desired camera position and adjusts it so the follow target isn't occluded by objects in the scene.
    //If the follow target would be occluded if the camera moved to desiredPos, this function moves desiredPos towards the follow target.
    //If there are no occluders in the way of the desired position, returns desiredPos unmodified
    private Vector3 GetOcclusionAvoidResult(float deltaTime)
    {
        Vector3 camPos = cam.transform.position;
        float maxDistance = Vector3.Distance(followTarget.transform.position, camPos);

        if (cam.LinecastsFromNearClipPane(followTarget.transform.position, out _, occluderLayers))
        //if(Physics.BoxCast(camPos, cam.GetNearClipPaneHalfExtents(), followTarget.transform.position - camPos, Quaternion.LookRotation(cam.transform.forward, Vector3.up), maxDistance, occluderLayers))
        {
            Vector3[] clipPanePoints = cam.GetNearClipPaneCornersWorld();
            foreach(Vector3 corner in clipPanePoints)
            {
                Debug.DrawRay(corner, followTarget.transform.position - camPos, Color.white);
            }

            Debug.DrawRay(camPos, (followTarget.transform.position - camPos).normalized * maxDistance, Color.cyan, 0.2f);
            
            Vector3 targetPos = preserveCameraHeight ?
                new Vector3(followTarget.transform.position.x, cam.transform.position.y, followTarget.transform.position.z)
                : followTarget.transform.position;

            Debug.DrawLine(camPos, targetPos, Color.green);
            return Smoothstep(camPos, targetPos, deltaTime * occlusionPullInSpeed);
        }

        return camPos;
    }

    private bool IsOccluded(float clipPanePadding = 0f)
    {
        //return cam.LinecastsFromNearClipPane(followTarget.transform.position, out _, occluderLayers, clipPanePadding);
        Vector3 dir = followTarget.transform.position - cam.transform.position;
        return cam.RaycastsFromNearClipPane(dir, out _, dir.magnitude, occluderLayers, clipPanePadding);
    }

    //Takes the camera's desired move direction and checks if moving to it would cause the follow target to be occluded.
    //If so, returns the move direction shortened to stop before the occluding object, else returns the same move direction vector
    private Vector3 ShortenMoveDirectionIfMovingIntoOcclusion(Vector3 camMoveDirection, float collisionSphereCheckRadius)
    {
        //if moving to desired orbit position would cause the camera to move into occlusion, shorten offset to before that happens
        if (Physics.SphereCast(followTarget.transform.position, collisionSphereCheckRadius, camMoveDirection, out RaycastHit hit, camMoveDirection.magnitude, occluderLayers))
        {
            return followTarget.transform.position + (camMoveDirection.normalized * (Vector3.Distance(followTarget.transform.position, hit.point) - collisionSphereCheckRadius));
        }
    
        return followTarget.transform.position + camMoveDirection;
    }

    private Vector3 ShortenFollowDistanceToAvoidRearCollision(Vector3 desiredPos)
    {
        if (Physics.Linecast(followTarget.transform.position, desiredPos, out RaycastHit hit, colliderLayers))
        {
            //return hit.point + (hit.normal * COLLISION_SPHERE_CHECK_RADIUS);
            return hit.point;
        }
        else
        {
            return desiredPos;
        }
    }

    private Vector3 OrbitTarget(float deltaTime)
    {
        Vector3 camPos = cam.transform.position;

        mouseX += Input.GetAxis("Mouse X") * orbitSensitivity;
        mouseY -= Input.GetAxis("Mouse Y") * orbitSensitivity;
        mouseX = Orbit_ClampMouseAngle(mouseX);
        mouseY = Mathf.Clamp(Orbit_ClampMouseAngle(mouseY), minOrbitYAngle, maxOrbitYAngle);

        Quaternion rotation = Quaternion.Euler(mouseY, mouseX, 0f);
        Vector3 desiredPos = followTarget.transform.position + rotation * desiredOffset;
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

        return Vector3.Lerp(camPos, ShortenFollowDistanceToAvoidRearCollision(desiredPos), deltaTime * orbitSpeed);
        //return Vector3.Lerp(camPos, ShortenMoveDirectionIfMovingIntoOcclusion(rotation * desiredOffset, collisionDetectionDistance), deltaTime * orbitSpeed);
    }

    private float Orbit_ClampMouseAngle(float mouseAngle)
    {
        if (mouseAngle < -360) return mouseAngle + 360;
        if (mouseAngle >= 360) return mouseAngle - 360;
        return mouseAngle;
    }

    //Returns the rotation which will make the camera look at the lookAtTarget
    private Quaternion GetTargetLookAtRotation(Vector3 lookAtTargetPos)
    {
        return Quaternion.LookRotation(lookAtTargetPos - cam.transform.position);
    }
}
