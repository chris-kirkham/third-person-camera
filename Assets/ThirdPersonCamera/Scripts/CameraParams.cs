using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    [CreateAssetMenu(fileName = "CameraParams", menuName = "ThirdPersonCamera/CameraParams")]
    public class CameraParams : ScriptableObject
    {
        /* Inspector parameters */
        #region camera mode
        public CameraBehaviourMode camMode = CameraBehaviourMode.Follow;
        [Min(0)] public float orbitToFollowHoldTime = 0f;
        public float orbitToFollowTransitionSpeed = 1f;
        public AnimationCurve transitionSpeedRamp;
        #endregion

        #region target follow
        [Header("Target following")]
        //interpolation - both front and rear offset
        public bool interpolateTargetFollow = true;

        //rear offset
        public Vector3 desiredOffset = Vector3.back;
        public float followSpeedOrientation = 1f;
        public float followSpeedHeight = 1f;
        public float followSpeedDistance = 1f;

        //front offset, for moving towards camera
        public bool allowMoveTowardsCamera = true;
        public Vector3 desiredFrontOffset = Vector3.forward;
        public float frontFollowSpeedOrientation = 1f;
        public float frontFollowSpeedHeight = 1f;
        public float frontFollowSpeedDistance = 1f;
        [Range(0, 90)] public float movingTowardsCameraAngleRange = 15f;

        //use world space offset (rear and front follow)
        public bool useWorldSpaceOffset = false; //false = transform offset to follow target's local space

        //max follow distance
        public bool useMinDistance = true;
        public float minDistanceFromTarget = 1f;
        public bool useMaxDistance = true;
        public float maxDistanceFromTarget = 10f;
        //public Vector3 minOffset = Vector3.back;
        //public Vector3 maxOffset = Vector3.back;

        //follow at specified height above target/ground
        public FollowHeightMode followHeightMode = FollowHeightMode.AboveTarget;
        public float desiredHeightAboveGround = 1f;
        public float aboveGroundFallbackHeight = 1f;
        [Min(0)] public float maxGroundDistance = 10f; //if the ground is more than this distance below the target, the camera will fall back on following relative to the target
        [Range(0, 90)] public float maxGroundSlopeAngle = 45f; //if the geometry below the camera slopes at a greater angle than this, it will not be treated as ground  
        public LayerMask groundLayerMask;
        #endregion

        #region orbit
        [Header("Target orbit")]
        public bool orbit = true;
        public float desiredOrbitDistance = 10f;
        public float orbitSpeed = 10f;
        public float orbitSpeedDistance = 10f;
        public float orbitSensitivity = 10f;
        public float minOrbitYAngle = -45f;
        public float maxOrbitYAngle = 45f;
        #endregion

        #region target look
        [Header("Target look")]
        public float lookOffset = 0f;
        public bool interpolateTargetLookAt = true;
        public float targetLookAtLerpSpeed = 1f;
        #endregion

        #region occlusion
        [Header("Occlusion avoidance")]
        public bool avoidFollowTargetOcclusion = true;
        public LayerMask occluderLayerMask;

        public float occlusionPullInSpeedHorizontal = 1f;
        public float occlusionPullInSpeedVertical = 1f;
        public float occlusionFollowSpeedIncrease = 1f;
        public float occlusionClipPanePadding = 0f;
        //public bool preserveCameraHeight = true;
        //public bool usePreviousTargetPositionsForCameraPullIn = true;

        //parameters for ramping up/down occlusion avoidance speedups based on how long the target has been occluded
        public bool useTimeInOcclusionMultiplier = true;
        public float maxTimeInOcclusionMultiplier = 10f;
        public float timeInOcclusionRampUpSpeed = 1f;
        public float timeInOcclusionRampDownSpeed = 1f;
        #endregion

        #region camera whiskers
        [Header("Camera whiskers")]
        public bool useCamWhiskers = true;
        public float whiskerPushStrength = 1f;
        [Min(2)] public int numWhiskers = 4;
        public float whiskerLength = 1f;
        [Range(0, 360)] public float whiskerSectorAngle = 180;
        #endregion

        #region collision avoidance
        [Header("Collision avoidance")]
        public bool avoidCollisionWithGeometry = true;
        public LayerMask colliderLayerMask;
        public float collisionMaxDistClampRelaxTime = 0f;
        #endregion

        #region update function
        public CameraUpdateFunction updateFunction = CameraUpdateFunction.LateUpdate;
        #endregion

        public CameraParams Clone()
        {
            return (CameraParams) this.MemberwiseClone();
        }

    }
}