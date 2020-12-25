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
        public float orbitToFollowTransitionTime = 1f;
        #endregion

        #region target follow parameters
        [Header("Target following")]
        //rear offset
        public Vector3 desiredOffset = Vector3.back;
        public float followSpeed = 1f;

        //front offset, for moving towards camera
        public bool allowMoveTowardsCamera = true;
        public Vector3 desiredFrontOffset = Vector3.forward;
        public float frontFollowSpeed = 1f;
        [Range(0, 90)] public float movingTowardsCameraAngleRange = 15f;

        //use world space offset (rear and front follow)
        public bool useWorldSpaceOffset = false; //false = transform offset to follow target's local space

        //max follow distance
        public bool useMaxDistance = true;
        public float maxDistanceFromTarget = 10f;
        //public Vector3 minOffset = Vector3.back;
        //public Vector3 maxOffset = Vector3.back;

        public enum FollowHeightMode { AboveTarget, AboveGround };
        public FollowHeightMode followHeightMode = FollowHeightMode.AboveTarget; //TODO: IMPLEMENT

        public bool interpolateTargetFollow = true;
        #endregion

        #region target look parameters
        [Header("Target look")]
        public float lookOffset = 0f;
        public bool interpolateTargetLookAt = true;
        public float targetLookAtLerpSpeed = 1f;
        #endregion

        #region occlusion parameters
        [Header("Occlusion avoidance")]
        public bool avoidFollowTargetOcclusion = true;
        public float occlusionPullInSpeedHorizontal = 1f;
        public float occlusionPullInSpeedVertical = 1f;
        public float occlusionFollowSpeedIncrease = 1f;
        public float occlusionClipPanePadding = 0f;
        //public bool preserveCameraHeight = true;

        //parameters for ramping up/down occlusion avoidance speedups based on how long the target has been occluded
        public bool useTimeInOcclusionMultiplier = true;
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
        #endregion

        #region orbit parameters
        [Header("Target orbit")]
        public bool orbit = true;
        public float orbitSpeed = 10f;
        public float orbitSensitivity = 10f;
        public float minOrbitYAngle = -45f;
        public float maxOrbitYAngle = 45f;
        #endregion

        #region parameters affecting more than one camera behaviour
        public bool usePreviousTargetPositionsForCameraPullIn = true;
        #endregion

        #region layer masks
        public LayerMask occluderLayerMask;
        public LayerMask colliderLayerMask;
        #endregion

    }
}