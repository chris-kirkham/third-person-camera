using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using static UnityEditor.EditorGUILayout;

[CustomEditor(typeof(ThirdPersonCameraController))]
public class ThirdPersonCameraControllerInspector : Editor
{
    private ThirdPersonCameraController cameraController;

    //foldout bools
    private bool showTargetComponents = false;
    private bool showTargetFollowParams = false;
    private bool showTargetLookParams = false;
    private bool showOcclusionAvoidanceParams = false;
    private bool showCamWhiskerParams = false;
    private bool showCollisionAvoidanceParams = false;
    private bool showTargetOrbitParams = false;

    public override void OnInspectorGUI()
    {
        //base.OnInspectorGUI();
        
        cameraController = (ThirdPersonCameraController) target;

        DoTargetComponents(cameraController);
        DoTargetFollowing(cameraController);
        DoTargetLook(cameraController);
        DoOcclusionAvoidance(cameraController);
        DoCamWhiskers(cameraController);
        DoCollisionAvoidance(cameraController);
        DoTargetOrbit(cameraController);
    }

    private void DoTargetComponents(ThirdPersonCameraController t)
    {
        showTargetComponents = FoldoutHeader(showTargetComponents, "Follow/look targets");
        if(showTargetComponents)
        {
            t.followTarget = (GameObject) ObjectField("Follow target", t.followTarget, typeof(GameObject), true);
            t.lookAtTarget = (GameObject) ObjectField("Look target", t.lookAtTarget, typeof(GameObject), true);
        }
    }

    private void DoTargetFollowing(ThirdPersonCameraController t)
    {
        showTargetFollowParams = FoldoutHeader(showTargetFollowParams, "Target follow");
        if(showTargetFollowParams)
        {
            t.desiredOffset = Vector3Field("Desired offset", t.desiredOffset); 
            t.minOffset = Vector3Field("Min offset", t.minOffset); 
            t.maxOffset = Vector3Field("Max offset", t.maxOffset);
            t.interpolateTargetFollowing = Toggle("Interpolate", t.interpolateTargetFollowing);
            if(t.interpolateTargetFollowing)
            {
                t.followSpeed = FloatField("Follow speed", t.followSpeed);
            }
        }
    }

    private void DoTargetLook(ThirdPersonCameraController t)
    {
        showTargetLookParams = FoldoutHeader(showTargetLookParams, "Target look");
        if(showTargetLookParams)
        {
            t.interpolateTargetLookAt = Toggle("Interpolate", t.interpolateTargetLookAt);
            if(t.interpolateTargetLookAt)
            {
                t.targetLookAtLerpSpeed = FloatField("Look at speed", t.targetLookAtLerpSpeed);
            }
        }
    }

    private void DoOcclusionAvoidance(ThirdPersonCameraController t)
    {
        showOcclusionAvoidanceParams = FoldoutHeader(showOcclusionAvoidanceParams, "Occlusion avoidance");
        if(showOcclusionAvoidanceParams)
        {
            t.avoidFollowTargetOcclusion = Toggle("Avoid occlusion", t.avoidFollowTargetOcclusion);
            if(t.avoidFollowTargetOcclusion)
            {
                t.usePreviousTargetPositionsForCameraPullIn = Toggle("Follow previous target positions for pull-in", t.usePreviousTargetPositionsForCameraPullIn);
                t.occlusionPullInSpeedHorizontal = FloatField("Pull-in speed horizontal", t.occlusionPullInSpeedHorizontal);
                t.occlusionPullInSpeedVertical = FloatField("Pull-in speed vertical", t.occlusionPullInSpeedVertical);
                t.occlusionIncreaseFollowSpeedMultiplier = FloatField("Increase follow speed", t.occlusionIncreaseFollowSpeedMultiplier);
                t.occlusionClipPanePadding = FloatField("Near clip pane padding", t.occlusionClipPanePadding);
                t.preserveCameraHeight = Toggle("Preserve camera height", t.preserveCameraHeight);
                t.useTimeInOcclusionMultiplier = Toggle("Ease in/out of occlusion avoidance", t.useTimeInOcclusionMultiplier);
                if(t.useTimeInOcclusionMultiplier)
                {
                    t.timeInOcclusionRampUpSpeed = FloatField("Ease in speed", t.timeInOcclusionRampUpSpeed);
                    t.timeInOcclusionRampDownSpeed = FloatField("Ease out speed", t.timeInOcclusionRampDownSpeed);
                }
            }
        }
    }

    private void DoCamWhiskers(ThirdPersonCameraController t)
    {
        showCamWhiskerParams = FoldoutHeader(showCamWhiskerParams, "Camera whiskers");
        if(showCamWhiskerParams)
        {
            t.useCamWhiskers = Toggle("Use camera whiskers", t.useCamWhiskers);
            if(t.useCamWhiskers)
            {
                t.whiskerPushStrength = FloatField("Push strength", t.whiskerPushStrength);
                t.numWhiskers = IntField("Number of whiskers", t.numWhiskers);
                t.whiskerLength = FloatField("Whisker length", t.whiskerLength);
                t.whiskerSectorAngle = Slider("Whisker angle", t.whiskerSectorAngle, 0f, 360f);
            }
        }
    }

    private void DoCollisionAvoidance(ThirdPersonCameraController t)
    {
        showCollisionAvoidanceParams = FoldoutHeader(showCollisionAvoidanceParams, "Collision avoidance");
        if(showCollisionAvoidanceParams)
        {
            t.avoidCollisionWithGeometry = Toggle("Avoid collisions", t.avoidCollisionWithGeometry);
            if(t.avoidCollisionWithGeometry)
            {
                t.collisionDetectionDistance = FloatField("Collision detection distance", t.collisionDetectionDistance);
                t.collisionPullInSpeed = FloatField("Pull-in speed", t.collisionPullInSpeed);
            }
        }
    }

    private void DoTargetOrbit(ThirdPersonCameraController t)
    {
        showTargetOrbitParams = FoldoutHeader(showTargetOrbitParams, "Target orbit");
        if(showTargetOrbitParams)
        {
            t.orbit = Toggle("Orbit target", t.orbit);
            if(t.orbit)
            {
                t.orbitSpeed = FloatField("Speed", t.orbitSpeed);
                t.orbitSensitivity = FloatField("Sensitivity", t.orbitSensitivity);
                t.minOrbitYAngle = FloatField("Min Y angle", t.minOrbitYAngle);
                t.maxOrbitYAngle = FloatField("Max Y angle", t.maxOrbitYAngle);
            }
        }
    }

    #region convenience inspector functions
    private bool FoldoutHeader(bool foldout, string label)
    {
        return Foldout(foldout, label, true, EditorStyles.foldoutHeader);
    }
    #endregion
}
