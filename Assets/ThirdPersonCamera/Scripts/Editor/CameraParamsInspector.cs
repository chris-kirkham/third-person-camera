using UnityEngine;
using UnityEditor;
using static UnityEditor.EditorGUILayout;
using UnityEditorInternal;
using ThirdPersonCamera;

[CustomEditor(typeof(CameraParams))]
public class CameraParamsInspector : Editor
{
    private CameraParams cameraParams;

    //foldout bools
    private bool showCameraModeParams = false;
    private bool showTargetFollowParams = false;
    private bool showTargetLookParams = false;
    private bool showOcclusionAvoidanceParams = false;
    private bool showCamWhiskerParams = false;
    private bool showCollisionAvoidanceParams = false;
    private bool showTargetOrbitParams = false;
    private bool showLayerMaskParams = false;

    public override void OnInspectorGUI()
    {
        //base.OnInspectorGUI();
        
        cameraParams = (CameraParams) target;

        DoCameraMode(cameraParams);
        DoTargetFollowing(cameraParams);
        DoTargetLook(cameraParams);
        DoOcclusionAvoidance(cameraParams);
        DoCamWhiskers(cameraParams);
        DoCollisionAvoidance(cameraParams);
        DoTargetOrbit(cameraParams);
        DoLayerMasks(cameraParams);

        EditorUtility.SetDirty(target);
    }

    private void DoCameraMode(CameraParams t)
    {
        showCameraModeParams = FoldoutHeader(showCameraModeParams, "Camera behaviour mode");
        if(showCameraModeParams)
        {
            t.camMode = (CameraBehaviourMode) EnumPopup("Camera mode", t.camMode);

            if(t.camMode == CameraBehaviourMode.FollowAndOrbit)
            {
                t.orbitToFollowHoldTime = FloatField("Transition delay", t.orbitToFollowHoldTime);
                t.orbitToFollowTransitionTime = FloatField("Transition time", t.orbitToFollowTransitionTime);
            }
        }
    }

    private void DoTargetFollowing(CameraParams t)
    {
        showTargetFollowParams = FoldoutHeader(showTargetFollowParams, "Target follow");
        if(showTargetFollowParams)
        {
            t.desiredOffset = Vector3Field("Desired offset", t.desiredOffset);
            if (t.interpolateTargetFollow)
            {
                t.followSpeed = FloatField("Follow speed", t.followSpeed);
            }
            
            t.useWorldSpaceOffset = Toggle(new GUIContent("Use world space offset?", "By default, the offset is relative to the follow target's local space." +
                " Set this to true to use world space offset."), t.useWorldSpaceOffset);

            t.allowMoveTowardsCamera = Toggle(new GUIContent("Target can move towards camera?", "Use the desired front offset when the follow target is facing the camera. Typically," +
                                "this is used to let the camera stay in front of the target when the target is moving towards it," +
                                " rather than trying to reorient behind the target."), t.allowMoveTowardsCamera);
            
            if (t.allowMoveTowardsCamera)
            {
                t.desiredFrontOffset = Vector3Field("Desired front offset", t.desiredFrontOffset);
                if(t.interpolateTargetFollow) t.frontFollowSpeed = FloatField("Front follow speed", t.frontFollowSpeed);
                t.movingTowardsCameraAngleRange = Slider("Angle range", t.movingTowardsCameraAngleRange, 0, 90);
            }


            //t.followHeightMode = (ThirdPersonCameraController.FollowHeightMode) EnumPopup("Follow height mode", t.followHeightMode);
            //t.minOffset = Vector3Field("Min offset", t.minOffset); //not currently used
            //t.maxOffset = Vector3Field("Max offset", t.maxOffset); //not currently used
            t.interpolateTargetFollow = Toggle("Interpolate", t.interpolateTargetFollow);
            
            t.useMaxDistance = Toggle("Clamp camera distance from target", t.useMaxDistance);
            if (t.useMaxDistance)
            {
                t.maxDistanceFromTarget = FloatField("Max distance", t.maxDistanceFromTarget);
            }
        }
    }

    private void DoTargetLook(CameraParams t)
    {
        showTargetLookParams = FoldoutHeader(showTargetLookParams, "Target look");
        if(showTargetLookParams)
        {
            t.lookOffset = FloatField("Look offset", t.lookOffset);
            t.interpolateTargetLookAt = Toggle("Interpolate", t.interpolateTargetLookAt);
            if(t.interpolateTargetLookAt)
            {
                t.targetLookAtLerpSpeed = FloatField("Look at speed", t.targetLookAtLerpSpeed);
            }
        }
    }

    private void DoOcclusionAvoidance(CameraParams t)
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
                t.occlusionFollowSpeedIncrease = FloatField("Increase follow speed", t.occlusionFollowSpeedIncrease);
                t.occlusionClipPanePadding = FloatField("Near clip pane padding", t.occlusionClipPanePadding);
                //t.preserveCameraHeight = Toggle("Preserve camera height", t.preserveCameraHeight);
                t.useTimeInOcclusionMultiplier = Toggle("Ease in/out of occlusion avoidance", t.useTimeInOcclusionMultiplier);
                if(t.useTimeInOcclusionMultiplier)
                {
                    t.maxTimeInOcclusionMultiplier = FloatField("Max speed increase", t.maxTimeInOcclusionMultiplier);
                    t.timeInOcclusionRampUpSpeed = FloatField("Ease in speed", t.timeInOcclusionRampUpSpeed);
                    t.timeInOcclusionRampDownSpeed = FloatField("Ease out speed", t.timeInOcclusionRampDownSpeed);
                }
            }
        }
    }

    private void DoCamWhiskers(CameraParams t)
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

    private void DoCollisionAvoidance(CameraParams t)
    {
        showCollisionAvoidanceParams = FoldoutHeader(showCollisionAvoidanceParams, "Collision avoidance");
        if(showCollisionAvoidanceParams)
        {
            t.avoidCollisionWithGeometry = Toggle("Avoid collisions", t.avoidCollisionWithGeometry);
        }
    }

    private void DoTargetOrbit(CameraParams t)
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

    private void DoLayerMasks(CameraParams t)
    {
        showLayerMaskParams = FoldoutHeader(showLayerMaskParams, "Layer masks");
        if(showLayerMaskParams)
        {
            //https://answers.unity.com/questions/42996/how-to-create-layermask-field-in-a-custom-editorwi.html
            //just doing t.whateverMask = MaskField("..", t.colliderLayerMask, InternalEditorUtility.layers) makes the actual mask value one up from whatever the inspector shows, for some reason
            LayerMask colliderLayers = MaskField("Camera colliders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.colliderLayerMask), InternalEditorUtility.layers);
            t.colliderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(colliderLayers);
            LayerMask occluderLayers = MaskField("Camera occluders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.occluderLayerMask), InternalEditorUtility.layers);
            t.occluderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(occluderLayers);
        }
    }

    #region convenience inspector functions
    private bool FoldoutHeader(bool foldout, string label)
    {
        return Foldout(foldout, label, true, EditorStyles.foldoutHeader);
    }
    #endregion
}
