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
    /* NOTE:
     * Serialising these and making them static is probably the simplest way to get foldout states to persist through selection/deselection -
     * however, it means every instance of this inspector will have the same foldout states, and it seems to reset on play anyway...
     */
    [SerializeField] private static bool showCameraModeParams = false;
    [SerializeField] private static bool showTargetFollowParams = false;
    [SerializeField] private static bool showTargetLookParams = false;
    [SerializeField] private static bool showOcclusionAvoidanceParams = false;
    [SerializeField] private static bool showCamWhiskerParams = false;
    [SerializeField] private static bool showCollisionAvoidanceParams = false;
    [SerializeField] private static bool showTargetOrbitParams = false;
    [SerializeField] private static bool showLayerMaskParams = false;
    [SerializeField] private static bool showCamUpdateFunctionParams = false;

    public override void OnInspectorGUI()
    {
        cameraParams = (CameraParams) target;

        DoCameraMode(cameraParams);
        DoTargetFollowing(cameraParams);
        DoTargetLook(cameraParams);
        DoOcclusionAvoidance(cameraParams);
        DoCamWhiskers(cameraParams);
        DoCollisionAvoidance(cameraParams);
        DoTargetOrbit(cameraParams);
        DoLayerMasks(cameraParams);
        DoCamUpdateFunction(cameraParams);

        EditorUtility.SetDirty(target);
    }

    private void DoCameraMode(CameraParams t)
    {
        showCameraModeParams = FoldoutHeader(showCameraModeParams, "Camera behaviour mode");
        if(showCameraModeParams)
        {
            EditorGUI.indentLevel++;

            t.camMode = (CameraBehaviourMode) EnumPopup("Camera mode", t.camMode);

            if(t.camMode == CameraBehaviourMode.FollowAndOrbit)
            {
                EditorGUI.indentLevel++;

                t.orbitToFollowHoldTime = FloatField("Transition hold time", t.orbitToFollowHoldTime);
                t.orbitToFollowTransitionSpeed = FloatField("Transition speed", t.orbitToFollowTransitionSpeed);
                
                EditorGUI.indentLevel--;
            }

            EditorGUI.indentLevel--;
        }
    }

    private void DoTargetFollowing(CameraParams t)
    {
        showTargetFollowParams = FoldoutHeader(showTargetFollowParams, "Target follow");
        if(showTargetFollowParams)
        {
            EditorGUI.indentLevel++;

            t.interpolateTargetFollow = Toggle("Interpolate", t.interpolateTargetFollow);
            
            t.desiredOffset = Vector3Field("Desired offset", t.desiredOffset);
            if (t.interpolateTargetFollow)
            {
                t.followSpeed = FloatField("Follow speed", t.followSpeed);
            }
            
            /*
            t.useWorldSpaceOffset = Toggle(new GUIContent("Use world space offset?", "By default, the offset is relative to the follow target's local space." +
                " Set this to true to use world space offset."), t.useWorldSpaceOffset);
            */

            t.allowMoveTowardsCamera = Toggle(new GUIContent("Target can move towards camera?", "Use the desired front offset when the follow target is facing the camera. Typically," +
                                "this is used to let the camera stay in front of the target when the target is moving towards it," +
                                " rather than trying to reorient behind the target."), t.allowMoveTowardsCamera);
            
            if (t.allowMoveTowardsCamera)
            {
                EditorGUI.indentLevel++;
                t.desiredFrontOffset = Vector3Field("Desired front offset", t.desiredFrontOffset);
                if(t.interpolateTargetFollow) t.frontFollowSpeed = FloatField("Front follow speed", t.frontFollowSpeed);
                t.movingTowardsCameraAngleRange = Slider("Angle range", t.movingTowardsCameraAngleRange, 0, 90);
                EditorGUI.indentLevel--;
            }


            //t.followHeightMode = (ThirdPersonCameraController.FollowHeightMode) EnumPopup("Follow height mode", t.followHeightMode);
            //t.minOffset = Vector3Field("Min offset", t.minOffset); //not currently used
            //t.maxOffset = Vector3Field("Max offset", t.maxOffset); //not currently used

            /*
            BeginHorizontal();
            t.useMinDistance = Toggle("Clamp min distance from target", t.useMinDistance);
            if (t.useMinDistance)
            {
                t.minDistanceFromTarget = FloatField(t.minDistanceFromTarget);
            }
            EndHorizontal();
            */

            BeginHorizontal();
            t.useMaxDistance = Toggle("Clamp max distance from target", t.useMaxDistance);
            if (t.useMaxDistance)
            {
                EditorGUI.indentLevel++;
                t.maxDistanceFromTarget = FloatField(t.maxDistanceFromTarget);
                EditorGUI.indentLevel--;
            }
            EndHorizontal();
            
            EditorGUI.indentLevel--;
        }

    }

    private void DoTargetLook(CameraParams t)
    {
        showTargetLookParams = FoldoutHeader(showTargetLookParams, "Target look");
        if(showTargetLookParams)
        {
            EditorGUI.indentLevel++;
            t.lookOffset = FloatField("Look offset", t.lookOffset);
            t.interpolateTargetLookAt = Toggle("Interpolate", t.interpolateTargetLookAt);
            if(t.interpolateTargetLookAt)
            {
                EditorGUI.indentLevel++;
                t.targetLookAtLerpSpeed = FloatField("Look at speed", t.targetLookAtLerpSpeed);
                EditorGUI.indentLevel--;
            }
            EditorGUI.indentLevel--;
        }
    }

    private void DoOcclusionAvoidance(CameraParams t)
    {
        showOcclusionAvoidanceParams = FoldoutHeader(showOcclusionAvoidanceParams, "Occlusion avoidance");
        if(showOcclusionAvoidanceParams)
        {
            EditorGUI.indentLevel++;
            t.avoidFollowTargetOcclusion = Toggle("Avoid occlusion", t.avoidFollowTargetOcclusion);
            if(t.avoidFollowTargetOcclusion)
            {
                EditorGUI.indentLevel++;
                //t.usePreviousTargetPositionsForCameraPullIn = Toggle("Follow previous target positions for pull-in", t.usePreviousTargetPositionsForCameraPullIn);
                t.occlusionPullInSpeedHorizontal = FloatField("Pull-in speed horizontal", t.occlusionPullInSpeedHorizontal);
                t.occlusionPullInSpeedVertical = FloatField("Pull-in speed vertical", t.occlusionPullInSpeedVertical);
                t.occlusionFollowSpeedIncrease = FloatField("Increase follow speed", t.occlusionFollowSpeedIncrease);
                t.occlusionClipPanePadding = FloatField("Near clip pane padding", t.occlusionClipPanePadding);
                //t.preserveCameraHeight = Toggle("Preserve camera height", t.preserveCameraHeight);
                t.useTimeInOcclusionMultiplier = Toggle("Ease in/out multiplier", t.useTimeInOcclusionMultiplier);
                if(t.useTimeInOcclusionMultiplier)
                {
                    EditorGUI.indentLevel++;
                    t.maxTimeInOcclusionMultiplier = FloatField("Max", t.maxTimeInOcclusionMultiplier);
                    t.timeInOcclusionRampUpSpeed = FloatField("Ease in speed", t.timeInOcclusionRampUpSpeed);
                    t.timeInOcclusionRampDownSpeed = FloatField("Ease out speed", t.timeInOcclusionRampDownSpeed);
                    EditorGUI.indentLevel--;
                }
                EditorGUI.indentLevel--;
            }
            EditorGUI.indentLevel--;
        }
    }

    private void DoCamWhiskers(CameraParams t)
    {
        showCamWhiskerParams = FoldoutHeader(showCamWhiskerParams, "Camera whiskers");
        if(showCamWhiskerParams)
        {
            EditorGUI.indentLevel++;
            t.useCamWhiskers = Toggle("Use camera whiskers", t.useCamWhiskers);
            if(t.useCamWhiskers)
            {
                EditorGUI.indentLevel++;
                t.whiskerPushStrength = FloatField("Push strength", t.whiskerPushStrength);
                t.numWhiskers = IntField("Number of whiskers", t.numWhiskers);
                t.whiskerLength = FloatField("Whisker length", t.whiskerLength);
                t.whiskerSectorAngle = Slider("Whisker angle", t.whiskerSectorAngle, 0f, 360f);
                EditorGUI.indentLevel--;
            }
            EditorGUI.indentLevel--;
        }
    }

    private void DoCollisionAvoidance(CameraParams t)
    {
        showCollisionAvoidanceParams = FoldoutHeader(showCollisionAvoidanceParams, "Collision avoidance");
        if (showCollisionAvoidanceParams)
        {
            EditorGUI.indentLevel++;
            t.avoidCollisionWithGeometry = Toggle("Avoid collisions", t.avoidCollisionWithGeometry);
            EditorGUI.indentLevel--;
        }
    }

    private void DoTargetOrbit(CameraParams t)
    {
        showTargetOrbitParams = FoldoutHeader(showTargetOrbitParams, "Target orbit");
        if(showTargetOrbitParams)
        {
            EditorGUI.indentLevel++;
            t.orbit = Toggle("Orbit target", t.orbit);
            if(t.orbit)
            {
                EditorGUI.indentLevel++;
                t.orbitSpeed = FloatField("Speed", t.orbitSpeed);
                t.orbitSensitivity = FloatField("Sensitivity", t.orbitSensitivity);

                //min/max y angles 
                LabelField("Min/max y angles:");
                BeginHorizontal();
                t.minOrbitYAngle = FloatField(t.minOrbitYAngle);
                //MinMaxSlider(ref t.minOrbitYAngle, ref t.maxOrbitYAngle, -90, 90, GUILayout.MinWidth(64), GUILayout.MaxWidth(256));
                t.maxOrbitYAngle = FloatField(t.maxOrbitYAngle);
                EndHorizontal();
                if (t.minOrbitYAngle > t.maxOrbitYAngle) t.minOrbitYAngle = t.maxOrbitYAngle; //stop min angle going above max angle
                EditorGUI.indentLevel--;
            }
            EditorGUI.indentLevel--;
        }
    }

    private void DoLayerMasks(CameraParams t)
    {
        showLayerMaskParams = FoldoutHeader(showLayerMaskParams, "Layer masks");
        if(showLayerMaskParams)
        {
            EditorGUI.indentLevel++;
            //https://answers.unity.com/questions/42996/how-to-create-layermask-field-in-a-custom-editorwi.html
            //just doing t.whateverMask = MaskField("..", t.colliderLayerMask, InternalEditorUtility.layers) makes the actual mask value one up from whatever the inspector shows, for some reason
            LayerMask colliderLayers = MaskField("Camera colliders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.colliderLayerMask), InternalEditorUtility.layers);
            t.colliderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(colliderLayers);
            LayerMask occluderLayers = MaskField("Camera occluders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.occluderLayerMask), InternalEditorUtility.layers);
            t.occluderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(occluderLayers);
            EditorGUI.indentLevel--;
        }
    }

    private void DoCamUpdateFunction(CameraParams t)
    {
        showCamUpdateFunctionParams = FoldoutHeader(showCamUpdateFunctionParams, "Camera update function");
        if(showCamUpdateFunctionParams)
        {
            EditorGUI.indentLevel++;

            t.updateFunction = (CameraUpdateFunction)EnumPopup(new GUIContent("Camera update function",
                "The Unity function in which to calculate camera movement. The standard is LateUpdate, which occurs after every Update " +
                "in the current frame. If you experience juddery camera movement using LateUpdate, try using FixedUpdate - this often solves camera jitter problems " +
                "when following a target which is moved in FixedUpdate. Also, if your follow target uses a Rigidbody for movement, and you are experiencing " +
                "camera jitter, try enabling interpolation or extrapolation on its rigidbody component."), t.updateFunction);
            
            EditorGUI.indentLevel--;
        }
    }

    #region convenience inspector functions
    private bool FoldoutHeader(bool foldout, string label)
    {
        return Foldout(foldout, label, true, EditorStyles.foldoutHeader);
    }
    #endregion
}
