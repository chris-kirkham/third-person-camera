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
    [SerializeField] private static bool showCameraModeParams = true;
    [SerializeField] private static bool showTargetFollowParams = true;
    [SerializeField] private static bool showTargetLookParams = true;
    [SerializeField] private static bool showOcclusionAvoidanceParams = true;
    [SerializeField] private static bool showCamWhiskerParams = true;
    [SerializeField] private static bool showCollisionAvoidanceParams = true;
    [SerializeField] private static bool showTargetOrbitParams = true;
    [SerializeField] private static bool showCamUpdateFunctionParams = true;

    //"tabs" via GUILayout.Toolbar
    private static int currentTabID = 0;
    private static readonly string[] tabNames = new string[7] { "Camera mode", "Target follow", "Target orbit", "Target look", "Occlusion avoidance", "Collision avoidance", "Update function" }; 

    public override void OnInspectorGUI()
    {
        cameraParams = (CameraParams) target;

        DoTabs();

        switch(currentTabID)
        {
            case 0:
                DoCameraMode(cameraParams);
                break;
            case 1:
                DoTargetFollowing(cameraParams);
                break;
            case 2:
                DoTargetOrbit(cameraParams);
                break;
            case 3:
                DoTargetLook(cameraParams);
                break;
            case 4:
                DoOcclusionAvoidance(cameraParams);
                break;
            case 5:
                DoCollisionAvoidance(cameraParams);
                break;
            case 6:
                DoCamUpdateFunction(cameraParams);
                break;
            default:
                break;
        }
        
        /*
        DoCameraMode(cameraParams);
        DoTargetFollowing(cameraParams);
        DoTargetOrbit(cameraParams);
        DoTargetLook(cameraParams);
        DoOcclusionAvoidance(cameraParams);
        DoCamWhiskers(cameraParams);
        DoCollisionAvoidance(cameraParams);
        DoCamUpdateFunction(cameraParams);
        */

        EditorUtility.SetDirty(target);
    }

    private void DoTabs()
    {
        currentTabID = GUILayout.SelectionGrid(currentTabID, tabNames, 2);
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
                //t.orbitToFollowTransitionSpeed = FloatField("Transition speed", t.orbitToFollowTransitionSpeed);
                
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
            /* Rear follow */

            LabelField("Rear follow", EditorStyles.boldLabel);

            EditorGUI.indentLevel++;

            t.interpolateTargetFollow = Toggle("Interpolate", t.interpolateTargetFollow);
            
            t.desiredOffset = Vector3Field("Desired offset", t.desiredOffset);
            if (t.interpolateTargetFollow)
            {
                LabelField("Follow speeds", EditorStyles.boldLabel);
                
                EditorGUI.indentLevel++;

                t.followSpeedOrientation = FloatField(new GUIContent("Orientation", "How quickly the camera will orient itself horizontally to the target's facing."), t.followSpeedOrientation);
                t.followSpeedHeight = FloatField(new GUIContent("Height", "How quickly the camera will move towards the target's height + offset."), t.followSpeedHeight);
                t.followSpeedDistance = FloatField(new GUIContent("Distance", "How quickly the camera will move towards the desired distance from the target."), t.followSpeedDistance);

                EditorGUI.indentLevel--;
            }

            Space();

            /*
            t.useWorldSpaceOffset = Toggle(new GUIContent("Use world space offset?", "By default, the offset is relative to the follow target's local space." +
                " Set this to true to use world space offset."), t.useWorldSpaceOffset);
            */

            /* Front follow */
            LabelField("Front follow", EditorStyles.boldLabel);
            t.allowMoveTowardsCamera = Toggle(new GUIContent("Target can move towards camera?", "Use the desired front offset when the follow target is facing the camera. Typically," +
                                "this is used to let the camera stay in front of the target when the target is moving towards it," +
                                " rather than trying to reorient behind the target."), t.allowMoveTowardsCamera);
            if (t.allowMoveTowardsCamera)
            {

                t.desiredFrontOffset = Vector3Field("Desired front offset", t.desiredFrontOffset);
                LabelField("Front follow speeds", EditorStyles.boldLabel);

                EditorGUI.indentLevel++;

                if(t.interpolateTargetFollow) t.frontFollowSpeedOrientation = FloatField("Orientation", t.frontFollowSpeedOrientation);
                if(t.interpolateTargetFollow) t.frontFollowSpeedHeight = FloatField("Height", t.frontFollowSpeedHeight);
                if(t.interpolateTargetFollow) t.frontFollowSpeedDistance = FloatField("Distance", t.frontFollowSpeedDistance);
                t.movingTowardsCameraAngleRange = Slider("Angle range", t.movingTowardsCameraAngleRange, 0, 90);
                
                EditorGUI.indentLevel--;
            }

            LabelField("Follow height", EditorStyles.boldLabel);

            t.followHeightMode = (FollowHeightMode) EnumPopup("Follow height mode", t.followHeightMode);
            if(t.followHeightMode == FollowHeightMode.AboveGround)
            {
                EditorGUI.indentLevel++;

                t.desiredHeightAboveGround = FloatField("Height above ground", t.desiredHeightAboveGround);
                t.aboveGroundFallbackHeight = FloatField(new GUIContent("Fallback height above target",
                    "If no valid ground is found below the camera, the camera will move to this height above the target."),
                    t.aboveGroundFallbackHeight);
                t.maxGroundDistance = FloatField(new GUIContent("Max ground check distance",
                    "The maximum distance to check for ground below the target - greater than this distance, the camera will revert to following relative to the target height."),
                    t.maxGroundDistance);
                if (t.desiredHeightAboveGround > t.maxGroundDistance) t.maxGroundDistance = t.desiredHeightAboveGround; //clamp max distance to desired height if necessary
                t.maxGroundSlopeAngle = Slider(new GUIContent("Max ground slope angle",
                    "The maximum slope angle at which geometry is counted as \"ground\"."),
                    t.maxGroundSlopeAngle, 0f, 90f);
                LayerMask groundLayers = MaskField("Ground layer mask", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.groundLayerMask), InternalEditorUtility.layers);
                t.groundLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(groundLayers);

                EditorGUI.indentLevel--;
            }


            LabelField("Clamp min/max distance", EditorStyles.boldLabel);

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

                LayerMask occluderLayers = MaskField("Camera occluders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.occluderLayerMask), InternalEditorUtility.layers);
                t.occluderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(occluderLayers);  

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

            //https://answers.unity.com/questions/42996/how-to-create-layermask-field-in-a-custom-editorwi.html
            //just doing t.whateverMask = MaskField("..", t.colliderLayerMask, InternalEditorUtility.layers) makes the actual mask value one up from whatever the inspector shows, for some reason
            LayerMask colliderLayers = MaskField("Camera colliders", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(t.colliderLayerMask), InternalEditorUtility.layers);
            t.colliderLayerMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(colliderLayers);

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

                t.desiredOrbitDistance = FloatField("Desired orbit distance", t.desiredOrbitDistance);
                t.orbitSpeed = FloatField("Speed", t.orbitSpeed);
                t.orbitSpeedDistance = FloatField("Speed (distance)", t.orbitSpeedDistance);
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
