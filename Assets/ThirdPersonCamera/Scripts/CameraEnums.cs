using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>State enum which represents the camera's current behaviour.</summary>
    public enum CameraState { FollowingTarget, OrbitingTarget, TargetMovingTowardsCamera, OrbitToFollow_HoldingOrbitAngle, OrbitToFollow_Transitioning };

    ///<summary>state enum which represents the target's facing relative to the camera.</summary>
    public enum CameraTargetOrientationState { TowardsCamera, AwayFromCamera };

    /// <summary>Represents the camera's behaviour modes</summary>
    public enum CameraBehaviourMode { Follow, Orbit, FollowAndOrbit };

}