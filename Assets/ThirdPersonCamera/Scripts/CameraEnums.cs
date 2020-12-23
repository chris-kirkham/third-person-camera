using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    public enum CameraState { FollowingTarget, OrbitingTarget, TargetMovingTowardsCamera, OrbitToFollow_HoldingOrbitAngle, OrbitToFollow_Transitioning };

    public enum CameraBehaviourMode { Follow, Orbit, FollowAndOrbit };

}