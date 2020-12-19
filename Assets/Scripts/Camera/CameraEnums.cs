using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    public enum CameraState { FollowingTarget, OrbitingTarget, TargetMovingTowardsCamera, TransitioningFromOrbitToFollow };

    public enum CameraBehaviourMode { Follow, Orbit, FollowAndOrbit };

}