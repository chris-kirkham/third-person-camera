using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Script to hold components used by more than one ThirdPersonCamera script.
    /// </summary>
    public class SharedCameraComponents : MonoBehaviour
    {
        public GameObject followTarget;
        public GameObject lookAtTarget;
        public CameraParams cameraParams; 
    }
}