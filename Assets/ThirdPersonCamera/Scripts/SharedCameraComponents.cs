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

        //a clone of cameraParams is returned by its getter; this is to prevent changes being made to the original cameraParams ScriptableObject which would persist after the game has ended
        [SerializeField] private CameraParams cameraParams; 
        private CameraParams cameraParamsClone; 

        private void Awake()
        {
            cameraParamsClone = Instantiate(cameraParams); //cache initial camera params
        }

        //returns the clone of cameraParams; this prevents changes being made to the original cameraParams ScriptableObject which would persist after the game has ended
        public CameraParams GetCameraParams()
        {
            return cameraParamsClone;
        }

        public void SetCameraParams(CameraParams cameraParams)
        {
            this.cameraParams = cameraParams;
            cameraParamsClone = Instantiate(cameraParams);
        }
    }
}