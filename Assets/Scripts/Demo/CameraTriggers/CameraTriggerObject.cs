using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// The script which camera behaviour triggers look for. Attach this to the object with which you want to trigger them (for example, the player the camera is following).
    /// </summary>
    public class CameraTriggerObject : MonoBehaviour
    {
        [Tooltip("The behaviour components of the camera controller you wish to be affected by the trigger.")]
        [SerializeField] private SharedCameraComponents associatedCamComponents; 

        public SharedCameraComponents GetCameraComponents()
        {
            return associatedCamComponents;
        }
    }
}