using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Class to get and display debug info about the third-person camera. Should be attached to a canvas with four children 
    /// each with "TextMeshPro - Text (UI)" components
    /// </summary>
    public class DebugHUD : MonoBehaviour
    {
        //camera/components to track
        public Camera cam;
        public ThirdPersonCameraController camController;
        public CameraStateController stateController;
        public CameraControllerInput camInput;

        //debug text objects
        private TextMeshProUGUI headerText;
        private TextMeshProUGUI positionText;
        private TextMeshProUGUI stateText;
        private TextMeshProUGUI inputText;

        private void Awake()
        {
            //Get child TextMesh components
            headerText = transform.GetChild(0).GetComponent<TextMeshProUGUI>();
            positionText = transform.GetChild(1).GetComponent<TextMeshProUGUI>(); 
            stateText = transform.GetChild(2).GetComponent<TextMeshProUGUI>(); 
            inputText = transform.GetChild(3).GetComponent<TextMeshProUGUI>(); 
        }

        // Update is called once per frame
        void Update()
        {
            headerText.text = "Camera: " + cam.name;
            positionText.text = "Position: " + cam.transform.position.ToString();
            stateText.text = "State: " + stateController.GetCameraState().ToString();
            inputText.text = "Orbit angles: " + camInput.GetOrbitAngles().ToString();
        }
    }
}