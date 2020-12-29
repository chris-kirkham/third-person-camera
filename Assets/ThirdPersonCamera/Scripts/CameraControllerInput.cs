using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Handles user input for the third-person camera controller.
    /// </summary>
    [RequireComponent(typeof(SharedCameraComponents))]
    public class CameraControllerInput : MonoBehaviour
    {
        private CameraParams camParams;

        //current orbit x and y angles
        private Vector2 orbitInput = Vector2.zero; //orbit input multiplied by orbit sensitivity
        private Vector2 rawOrbitInput = Vector2.zero;

        //flag set if orbit angle has changed since last tick
        public bool OrbitAngleChanged { get; private set; } = false;

        /* Input settings */
        //invert y
        [SerializeField] private bool invertY = false;

        //orbit input smoothing
        [SerializeField] private bool useOrbitInputSmoothing = false;
        [SerializeField] private float smoothingSharpness = 100;

        void Awake()
        {
            camParams = GetComponent<SharedCameraComponents>().cameraParams;
        }

        void Update()
        {
            UpdateOrbitInput(Time.deltaTime);
        }

        private void UpdateOrbitInput(float deltaTime)
        {
            //smooth orbit input if using smoothing, or reset it
            if(useOrbitInputSmoothing)
            { 
                //move input towards (0, 0)
                orbitInput *= Mathf.Clamp01(1 - (deltaTime * smoothingSharpness));
            }
            else
            {
                orbitInput = Vector2.zero;
            }

            rawOrbitInput = new Vector2(Input.GetAxis("Mouse Y"), Input.GetAxis("Mouse X")); //swap x and y so x axis rotates horizontally and y rotates vertically
            if (invertY) rawOrbitInput.x *= -1;
            orbitInput += rawOrbitInput * camParams.orbitSensitivity * deltaTime;

            OrbitAngleChanged = (orbitInput.x != 0 || orbitInput.y != 0);
        }

        public Vector2 GetOrbitInput()
        {
            return orbitInput;
        }
    }
}