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

        //flag set if orbit angle has changed since last tick
        public bool OrbitAngleChanged { get; private set; } = false;

        /* Input settings */
        //invert y
        [SerializeField] private bool invertY = false;

        //orbit input smoothing
        [SerializeField] private bool useOrbitInputSmoothing = false;
        [SerializeField] private float smoothingSharpness = 100;

        void Start()
        {
            camParams = GetComponent<SharedCameraComponents>().cameraParams;
        }

        void Update()
        {
            UpdateOrbitInput(Time.deltaTime);
        }

        private void UpdateOrbitInput(float deltaTime)
        {
            Vector2 rawOrbitInput = new Vector2(Input.GetAxis("Mouse Y"), Input.GetAxis("Mouse X")); //swap x and y so x axis rotates horizontally and y rotates vertically

            //if no (or virtually no) input, smooth orbit input if using smoothing, or reset it
            if (rawOrbitInput.sqrMagnitude < Mathf.Epsilon)
            {
                orbitInput = useOrbitInputSmoothing ? orbitInput * Mathf.Clamp01(1 - (deltaTime * smoothingSharpness)) : Vector2.zero;
            }
            
            if (invertY) rawOrbitInput.x *= -1;
            orbitInput += rawOrbitInput * camParams.orbitSensitivity * deltaTime;

            OrbitAngleChanged = (orbitInput.sqrMagnitude > Mathf.Epsilon);
        }

        public Vector2 GetOrbitInput()
        {
            return orbitInput;
        }
    }
}