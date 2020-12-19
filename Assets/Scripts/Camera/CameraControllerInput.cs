using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Handles user input for the third-person camera controller.
    /// </summary>
    public class CameraControllerInput : MonoBehaviour
    {
        /* orbit */
        [SerializeField] private float orbitSensitivity = 10f;

        //current orbit x and y angles
        private Vector2 orbitAngles = Vector2.zero;

        //flag set if orbit angle has changed since last tick
        public bool OrbitAngleChanged { get; private set; } = false;

        void Update()
        {
            UpdateOrbitAngles();
        }

        private void UpdateOrbitAngles()
        {
            float inputX = Input.GetAxis("Mouse X") * orbitSensitivity;
            float inputY = Input.GetAxis("Mouse Y") * orbitSensitivity;
            
            OrbitAngleChanged = (inputX > 0 || inputY > 0);

            orbitAngles.x += inputX;
            orbitAngles.y -= inputY;
            orbitAngles.x = ClampOrbitAngle(orbitAngles.x);
            orbitAngles.y = ClampOrbitAngle(orbitAngles.y);
        }

        private float ClampOrbitAngle(float angle)
        {
            if (angle < -360) return angle + 360;
            if (angle >= 360) return angle - 360;
            return angle;
        }

        public Vector2 GetOrbitAngles()
        {
            return orbitAngles;
        }
    }
}