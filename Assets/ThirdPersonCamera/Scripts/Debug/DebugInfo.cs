using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThirdPersonCamera
{
    /// <summary>
    /// Stores debug info about the camera and its controlling scripts and displays it visually as gizmos etc.
    /// </summary>
    public class DebugInfo : MonoBehaviour
    {
        //target info
        [HideInInspector] public Vector3 followTargetPos;

        //camera virtual size
        [HideInInspector] public Vector3 camNearClipPanePos;
        [HideInInspector] public float camVirtualSize;

        //Collision avoidance info
        [HideInInspector] public Vector3 initialNearClipPanePos, updatedNearClipPanePos;
        [HideInInspector] public List<Vector3> hitPoints;
        [HideInInspector] public Vector3 minHitSphereCentre;
        [HideInInspector] public float minHitDistance;
        [HideInInspector] public Vector3 avoidDir;

        private void Awake()
        {
            hitPoints = new List<Vector3>();
        }

        private void OnDrawGizmos()
        {
            DrawCameraVirtualSize();
            DrawCollisionDebugGizmos();
        }

        private void DrawCollisionDebugGizmos()
        {
            //draw camera initial and updated clip pane positions for this tick
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(initialNearClipPanePos, camVirtualSize);

            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(updatedNearClipPanePos, camVirtualSize);

            //draw hit(s) info
            foreach(Vector3 hitPoint in hitPoints)
            {
                Debug.DrawLine(followTargetPos, hitPoint, Color.magenta);
                DrawCollisionHitPoint(hitPoint);
            }
        }

        private void DrawCollisionHitPoint(Vector3 hitPoint)
        {
            Debug.DrawRay(hitPoint, Vector3.up * 0.05f, Color.magenta);
            Debug.DrawRay(hitPoint, Vector3.right * 0.05f, Color.magenta);
            Debug.DrawRay(hitPoint, Vector3.down * 0.05f, Color.magenta);
            Debug.DrawRay(hitPoint, Vector3.left * 0.05f, Color.magenta);
            Debug.DrawRay(hitPoint, Vector3.forward * 0.05f, Color.magenta);
            Debug.DrawRay(hitPoint, Vector3.back * 0.05f, Color.magenta);
        }
        
        private void DrawCameraVirtualSize()
        {
            Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
            Gizmos.DrawWireSphere(camNearClipPanePos, camVirtualSize);
        }



    }
}