using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
    using UnityEditor;
#endif 

namespace ThirdPersonCamera
{
    /// <summary>
    /// Stores debug info about the camera and its controlling scripts and displays it visually as gizmos etc.
    /// </summary>
    public class DebugDrawing : MonoBehaviour
    {
        //target info
        [HideInInspector] public Vector3 followTargetPos;

        //camera virtual size
        [SerializeField] private bool drawCameraVirtualSize = true;
        [HideInInspector] public Vector3 camNearClipPanePos;
        [HideInInspector] public float camVirtualSize;

        //Collision avoidance info
        [SerializeField] private bool drawCollisionInfo = true;
        [HideInInspector] public Vector3 initialNearClipPanePos, updatedNearClipPanePos;
        [HideInInspector] public List<Vector3> hitPoints = new List<Vector3>();
        [HideInInspector] public Vector3 minHitSphereCentre;
        [HideInInspector] public float minHitDistance;
        [HideInInspector] public Vector3 avoidDir;

        //Collision history tracking
        [SerializeField] private bool drawCollisionHistory = true;
        [HideInInspector] public List<Vector3> camCollisionPositions = new List<Vector3>(); //stores positions at which the camera collided with geometry
        private int maxCollisionPositionsToTrack = 100;

        private void OnDrawGizmos()
        {
            if(drawCameraVirtualSize) DrawCameraVirtualSize();
            if(drawCollisionInfo) DrawCollisionDebugGizmos();
            if(drawCollisionHistory) DrawCollisionLines();
        }

        private void DrawCollisionDebugGizmos()
        {
            //draw camera initial and updated clip pane positions for this tick
            Gizmos.color = new Color(1, 1, 1, 0.5f);
            Gizmos.DrawWireSphere(initialNearClipPanePos, camVirtualSize);

            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(updatedNearClipPanePos, camVirtualSize);

            //draw hit(s) info
            if(hitPoints != null)
            {
                foreach (Vector3 hitPoint in hitPoints)
                {
                    Debug.DrawLine(followTargetPos, hitPoint, Color.magenta);
                    DrawCollisionHitPoint(hitPoint);
                }
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

        private void DrawCollisionLines()
        {
            if(camCollisionPositions != null && camCollisionPositions.Count > 0)
            {
                for (int i = 1; i < camCollisionPositions.Count; i++)
                {
                    Gizmos.color = new Color(1, 0, 1, 1) * Vector3.Distance(camCollisionPositions[i - 1], camCollisionPositions[i]);
                    Gizmos.DrawLine(camCollisionPositions[i - 1], camCollisionPositions[i]);
#if UNITY_EDITOR
                    Handles.Label(camCollisionPositions[i], i.ToString());
#endif
                    Gizmos.DrawWireSphere(camCollisionPositions[i], camVirtualSize);
                }
            }
        }

        public void AddCamCollisionPosition(Vector3 pos)
        {
            camCollisionPositions.Add(pos);
            if (camCollisionPositions.Count > maxCollisionPositionsToTrack)
            {
                camCollisionPositions.RemoveAt(0);
            }
        }

    }
}