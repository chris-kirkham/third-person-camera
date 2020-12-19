using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Draws lines across the camera's view to help with scene composition
/// </summary>
[RequireComponent(typeof(Camera))]
public class CompositionHelper : MonoBehaviour
{
    private Camera cam;

    public bool showCentreLines;
    public bool showThirds;

    private void Start()
    {
        cam = GetComponent<Camera>();
    }

    private void OnDrawGizmos()
    {
        //Get clip pane info
        float z = cam.nearClipPlane;
        float y = Mathf.Tan(cam.fieldOfView / 2 * Mathf.Deg2Rad) * z;
        float x = y * cam.aspect;

    }
}
