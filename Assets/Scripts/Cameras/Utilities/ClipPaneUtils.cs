using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Extension methods for doing stuff relating to the camera clip pane (raycasts etc.)
/// </summary>
public static class ClipPaneUtils
{
    //Fires rays in a direction from the corners of the camera's near clip pane, returning a bool array 
    //representing raycast hits (clockwise from top left)
    /*
    public static bool[] RaycastsFromNearClipPane(this Camera cam, Vector3 dir, out RaycastHit[] hits, float maxDist, LayerMask layerMask)
    {
        Vector3[] points = GetNearClipPaneCornersWorld(cam);

        bool[] hitBools = new bool[4] { false, false, false, false };
        hits = new RaycastHit[4];
        if (Physics.Raycast(points[0], dir, out hits[0], maxDist, layerMask)) hitBools[0] = true;
        if (Physics.Raycast(points[1], dir, out hits[1], maxDist, layerMask)) hitBools[1] = true;
        if (Physics.Raycast(points[2], dir, out hits[2], maxDist, layerMask)) hitBools[2] = true;
        if (Physics.Raycast(points[3], dir, out hits[3], maxDist, layerMask)) hitBools[3] = true;

        return hitBools;
    }
    */

    //Fires rays in a direction from the corners of the camera's near clip pane;   
    //returns true if any hit and false if none hit. The out hits variable only returns successful hits, so it will be empty if none hit.
    public static bool RaycastsFromNearClipPane(this Camera cam, Vector3 dir, out List<RaycastHit> hits, float maxDist, LayerMask layerMask, float clipPanePadding = 0f)
    {
        Vector3[] points = GetNearClipPaneCornersWorld(cam, clipPanePadding);

        List<RaycastHit> hitsList = new List<RaycastHit>();
        RaycastHit hit;
        if (Physics.Raycast(points[0], dir, out hit, maxDist, layerMask)) hitsList.Add(hit);
        if (Physics.Raycast(points[1], dir, out hit, maxDist, layerMask)) hitsList.Add(hit);
        if (Physics.Raycast(points[2], dir, out hit, maxDist, layerMask)) hitsList.Add(hit);
        if (Physics.Raycast(points[3], dir, out hit, maxDist, layerMask)) hitsList.Add(hit);

        hits = hitsList;
        return hitsList.Count > 0;
    }

    //Fires rays from each corner of the camera's near clip pane to a given point in world space;
    //returns true if any hit and false if none hit. The out hits variable only returns successful hits, so it will be empty if none hit.
    public static bool LinecastsFromNearClipPane(this Camera cam, Vector3 target, out List<RaycastHit> hits, LayerMask layerMask, float clipPanePadding = 0f)
    {
        Vector3[] points = cam.GetNearClipPaneCornersWorld(clipPanePadding);

        List<RaycastHit> hitsList = new List<RaycastHit>();
        RaycastHit hit;
        if (Physics.Linecast(points[0], target, out hit, layerMask)) hitsList.Add(hit);
        if (Physics.Linecast(points[1], target, out hit, layerMask)) hitsList.Add(hit);
        if (Physics.Linecast(points[2], target, out hit, layerMask)) hitsList.Add(hit);
        if (Physics.Linecast(points[3], target, out hit, layerMask)) hitsList.Add(hit);

        hits = hitsList;
        return hitsList.Count > 0;
    }

    //Gets near clip pane corner points in world space from a given camera.
    //Returns points in clockwise order from top left
    public static Vector3[] GetNearClipPaneCornersWorld(this Camera cam, float clipPanePadding = 0f)
    {
        Vector3[] points = new Vector3[4];

        //clip pane y/x axes multiplied by this to pad clip pane size
        float paddingMult = clipPanePadding + 1;

        //from comment on https://www.youtube.com/watch?v=MkbovxhwM4I
        float z = cam.nearClipPlane;
        float y = Mathf.Tan(cam.fieldOfView / 2 * Mathf.Deg2Rad) * z * paddingMult;
        float x = y * cam.aspect * paddingMult;
        points[0] = cam.transform.TransformPoint(new Vector3(-x, y, z)); //top left
        points[1] = cam.transform.TransformPoint(new Vector3(x, y, z)); //top right
        points[2] = cam.transform.TransformPoint(new Vector3(x, -y, z)); //bottom right
        points[3] = cam.transform.TransformPoint(new Vector3(-x, -y, z)); //bottom left

        return points;
    }

    //Gets near clip pane corner points in local space from a given camera.
    //Returns points in clockwise order from top left
    public static Vector3[] GetNearClipPaneCornersLocal(this Camera cam, float clipPanePadding = 0f)
    {
        Vector3[] points = new Vector3[4];

        //clip pane y/x axes multiplied by this to pad clip pane size
        float paddingMult = clipPanePadding + 1;

        //from comment on https://www.youtube.com/watch?v=MkbovxhwM4I
        float z = cam.nearClipPlane;
        float y = Mathf.Tan(cam.fieldOfView / 2 * Mathf.Deg2Rad) * z * paddingMult;
        float x = y * cam.aspect * clipPanePadding * paddingMult;
        points[0] = new Vector3(-x, y, z); //top left
        points[1] = new Vector3(x, y, z); //top right
        points[2] = new Vector3(x, -y, z); //bottom right
        points[3] = new Vector3(-x, -y, z); //bottom left

        return points;
    }

    public static Vector3 GetNearClipPaneHalfExtents(this Camera cam, float clipPanePadding = 0f)
    {
        //clip pane y/x axes multiplied by this to pad clip pane size
        float paddingMult = clipPanePadding + 1;

        float z = cam.nearClipPlane;
        float y = Mathf.Tan(cam.fieldOfView / 2 * Mathf.Deg2Rad) * z * paddingMult;
        float x = y * cam.aspect * paddingMult;

        return new Vector3(x, y, z);
    }

    //returns the normalised screen point of a given world position for this camera ((0, 0) is top left, (1, 1) is bottom right of camera view)
    public static Vector2 WorldToScreenPointNormalised(this Camera cam, Vector3 worldPos)
    {
        Vector3 screenPos = cam.WorldToScreenPoint(worldPos);
        return new Vector2(screenPos.x / cam.pixelWidth, screenPos.y / cam.pixelHeight);
    }

    //Returns the x and y offset of an object from the centre of the camera view.
    //Ranges from (-0.5, -0.5) for top left to (0.5, 0.5) for bottom right (centre is (0, 0)).
    public static Vector2 GetOffsetFromCentreOfView(this Camera cam, Vector3 worldPos)
    {
        Vector3 screenPos = cam.WorldToScreenPoint(worldPos);
        //Debug.Log(objWorldPos + " screen offset = " + new Vector2((screenPos.x / cam.pixelWidth) - 0.5f, (screenPos.y / cam.pixelHeight) - 0.5f));
        //normalise x and y screen points to [0, 1] by dividing width and height; subtract 0.5 from each so (0, 0) becomes the centre
        return new Vector2((screenPos.x / cam.pixelWidth) - 0.5f, (screenPos.y / cam.pixelHeight) - 0.5f);
    }

    //Returns true if an object is within given x and y offsets from the centre of the camera view.
    //Deadzone values range from 0 (centre of screen i.e. no deadzone) to 0.5 (edge of screen)
    public static bool IsWithinDeadzone(this Camera cam, Vector3 worldPos, float xDeadzone, float yDeadzone)
    {
        if (xDeadzone == 0.5f && yDeadzone == 0.5f) return true;
        if (xDeadzone == 0 && yDeadzone == 0) return false;

        Vector2 offset = cam.GetOffsetFromCentreOfView(worldPos);
        return Mathf.Abs(offset.x) < xDeadzone && Mathf.Abs(offset.y) < yDeadzone;
    }

    //Returns true if an object is within given x and y offsets from the centre of the camera view.
    //Deadzone values range from 0 (centre of screen i.e. no deadzone) to 0.5 (edge of screen in that direction)
    public static bool IsWithinDeadzone(this Camera cam, Vector3 worldPos, float xDeadzoneLeft, float xDeadzoneRight, float yDeadzoneTop, float yDeadzoneBottom)
    {
        Vector2 offset = cam.GetOffsetFromCentreOfView(worldPos);
        return offset.x > -xDeadzoneLeft && offset.x < xDeadzoneRight && offset.y > -yDeadzoneTop && offset.y < yDeadzoneBottom;
    }

    //Returns the distance from the centre of the camera's near clip pane to its corner
    public static float GetNearClipPaneCentreToCornerDistance(this Camera cam, float clipPanePadding = 0)
    {
        Vector3 halfExtents = cam.GetNearClipPaneHalfExtents(clipPanePadding);
        return Vector3.Distance(new Vector3(0f, 0f, halfExtents.z), halfExtents);
    }

}
