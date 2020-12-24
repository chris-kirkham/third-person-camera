using System.Collections;
using System.Collections.Generic;
using uei = UnityEngine.Internal;
using UnityEngine;

public static class Lerps 
{
    /* FLOAT */
    //Smoothstep and Smootherstep from https://chicounity3d.wordpress.com/2014/05/23/how-to-lerp-like-a-pro/
    public static float Smoothstep(float a, float b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * (3f - 2f * t);
        return a + ((b - a) * t);
    }
    
    public static float Smootherstep(float a, float b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * t * (t * (6f * t - 15f) + 10f);
        return a + ((b - a) * t);
    }

    /* DOUBLE */
    public static double Smoothstep(double a, double b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * (3f - 2f * t);
        return a + ((b - a) * t);
    }

    public static double Smootherstep(double a, double b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * t * (t * (6f * t - 15f) + 10f);
        return a + ((b - a) * t);
    }

    /* VECTOR3 */
    public static Vector3 Smoothstep(Vector3 a, Vector3 b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * (3f - 2f * t);
        return a + ((b - a) * t);
    }

    public static Vector3 Smootherstep(Vector3 a, Vector3 b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * t * (t * (6f * t - 15f) + 10f);
        return a + ((b - a) * t);
    }



    //Smoothdamp with different smooth times for each axis 
    public static Vector3 SmoothDampSeparateAxis
    (
        Vector3 current, 
        Vector3 target, 
        ref Vector3 currentVelocity, 
        Vector3 smoothTime,
        [uei.DefaultValue("Time.deltaTime")] float deltaTime
    )
    {
        float x = Mathf.SmoothDamp(current.x, target.x, ref currentVelocity.x, smoothTime.x);
        float y = Mathf.SmoothDamp(current.y, target.y, ref currentVelocity.y, smoothTime.y);
        float z = Mathf.SmoothDamp(current.z, target.z, ref currentVelocity.z, smoothTime.z);
        return new Vector3(x, y, z);
    }
    
    //Smoothdamp with different smooth times for each axis
    public static Vector3 SmoothDampSeparateAxis
    (
        Vector3 current,
        Vector3 target,
        ref Vector3 currentVelocity,
        Vector3 smoothTime,
        Vector3 maxSpeed,
        [uei.DefaultValue("Time.deltaTime")] float deltaTime
    )
    {
        float x = Mathf.SmoothDamp(current.x, target.x, ref currentVelocity.x, smoothTime.x, maxSpeed.x, deltaTime);
        float y = Mathf.SmoothDamp(current.y, target.y, ref currentVelocity.y, smoothTime.y, maxSpeed.y, deltaTime);
        float z = Mathf.SmoothDamp(current.z, target.z, ref currentVelocity.z, smoothTime.z, maxSpeed.z, deltaTime);
        return new Vector3(x, y, z);
    }


    //Smoothdamp with separate smooth times for each axis and for positive/negative Y values
    public static Vector3 SmoothDampSeparateAxisSeparateY
    (
        Vector3 current,
        Vector3 target,
        ref Vector3 currentVelocity,
        float smoothTimeX,
        float smoothTimePositiveY,
        float smoothTimeNegativeY,
        float smoothTimeZ,
        [uei.DefaultValue("Time.deltaTime")] float deltaTime
    )
    {
        float x = Mathf.SmoothDamp(current.x, target.x, ref currentVelocity.x, smoothTimeX, Mathf.Infinity, deltaTime);
        float smoothTimeY = target.y - current.y > 0 ? smoothTimePositiveY : smoothTimeNegativeY;
        float y = Mathf.SmoothDamp(current.y, target.y, ref currentVelocity.y, smoothTimeY, Mathf.Infinity, deltaTime);
        float z = Mathf.SmoothDamp(current.z, target.z, ref currentVelocity.z, smoothTimeZ, Mathf.Infinity, deltaTime);
        return new Vector3(x, y, z);
    }



    /* QUATERNION */
    public static Quaternion Smoothstep(Quaternion a, Quaternion b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * (3f - 2f * t);
        return Quaternion.Lerp(a, b, t);
    }

    public static Quaternion Smootherstep(Quaternion a, Quaternion b, float t)
    {
        t = Mathf.Clamp01(t);
        t = t * t * t * (t * (6f * t - 15f) + 10f);
        return Quaternion.Lerp(a, b, t);
    }
}
