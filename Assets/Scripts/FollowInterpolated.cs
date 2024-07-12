using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowInterpolated : MonoBehaviour
{
    public Transform target;
    public bool followPosition;
    public bool followRotation;
    public float lerpSpeed;
    public bool useUnscaledDeltaTime;

    private void LateUpdate()
    {
        FollowTarget();
    }

    private void FollowTarget()
    {
        if(!target)
        {
            return;
        }

        if (followPosition)
        {
            transform.position = Vector3.Lerp(
            transform.position,
            target.position,
            (useUnscaledDeltaTime ? Time.unscaledDeltaTime : Time.deltaTime) * lerpSpeed);
        }

        if(followRotation)
        {
            transform.rotation = target.rotation;
        }

    }
}
