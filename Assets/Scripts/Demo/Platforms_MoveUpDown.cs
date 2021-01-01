using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Platforms_MoveUpDown : MonoBehaviour
{
    public Vector3 moveDirection = Vector3.forward;
    [Min(0)] public float moveDistance = 1f;
    [Min(0.01f)] public float moveTime = 1f;

    private Vector3 origin;

    void Start()
    {
        origin = transform.position;
    }

    void Update()
    {
        float t = (Mathf.Sin(Time.time / moveTime) + 1) * 0.5f;
        transform.position = Vector3.Lerp(origin, origin + (moveDirection.normalized * moveDistance), t);
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
#if UNITY_EDITOR
        if (UnityEditor.EditorApplication.isPlaying)
        {
            Gizmos.DrawRay(origin, moveDirection.normalized * moveDistance);
        }
        else
        {
            Gizmos.DrawRay(transform.position, moveDirection.normalized * moveDistance);
        }
#else
        Gizmos.DrawRay(transform.position, moveDirection.normalized * moveDistance);
#endif

    }
}
