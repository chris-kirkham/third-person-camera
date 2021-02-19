using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Script for a camera follow target which follows the player with interpolation - 
/// this smooths out the potentially jerky movement that can happen when the camera follows the player directly
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(SphereCollider))]
public class FollowTargetLerp : MonoBehaviour
{
    private Rigidbody rb;

    [SerializeField] private GameObject followTarget;
    [SerializeField] [Min(0)] private float followSpeed = 1;
    [SerializeField] [Min(0)] private float maxDistance = 1;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        if(followTarget != null)
        {
            //lerp follow target position
            Vector3 objToFollowTarget = followTarget.transform.position - transform.position;
            rb.AddForce(objToFollowTarget * followSpeed);
        }
    }
}
