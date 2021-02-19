using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TowerFollowTarget : MonoBehaviour
{
    [SerializeField] private GameObject heightFollowTarget;
    [SerializeField] private float lerpSpeed = 10f;
    [SerializeField] private float rotationLerpSpeed = 10f;

    void Update()
    {
        if (heightFollowTarget != null)
        {
            Vector3 newPos = new Vector3(transform.position.x, heightFollowTarget.transform.position.y, transform.position.z);
            transform.position = Vector3.Lerp(transform.position, newPos, Time.deltaTime * lerpSpeed);

            Vector3 newFacing = (transform.position - heightFollowTarget.transform.position).normalized;
            newFacing.y = 0f;
            transform.forward = Vector3.Lerp(transform.forward, newFacing, Time.deltaTime * rotationLerpSpeed);
            Debug.DrawRay(transform.position, transform.forward, Color.green);
        }
    }
}
