using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TowerFollowTarget_FollowPlayerHeight : MonoBehaviour
{
    [SerializeField] private GameObject heightFollowTarget;

    void Update()
    {
        if (heightFollowTarget != null) transform.position = new Vector3(transform.position.x, heightFollowTarget.transform.position.y, transform.position.z);   
    }
}
