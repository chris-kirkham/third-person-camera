using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simple rolling ball controller for the camera demo
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class BallMovement : MonoBehaviour
{
    private Rigidbody rb;

    public Camera cam;
    public float moveSpeed = 1f;
    public float jumpForce = 1f;
    public LayerMask groundLayer;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        Vector3 camForwardFlat = new Vector3(cam.transform.forward.x, 0f, cam.transform.forward.z).normalized;
        Quaternion camLookFlat = Quaternion.LookRotation(camForwardFlat, Vector3.up);
        Vector3 moveDirection = camLookFlat * new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical"));
        rb.AddForce(moveDirection * moveSpeed, ForceMode.Force);

        bool jump = Input.GetKeyDown(KeyCode.Space);
        if (jump && Physics.Raycast(transform.position, Vector3.down, 2f, groundLayer))
        {
            rb.AddForce(Vector3.up, ForceMode.Impulse);
        }

        if(rb.velocity.magnitude > 0.01f) transform.forward = Vector3.Lerp(transform.forward, new Vector3(rb.velocity.normalized.x, 0f, rb.velocity.normalized.z), 0.5f);
    }
}
