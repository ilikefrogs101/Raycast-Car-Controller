using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    //Transforms
    public Transform[] wheels;
    public Transform centerOfMass;
    public Transform centerOfForce;
    public Transform orientation;
    public Transform raycastPoint;

    //Private
    Rigidbody rb;

    //Masks
    public LayerMask drivable;

    //Values
    public float maxSuspensionForce;
    public float maxSuspensionDistance;
    public float wheelRadius;
    public float dampingFactor;
    public float speed;
    public float turnSpeed;
    public float driftTraction;
    public float forwardTraction;
    public float groundDistance;
    public bool stabilise;
    bool grounded;  
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = transform.InverseTransformPoint(centerOfMass.position);
    }
    void FixedUpdate()
    {
        orientation.rotation = Quaternion.Euler(0, transform.eulerAngles.y, 0);
        grounded = Physics.Raycast(raycastPoint.position, Vector3.down, groundDistance, drivable, QueryTriggerInteraction.Ignore);
        foreach(Transform i in wheels)
        {
            RaycastHit wheelHit;
        
            if(Physics.Raycast(i.position, -i.up, out wheelHit, maxSuspensionDistance, drivable, QueryTriggerInteraction.Ignore))
            {
                float damping = dampingFactor * Vector3.Dot(rb.GetPointVelocity(i.position), i.up);
                rb.AddForceAtPosition(maxSuspensionForce * Time.deltaTime * transform.up * Mathf.Max((maxSuspensionDistance - wheelHit.distance + wheelRadius) / maxSuspensionDistance - damping, 0), i.position);
                
            }
        }  
        if(stabilise)
        {
            Stabilise();
        }  
        if(Input.GetKey(KeyCode.W) && grounded)
        {
            rb.AddForceAtPosition(orientation.forward * speed, centerOfForce.position);
        }
        if(Input.GetKey(KeyCode.S) && grounded)
        {
            rb.AddForceAtPosition(orientation.forward * -speed, centerOfForce.position);
        }
        if(Input.GetKey(KeyCode.A) && grounded)
        {
            rb.AddTorque(transform.up * Vector3.Dot(rb.velocity, transform.forward) * -turnSpeed);
        }        
        if(Input.GetKey(KeyCode.D) && grounded)
        {
            rb.AddTorque(transform.up * Vector3.Dot(rb.velocity, transform.forward) * turnSpeed);
        }
        float _sideSpeed = Vector3.Dot(transform.right, rb.velocity) * driftTraction; 
        rb.AddForce((-_sideSpeed * transform.right), ForceMode.VelocityChange); 
        float _forwardSpeed = Vector3.Dot(transform.forward, rb.velocity) * forwardTraction; 
        rb.AddForce((-_forwardSpeed * transform.forward), ForceMode.VelocityChange); 
    }
    public float stabiliseDamped = 0.07f;
    public float stabiliseAdjust = 0.07f;
    public bool stabiliseWithInput = false;
    public void Stabilise()
    {
        if(Input.anyKey && !stabiliseWithInput){return;}
        Quaternion deltaQuat = Quaternion.FromToRotation(rb.transform.up, Vector3.up);

        Vector3 axis;
        float angle;
        deltaQuat.ToAngleAxis(out angle, out axis);

        rb.AddTorque(-rb.angularVelocity * stabiliseDamped, ForceMode.Acceleration);

        rb.AddTorque(axis.normalized * angle * stabiliseAdjust, ForceMode.Acceleration);
    }
}
