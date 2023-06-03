using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ilikefrogs101.RaycastCarController
{
    public class CarController : MonoBehaviour
    {
        [SerializeField] private Wheel[] wheels;
        [SerializeField] private Transform centerOfMass;
        [SerializeField] private Transform orientation;
        [SerializeField] private Transform raycastPoint;

        private Rigidbody rb;

        public LayerMask drivable;

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
        public bool grounded;
        public float wheelYoffset;

        private int poweredWheelCount;
        private int steeringWheelCount;
        private bool initialised;

        public Vector3 wheelRollAxis;
        public Vector3 wheelTurnAxis;

        private void Awake()
        {
            rb = GetComponent<Rigidbody>();
            rb.centerOfMass = transform.InverseTransformPoint(centerOfMass.position);
            poweredWheelCount = 0;
            steeringWheelCount = 0;
            initialised = true;

            foreach (Wheel wheel in wheels)
            {
                wheel.rotation = wheel.visual.localEulerAngles;

                if (wheel.powered)
                {
                    poweredWheelCount++;
                }
                if (wheel.steering)
                {
                    steeringWheelCount++;
                }
            }
        }

        private void FixedUpdate()
        {
            if (!initialised)
                return;

            orientation.rotation = Quaternion.Euler(0, transform.eulerAngles.y, 0);
            grounded = Physics.Raycast(raycastPoint.position, Vector3.down, groundDistance, drivable, QueryTriggerInteraction.Ignore);

            float rotationAmount = 0f;

            foreach (Wheel wheel in wheels)
            {
                RaycastHit wheelHit;

                if (Physics.Raycast(wheel.wheel.position, -wheel.wheel.up, out wheelHit, maxSuspensionDistance, drivable, QueryTriggerInteraction.Ignore))
                {
                    float damping = dampingFactor * Vector3.Dot(rb.GetPointVelocity(wheel.wheel.position), wheel.wheel.up);
                    rb.AddForceAtPosition(maxSuspensionForce * Time.deltaTime * transform.up * Mathf.Max((maxSuspensionDistance - wheelHit.distance + wheelRadius) / maxSuspensionDistance - damping, 0), wheel.wheel.position);

                    if (wheel.visual)
                    {
                        wheel.visual.position = new Vector3(wheel.visual.position.x, wheelHit.point.y + wheelYoffset, wheel.visual.position.z);

                        float wheelCircumference = 2 * Mathf.PI * wheelRadius;
                        float wheelRotationAmount = (rb.velocity.magnitude * Time.deltaTime) / wheelCircumference;
                        rotationAmount += wheelRotationAmount;

                        wheel.visual.Rotate(wheelRollAxis * rotationAmount);
                    }
                }
            }

            if (stabilise)
            {
                Stabilise();
            }

            if (Input.GetKey(KeyCode.W) && grounded)
            {
                foreach (Wheel wheel in wheels)
                {
                    if (wheel.powered)
                        rb.AddForceAtPosition(orientation.forward * (speed / poweredWheelCount), wheel.wheel.position);
                }
            }
            if (Input.GetKey(KeyCode.S) && grounded)
            {
                foreach (Wheel wheel in wheels)
                {
                    if (wheel.powered)
                        rb.AddForceAtPosition(orientation.forward * -(speed / poweredWheelCount), wheel.wheel.position);
                }
            }
            if (Input.GetKey(KeyCode.A))
            {
                foreach (Wheel wheel in wheels)
                {
                    if (wheel.steering)
                    {
                        if (grounded)
                            rb.AddTorque(wheel.wheel.up * Vector3.Dot(rb.velocity, transform.forward) * -(turnSpeed / steeringWheelCount));
                    }
                }
            }
            if (Input.GetKey(KeyCode.D))
            {
                foreach (Wheel wheel in wheels)
                {
                    if (wheel.steering)
                    {
                        if (grounded)
                            rb.AddTorque(wheel.wheel.up * Vector3.Dot(rb.velocity, transform.forward) * (turnSpeed / steeringWheelCount));
                    }
                }
            }


            float sideSpeed = Vector3.Dot(transform.right, rb.velocity) * driftTraction;
            rb.AddForce((-sideSpeed * transform.right), ForceMode.VelocityChange);
            float forwardSpeed = Vector3.Dot(transform.forward, rb.velocity) * forwardTraction;
            rb.AddForce((-forwardSpeed * transform.forward), ForceMode.VelocityChange);
        }

        public float stabiliseDamped = 0.07f;
        public float stabiliseAdjust = 0.07f;
        public bool stabiliseWithInput = false;

        public void Stabilise()
        {
            if (Input.anyKey && !stabiliseWithInput)
            {
                return;
            }

            Quaternion deltaQuat = Quaternion.FromToRotation(rb.transform.up, Vector3.up);

            Vector3 axis;
            float angle;
            deltaQuat.ToAngleAxis(out angle, out axis);

            rb.AddTorque(-rb.angularVelocity * stabiliseDamped, ForceMode.Acceleration);

            rb.AddTorque(axis.normalized * angle * stabiliseAdjust, ForceMode.Acceleration);
        }
    }

    [System.Serializable]
    public class Wheel
    {
        public Transform wheel;
        public bool powered;
        public bool steering;
        public Transform visual;

        [HideInInspector]
        public Vector3 rotation;
    }
}
