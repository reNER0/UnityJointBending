using System.Collections.Generic;
using UnityEngine;


public enum PhysicsProcessThread
{
    visualThread,
    physicsThread
}


[RequireComponent(typeof(ConfigurableJoint))]
public class JointBending : MonoBehaviour
{
    [Tooltip("The minimum angle at which the joint begins to deform")]
    [SerializeField] private float bendingAngle = 5f;
    [Tooltip("Delay in iterations between bending calculated and updated. Set low value ​​for accuracy, high value ​​for performance")]
    [Range(0, 100)]
    [SerializeField] private int physicsIterationsDelay = 5;
    [SerializeField] private PhysicsProcessThread physicsProcessType = PhysicsProcessThread.physicsThread;

    private int currentIteration;
    private Dictionary<ConfigurableJoint, Quaternion> JointsToStartRotationsDictionary;


    private void Awake()
    {
        var joints = GetComponents<ConfigurableJoint>();

        JointsToStartRotationsDictionary = new Dictionary<ConfigurableJoint, Quaternion>();
        foreach (var joint in joints)
        {
            var startRotation = GetJointLocalRotationInConnectedBodySpace(joint);
            JointsToStartRotationsDictionary.Add(joint, startRotation);
        }
    }

    private void Update()
    {
        if (physicsProcessType != PhysicsProcessThread.visualThread)
            return;

        ProcessPhysics();
    }

    private void FixedUpdate()
    {
        if (physicsProcessType != PhysicsProcessThread.physicsThread)
            return;

        ProcessPhysics();
    }


    public void SetBendingAngle(float angle)
    {
        bendingAngle = Mathf.Clamp(angle, 0, 180);
    }

    public void SetPhysicsSettings(PhysicsProcessThread physicsType, int physicsDelay)
    {
        physicsProcessType = physicsType;
        physicsIterationsDelay = physicsDelay;
    }


    private void ProcessPhysics()
    {
        if (currentIteration < physicsIterationsDelay)
        {
            currentIteration++;
            return;
        }

        currentIteration = 0;

        foreach (var jointToStartRotations in JointsToStartRotationsDictionary)
        {
            SetTargetAngle(jointToStartRotations.Key, jointToStartRotations.Value);
        }
    }

    private void SetTargetAngle(ConfigurableJoint joint, Quaternion startRotation)
    {
        var currentProcessedAngles = GetProcessedAngles(GetJointRotationInJointAxisSpace(joint, startRotation).eulerAngles);
        var targetProcessedAngles = GetProcessedAngles(joint.targetRotation.eulerAngles);

        var deltaAngles = currentProcessedAngles - targetProcessedAngles;


        var newAngles = targetProcessedAngles;


        if (deltaAngles.x > bendingAngle)
            newAngles.x = currentProcessedAngles.x + bendingAngle;

        if (deltaAngles.y > bendingAngle)
            newAngles.y = currentProcessedAngles.y + bendingAngle;

        if (deltaAngles.z > bendingAngle)
            newAngles.z = currentProcessedAngles.z + bendingAngle;


        if (deltaAngles.x < -bendingAngle)
            newAngles.x = currentProcessedAngles.x - bendingAngle;

        if (deltaAngles.y < -bendingAngle)
            newAngles.y = currentProcessedAngles.y - bendingAngle;

        if (deltaAngles.z < -bendingAngle)
            newAngles.z = currentProcessedAngles.z - bendingAngle;


        joint.targetRotation = Quaternion.Euler(newAngles);
    }

    private Vector3 GetProcessedAngles(Vector3 angles)
    {
        return new Vector3(GetProcessedAngle(angles.x), GetProcessedAngle(angles.y), GetProcessedAngle(angles.z));

        float GetProcessedAngle(float angle)
        {
            return angle < 180
                ? angle
                : angle - 360;
        }
    }

    private Quaternion GetJointLocalRotationInConnectedBodySpace(ConfigurableJoint joint)
    {
        return Quaternion.Inverse(joint.connectedBody.transform.rotation) * joint.transform.rotation;
    }

    private Quaternion GetJointRotationInJointAxisSpace(ConfigurableJoint joint, Quaternion initLocalRot)
    {
        Quaternion JointAxisCoordRot = Quaternion.LookRotation(Vector3.Cross(joint.axis, joint.secondaryAxis), joint.secondaryAxis);
        return SwitchQuaternionCoordinateSystem(Quaternion.Inverse(Quaternion.Inverse(initLocalRot) * Quaternion.Inverse(joint.connectedBody.transform.rotation) * joint.transform.rotation), Quaternion.Inverse(JointAxisCoordRot));
    }

    private Quaternion SwitchQuaternionCoordinateSystem(Quaternion q, Quaternion coordRotation)
    {
        return coordRotation * q * Quaternion.Inverse(coordRotation);
    }
}
