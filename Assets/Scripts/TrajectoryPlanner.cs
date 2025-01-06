using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "ur10e_rg2_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR10e;
    public GameObject UR10e { get => m_UR10e; set => m_UR10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(180, 90, 0);
    // TODO: Adjust for better position offset
    readonly Vector3 m_PickPoseOffset = Vector3.down * 0.6f + Vector3.forward * 0.2f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    // TODO: Handle gripper later
    ArticulationBody m_Gripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR10e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // TODO: Handle gripper later
        // Find left and right fingers
        // var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        // var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";
        var gripper = linkName + "/onrobot_rg2_base_link/left_outer_knuckle";
        m_Gripper = m_UR10e.transform.Find(gripper).GetComponent<ArticulationBody>();
        if (m_Gripper == null)
        {
            Debug.LogError("Finger joint not found in hierarchy!");
        }
        else
        {
            Debug.Log("Finger joint found and assigned.");
        }

        // m_RightGripper = m_UR10e.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        // m_LeftGripper = m_UR10e.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        // var leftDrive = m_LeftGripper.xDrive;
        // var rightDrive = m_RightGripper.xDrive;

        // leftDrive.target = -0.01f;
        // rightDrive.target = 0.01f;

        // m_LeftGripper.xDrive = leftDrive;
        // m_RightGripper.xDrive = rightDrive;

        if (m_Gripper != null)
        {
            var drive = m_Gripper.xDrive;
            drive.stiffness = 1000f;      // Adjust for responsiveness
            drive.damping = 100f;        // Adjust for smooth motion
            drive.forceLimit = 1000f;    // Maximum force applied
            drive.target = 0.785398f; // Target position for closing the gripper
            m_Gripper.xDrive = drive;

            Debug.Log("Gripper Closed.");
        }
        else
        {
            Debug.LogError("Gripper joint not found.");
        }

        }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        // TODO: Handle gripper later
        Debug.Log("Gripper Opened.");
        // var leftDrive = m_LeftGripper.xDrive;
        // var rightDrive = m_RightGripper.xDrive;

        // leftDrive.target = 0.01f;
        // rightDrive.target = -0.01f;

        // m_LeftGripper.xDrive = leftDrive;
        // m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>Ur10eMoveitJoints</returns>
    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(180, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        Debug.Log($"Pick Pose: Position: {request.pick_pose.position.x}, {request.pick_pose.position.y}, {request.pick_pose.position.z} | Orientation: {request.pick_pose.orientation.x}, {request.pick_pose.orientation.y}, {request.pick_pose.orientation.z}, {request.pick_pose.orientation.w}");

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.Log($"Place Pose: Position: {request.place_pose.position.x}, {request.place_pose.position.y}, {request.place_pose.position.z} | Orientation: {request.place_pose.orientation.x}, {request.place_pose.orientation.y}, {request.place_pose.orientation.z}, {request.place_pose.orientation.w}");

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from ur10e_rg2_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}