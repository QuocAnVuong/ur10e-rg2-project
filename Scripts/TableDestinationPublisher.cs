using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TableDestinationPublisher : MonoBehaviour
{
    [SerializeField]
    string m_TopicName = "/collision_object";

    // [SerializeField]
    // string m_AttachTopicName = "/attached_collision_object";

    [SerializeField]
    GameObject m_Table;

    [SerializeField]
    GameObject m_Target;


    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);
        // m_Ros.RegisterPublisher<AttachedCollisionObjectMsg>(m_AttachTopicName);
    }

    public void Publish()
    {
        var tablePose = new PoseMsg
        {
            position = m_Table.transform.position.To<FLU>(),
            orientation = m_Table.transform.rotation.To<FLU>()
        };
        tablePose.position.z = tablePose.position.z - 0.32;

        // var targetPose = new PoseMsg
        // {
        //     position = m_Target.transform.position.To<FLU>(),
        //     orientation = m_Target.transform.rotation.To<FLU>()
        // };
        // targetPose.position.z = targetPose.position.z - 0.3;

        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_link"
            },
            id = "table",
            operation = CollisionObjectMsg.ADD,
            primitive_poses = new List<PoseMsg> { tablePose}.ToArray(),
            primitives = new List<SolidPrimitiveMsg> {
                new SolidPrimitiveMsg
                {
                    type = SolidPrimitiveMsg.BOX,
                    dimensions = new double[] { 5.0, 5.0, 0.64} // Example dimensions
                },
                // new SolidPrimitiveMsg
                // {
                //     type = SolidPrimitiveMsg.BOX,
                //     dimensions = new double[] { 0.2, 0.2, 0.05 } // Example dimensions of the target
                // }
            }.ToArray()
        };

        // Publish the collision object
        m_Ros.Publish(m_TopicName, collisionObject);

        // var attachedCollisionObject = new AttachedCollisionObjectMsg
        // {
        //     link_name = "base_link",
        //     object_field = collisionObject // Reference the same collision object
        // };

        // m_Ros.Publish(m_AttachTopicName, attachedCollisionObject);

    }
}
