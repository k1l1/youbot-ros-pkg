/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Alexey Zakharov, Yury Brodskiy
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/


#include <boost/units/systems/si.hpp>
#include <boost/units/physical_dimensions.hpp>
#include <boost/units/io.hpp>
#include <geometry_msgs/Quaternion.h>

#include "youbot_interaction_controller.h"
#include "20_sim_interaction_control/common/xxmatrix.h"
#include "pluginlib/class_list_macros.h"
#include <tf/transform_datatypes.h>

PLUGINLIB_DECLARE_CLASS(youbot_description, InteractionController, controller::InteractionController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

    InteractionController* InteractionControllerDebug::getControllerPtr() {
        return dynamic_cast<InteractionController*>(this->controllerPtr);
    }

    void InteractionControllerDebug::publishTf(double* tf, string parent, string child) {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(tf[3],tf[7],tf[11]));
        btMatrix3x3 rotMatrix(tf[0],tf[1],tf[2],
                              tf[4],tf[5],tf[6],
                              tf[8],tf[9],tf[10]);
        tf::Quaternion quat;
        rotMatrix.getRotation(quat);
        trans.setRotation(quat);
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),parent,child));
    }

    void InteractionControllerDebug::init() {

        ros::NodeHandle &nodeHandle = getControllerPtr()->nodeHandle;
        gazeboJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "gazebo_joint_positions", 1));
        gazeboJointPositions->lock();
        controllerJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "controller_joint_positions", 1));
        controllerJointPositions->lock();
        gazeboJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques> (nodeHandle, "gazebo_joint_torques", 1));
        gazeboJointTorques->lock();
        controllerJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques> (nodeHandle, "controller_joint_torques", 1));
        controllerJointTorques->lock();
        gazeboJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "gazebo_joint_pose", 1));
        gazeboJointPose->lock();
        controllerJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "controller_joint_pose", 1));
        controllerJointPose->lock();

    }

    void InteractionControllerDebug::publish() {

        vector <brics_actuator::JointValue> positionsGazebo;
        vector <brics_actuator::JointValue> positionsController;
        vector <brics_actuator::JointValue> torquesGazebo;

        InteractionController* ctrlPtr = getControllerPtr();

        unsigned int size = ctrlPtr->joints.size();
        positionsGazebo.resize(size);
        positionsController.resize(size);
        torquesGazebo.resize(size);

        XXMatrix* matrix = ctrlPtr->autoGenerated20simController.m_M;
        double* joint1 = matrix[41].mat;
        publishTf(joint1,"/arm_link_0", "/joint1");
        double* joint2 = matrix[42].mat;
        publishTf(joint2,"/arm_link_0", "/joint2");
        double* joint3 = matrix[43].mat;
        publishTf(joint3,"/arm_link_0", "/joint3");
        double* joint4 = matrix[44].mat;
        publishTf(joint4,"/arm_link_0", "/joint4");
        double* joint5 = matrix[45].mat;
        publishTf(joint5,"/arm_link_0", "/joint5");
        double* tip = matrix[3].mat;
        publishTf(tip,"/arm_link_0", "tip");

        ROS_INFO("Рас: %f,%f,%f,%f,%f,%f,%f,%f\n", matrix[0].mat[0], matrix[0].mat[1], matrix[0].mat[2], matrix[0].mat[3], matrix[0].mat[4], matrix[0].mat[5], matrix[0].mat[6], matrix[0].mat[7]);

        std::vector<pr2_mechanism_model::JointState*>::iterator joinsIterator;

        using namespace boost::units;
        unsigned int j = 0;
        for (joinsIterator = ctrlPtr->joints.begin(); joinsIterator != ctrlPtr->joints.end(); ++joinsIterator) {
            positionsGazebo[j].joint_uri = (*joinsIterator)->joint_->name;
            positionsGazebo[j].value = (*joinsIterator)->position_;
            positionsGazebo[j].unit = to_string(si::radians);
            positionsGazebo[j].timeStamp = ctrlPtr->currentTime;

            positionsController[j].joint_uri = (*joinsIterator)->joint_->name;
            //positionsController[j].value = u[28+j]; //CHANGE THAT!
            positionsController[j].unit = to_string(si::radians);
            positionsController[j].timeStamp = ctrlPtr->currentTime;

            torquesGazebo[j].joint_uri = (*joinsIterator)->joint_->name;
            torquesGazebo[j].value = (*joinsIterator)->commanded_effort_;
            torquesGazebo[j].unit = to_string(si::newton_meter);
            torquesGazebo[j].timeStamp = ctrlPtr->currentTime;
            ++j;
        }

        gazeboJointPositions->msg_.positions = positionsGazebo;
        controllerJointPositions->msg_.positions = positionsController;
        gazeboJointTorques->msg_.torques = torquesGazebo;

        gazeboJointPositions->unlockAndPublish();
        controllerJointPositions->unlockAndPublish();
        gazeboJointTorques->unlockAndPublish();

    }


    InteractionController::InteractionController()
    : robotPtr(NULL) {
        loopCount = 0;
        this->position[0] = 0;
		this->position[1] = 0;
		this->position[2] = 0;

		this->orientation[0] = 0;
		this->orientation[1] = 0;
		this->orientation[2] = 0;
		this->orientation[3] = 0;
#ifdef DEBUG_INFO
        debugInfo = new InteractionControllerDebug(this);
#else
        debugInfo = new Debug(this);
#endif

    }

    InteractionController::~InteractionController() {
        subscriber.shutdown();
        autoGenerated20simController.Terminate(u, y);
        delete debugInfo;
    }

/*  auto-generated by 20sim;
    initialize the inputs and outputs with correct initial values */
    void InteractionController::init20SimController() {

        u[0] = 0.0;		/* outputForces.f */
        u[1] = 0.0;
        u[2] = 0.0;
        u[3] = 0.0;
        u[4] = 0.0;
        u[5] = 0.0;
        u[6] = 0.0;
        u[7] = 0.0;
        u[8] = 0.0;		/* q */
        u[9] = 0.0;
        u[10] = 0.0;
        u[11] = 0.0;
        u[12] = 0.0;
        u[13] = 0.0;
        u[14] = 0.0;
        u[15] = 0.0;
        u[16] = 1.0;		/* setPointH */
        u[17] = 0.0;
        u[18] = 0.0;
        u[19] = 0.0;
        u[20] = 0.0;
        u[21] = 1.0;
        u[22] = 0.0;
        u[23] = 0.0;
        u[24] = 0.0;
        u[25] = 0.0;
        u[26] = 1.0;
        u[27] = 0.0;
        u[28] = 0.0;
        u[29] = 0.0;
        u[30] = 0.0;
        u[31] = 1.0;

        y[0] = 0.0;		/* outputForces.e */
        y[1] = 0.0;
        y[2] = 0.0;
        y[3] = 0.0;
        y[4] = 0.0;
        y[5] = 0.0;
        y[6] = 0.0;
        y[7] = 0.0;

        y[8] = 0.0;		/* tipH */
        y[9] = 0.0;
        y[10] = 0.0;
        y[11] = 0.0;

        y[12] = 0.0;
        y[13] = 0.0;
        y[14] = 0.0;
        y[15] = 0.0;

        y[16] = 0.0;
        y[17] = 0.0;
        y[18] = 0.0;
        y[19] = 0.0;

        y[20] = 0.0;
        y[21] = 0.0;
        y[22] = 0.0;
        y[23] = 0.0;
        ROS_INFO("20sim init.\n");
        autoGenerated20simController.Initialize(u, y, 0.0);

    }

    bool InteractionController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle) {
        using namespace XmlRpc;
        this->nodeHandle = nodeHandle;
        this->robotPtr = robotPtr;

        ROS_INFO("Initializing interaction control for the youbot arm...\n");

        // Gets all of the joint pointers from the RobotState to a joints vector
        XmlRpc::XmlRpcValue jointNames;
        if (!nodeHandle.getParam("joints", jointNames)) {
            ROS_ERROR("No joints given. (namespace: %s)", nodeHandle.getNamespace().c_str());
            return false;
        }

        if (jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Malformed joint specification.  (namespace: %s)", nodeHandle.getNamespace().c_str());
            return false;
        }

        for (unsigned int i = 0; i < static_cast<unsigned int> (jointNames.size()); ++i) {
            XmlRpcValue &name = jointNames[i];
            if (name.getType() != XmlRpcValue::TypeString) {
                ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)", nodeHandle.getNamespace().c_str());
                return false;
            }

            pr2_mechanism_model::JointState *jointStatePtr = robotPtr->getJointState((std::string)name);
            if (jointStatePtr == NULL) {
                ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name).c_str(), nodeHandle.getNamespace().c_str());
                return false;
            }

            joints.push_back(jointStatePtr);
        }

        // Ensures that all the joints are calibrated.
        for (unsigned int i = 0; i < joints.size(); ++i) {
            if (!joints[i]->calibrated_) {
                ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
                return false;
            }
        }

        // Initializing target efforts vector
        targetEfforts.resize(joints.size());

        // Subscribing for an input pose command
        subscriber = nodeHandle.subscribe("command", 1, &InteractionController::positionCommand, this);

        // Initializing 20Sim controller
        init20SimController();

        // Initializing twist publisher for the base
        ROS_INFO("base_confghfhgtroller/command\n");
        baseTwist.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist> (nodeHandle, "base_controller/command", 1));
        baseTwist->lock();

        subscriberOdometry = nodeHandle.subscribe("base_odometry/odometry", 1, &InteractionController::odometryCommand, this);
        locked = false;

        // Initializing debug output
        debugInfo->init();

        return true;
    }

    void InteractionController::starting() {
         // Initializing timer
        currentTime = robotPtr->getTime();
        lastTime = currentTime;
    }

    void InteractionController::udpate20SimControl() {


     //   u[0] = currentBaseTwist.angular.z;		/* outputForces.f */
      //  u[1] = currentBaseTwist.linear.y;
       // u[2] = currentBaseTwist.linear.x;
        //u[3] = -joints[0]->velocity_;
        //u[4] = joints[1]->velocity_;
        //u[5] = -joints[2]->velocity_;
        //u[6] = joints[3]->velocity_;
        //u[7] = joints[4]->velocity_;


  //      u[8] = tf::getYaw(currentBasePose.orientation);		/* q */
        //ROS_INFO("Yaw=%f\n", u[8]);
   //     u[9] = currentBasePose.position.y;
   //     u[10] = currentBasePose.position.x;
        u[11] = joints[0]->position_ - 170 * M_PI / 180; /* q */
        u[12] = joints[1]->position_ - 65 * M_PI / 180;
        u[13] = joints[2]->position_ - 146 * M_PI / 180;
        u[14] = joints[3]->position_ - 102.5 * M_PI / 180;
        u[15] = joints[4]->position_ - 167.5 * M_PI / 180;

        btQuaternion quat(orientation[0], orientation[1], orientation[2], orientation[3]);
        btMatrix3x3 orientationMatrix(quat);



        u[16] = 1;//orientationMatrix[0][0];		/* setPointH */
        u[17] = 0;//orientationMatrix[1][0];
        u[18] = 0;//orientationMatrix[2][0];
        u[19] = position[0];
        u[20] = 0;//orientationMatrix[0][1];
        u[21] = 1;//orientationMatrix[1][1];
        u[22] = 0;//orientationMatrix[2][1];
        u[23] = position[1];
        u[24] = 0;//orientationMatrix[0][2];
        u[25] = 0;//orientationMatrix[1][2];
        u[26] = 1;//orientationMatrix[2][2];
        u[27] = position[2];
        u[28] = 0.0;
        u[29] = 0.0;
        u[30] = 0.0;
        u[31] = 1.0;
       // ROS_INFO("%f , %f , %f\n", position[0], position[1], position[2]);

        autoGenerated20simController.Calculate(u, y);
// The limits signs and rotation directions are inconsitent
        targetEfforts[0] = y[3];//- u[11]*100;  // -170 +170, clockwise
        targetEfforts[1] = y[4];//- u[12]*1000; // -65  +90,  counterclockwise
        targetEfforts[2] = y[5];//- u[13]*100;  // +146 -151, clockwise 
        targetEfforts[3] = y[6];//- u[14]*100;  // -102 +102, counterclockwise
        targetEfforts[4] = y[7];//- u[15]*100;  // -167 +167, clockwise
        ROS_INFO("\njoints position :%f, %f,%f,%f %f,\n",u[11]*180/3.14,u[12]*180/3.14,u[13]*180/3.14,u[14]*180/3.14,u[15]*180/3.14);
//        ROS_INFO("joints torques :%f, %f,%f,%f %f,\n",y[3],y[4],y[5],y[6],y[7]);
        ROS_INFO("\njoints torques :%f, %f,%f,%f %f,\n", targetEfforts[0], targetEfforts[1], targetEfforts[2], targetEfforts[3], targetEfforts[4]);
      // ROS_INFO("tipH test :%f, %f,%f,%f \t position: %f,%f,%f\n",y[8],y[13],y[18],y[23],y[11],y[15],y[19]);

      //  y[0] = 0.0;		/* outputForces.e */
      /*  y[1] = 0.0;
        y[2] = 0.0;
        y[3] = 0.0;
        y[4] = 0.0;
        y[5] = 0.0;
        y[6] = 0.0;
        y[7] = 0.0;*/
      //  y[8] = 0.0;		/* tipH */
      /*  y[9] = 0.0;
        y[10] = 0.0;
        y[11] = 0.0;
        y[12] = 0.0;
        y[13] = 0.0;
        y[14] = 0.0;
        y[15] = 0.0;
        y[16] = 0.0;
        y[17] = 0.0;
        y[18] = 0.0;
        y[19] = 0.0;
        y[20] = 0.0;
        y[21] = 0.0;
        y[22] = 0.0;
        y[23] = 0.0;
*/

   /*     // setting current joint velocities to the 20Sim controller
        u[0] = joints[0]->velocity_;
        u[1] = joints[1]->velocity_;
        u[2] = joints[2]->velocity_;
        u[3] = joints[3]->velocity_;
        u[4] = joints[4]->velocity_;

        // setting current joint positions + offset to the 20Sim controller
        u[23] = -joints[0]->position_ + 170 * M_PI / 180;
        u[24] = joints[1]->position_ - 65 * M_PI / 180;
        u[25] = joints[2]->position_ + 146 * M_PI / 180;
        u[26] = joints[3]->position_ - 102.5 * M_PI / 180;
        u[27] = joints[4]->position_ + 167.5 * M_PI / 180;

        // setting target pose for the end effector
        u[28] = position[0];
        u[29] = position[1];
        u[30] = position[2];
        u[31] = orientation[0];
        u[32] = orientation[1];
        u[33] = orientation[2];
        u[34] = orientation[3];

        // running 20Sim controller
        autoGenerated20simController.Calculate(u, y);

        targetEfforts[0] = y[0];
        targetEfforts[1] = y[1];
        targetEfforts[2] = y[2];
        targetEfforts[3] = y[3];
        targetEfforts[4] = y[4];
*/

    }

    void InteractionController::update() {

        baseTwist->trylock();

        currentTime = robotPtr->getTime();
        ros::Duration dt = currentTime - lastTime;
        lastTime = currentTime;

        // Initializing error vector
        if (!locked) //{
            udpate20SimControl();
            //ROS_INFO ("Current speed is: %f, %f\n", currentBaseTwist.angular.z, currentBaseTwist.linear.y);

       // } else
         //   ROS_INFO ("locked\n");


        // Doing control here, calculating and applying the efforts
        for (unsigned int i = 0; i < joints.size(); ++i) {
            joints[i]->commanded_effort_ += targetEfforts[i];
        }

        // Sending control comands for the base

        //gazeboJointPositions->msg_.positions = positionsGazebo;
        //baseTwist->msg_ = ;
        //geometry_msgs::Twist twist;
       // twist.angular.z = 0.1;
        //baseTwist->msg_ = twist;
        //baseTwist->unlockAndPublish();

        // if debug mode is set, publishing debug TFs and controller information
        if (loopCount % 10 == 0) {
            debugInfo->publish();
        }
        ++loopCount;
    }



    void InteractionController::positionCommand(const brics_actuator::CartesianPose &pose) {

        using namespace boost::units;

        brics_actuator::CartesianVector tipPosition;
		geometry_msgs::Quaternion tipOrientation;
		tipPosition = pose.position;
		tipOrientation = pose.orientation;

		if (tipPosition.unit != to_string(si::meter))
            ROS_ERROR("Position value is set in the inpcompatible units %s, expecting meters", tipPosition.unit.c_str());

        this->position[0] = tipPosition.x;
		this->position[1] = tipPosition.y;
		this->position[2] = tipPosition.z;

		this->orientation[0] = tipOrientation.x;
		this->orientation[1] = tipOrientation.y;
		this->orientation[2] = tipOrientation.z;
		this->orientation[3] = tipOrientation.w;

    }

    void InteractionController::odometryCommand(const nav_msgs::Odometry &odometry) {
        locked = true;
        currentBaseTwist = odometry.twist.twist;
        currentBasePose = odometry.pose.pose;
        locked = false;

    }

}
