/******************************************************************************
 * Copyright (c) 2011
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
 * * Neither the name of the company nor the names of its
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

#ifndef INTERACTIOM_CONTROLLER
#define INTERACTIOM_CONTROLLER

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>

#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <tf/transform_broadcaster.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <brics_actuator/CartesianPose.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointTorques.h>

#include <nav_msgs/Odometry.h>
#include "20_sim_interaction_control_base/interaction_control_gravity_compensation.h"

namespace controller {

class InteractionControllerBase;

class Debug {
public:

	Debug(pr2_controller_interface::Controller* controllerPtr) : controllerPtr(controllerPtr) {
	};

	virtual void init() {
	};

	virtual void publish() {
	};
protected:
	pr2_controller_interface::Controller* controllerPtr;
};

class InteractionControllerDebug : public Debug {
public:

	InteractionControllerDebug(pr2_controller_interface::Controller* controllerPtr) : Debug(controllerPtr) {
	};
	virtual void init();
	virtual void publish();
private:

	void publishTf(double* tf, std::string parentFrameId, std::string childFrameId);
	InteractionControllerBase* getControllerPtr();

	tf::TransformBroadcaster br;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointPositions> > gazeboJointPositions;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointPositions> > controllerJointPositions;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointTorques> > gazeboJointTorques;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointTorques> > controllerJointTorques;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> > gazeboJointPose;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> > controllerJointPose;

};

class InteractionControllerBase : public pr2_controller_interface::Controller {
public:

	InteractionControllerBase();
	~InteractionControllerBase();

	void init20SimController();
	bool init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle);
	void starting();
	void update();
	void update20SimControl();

private:
    interaction_control_gravity_compensation autoGenerated20simController; /* auto-generated 20 sim controller    */
	XXDouble u [22 + 1]; /* input parameters for the auto-generated 20 sim controller  */
	XXDouble y [24 + 1]; /* output parameters for the auto-generated 20 sim controller */

	ros::Subscriber subscriber;
	ros::Subscriber subscriberOdometry;
	ros::NodeHandle nodeHandle;
	pr2_mechanism_model::RobotState *robotPtr;
	std::vector<pr2_mechanism_model::JointState*> joints;

	unsigned int loopCount;
	ros::Time currentTime;
	ros::Time lastTime;
	double orientationYPR[3]; //Yaw Pitch Roll position
	double position[3]; // XYZ position
	std::vector <double> targetEfforts;
	double targetSpeed[3];

	void positionCommand(const brics_actuator::CartesianPose &pose);
	void odometryCommand(const nav_msgs::Odometry &odometry);
	bool locked;
	geometry_msgs::Twist currentBaseTwist;
	geometry_msgs::Pose currentBasePose;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > baseTwist;


	Debug* debugInfo;

	friend class InteractionControllerDebug;
};

} // namespace

#endif
