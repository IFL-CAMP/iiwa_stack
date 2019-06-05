/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided
 * that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.time.TimeProvider;

// import com.kuka.generated.ioAccess.MediaFlangeIOGroup; // MEDIAFLANGEIO
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

/**
 * This class implements a ROS Node that publishes the current state of the robot. <br>
 * Messages will be send via topics in this format : <robot name>/state/<iiwa_msgs type> (e.g.
 * MyIIWA/state/CartesianPosition)
 */
public class iiwaPublisher extends AbstractNodeMain {

  // ROSJava Publishers for iiwa_msgs
  // Cartesian Message Publishers
  private Publisher<iiwa_msgs.CartesianPose> cartesianPosePublisher;
  private Publisher<iiwa_msgs.CartesianWrench> cartesianWrenchPublisher;
  // Joint Message Publishers
  private Publisher<iiwa_msgs.JointPosition> jointPositionPublisher;
  private Publisher<iiwa_msgs.JointPositionVelocity> jointPositionVelocityPublisher;
  private Publisher<iiwa_msgs.JointTorque> jointTorquePublisher;
  private Publisher<iiwa_msgs.JointTorque> externalJointTorquePublisher;
  private Publisher<iiwa_msgs.JointVelocity> jointVelocityPublisher;
  // UserKey Event Publisher
  private Publisher<std_msgs.String> iiwaButtonPublisher; // TODO: iiwa_msgs.ButtonEvent
  // JointState publisher (optional)
  private Publisher<sensor_msgs.JointState> jointStatesPublisher;
  private boolean publishJointState = false;
  // DestinationReachedFlag publisher
  private Publisher<std_msgs.Time> destinationReachedPublisher;
  // Publishes the status of the Media Flange button.
  // private Publisher<std_msgs.Bool> mediaFlangeButtonPublisher; // MEDIAFLANGEIO
  // Name to use to build the name of the ROS topics
  private String robotName = "iiwa";

  private LBR robot;

  // Object to easily build iiwa_msgs from the current robot state
  private MessageGenerator helper;

  private ConnectedNode node = null;

  // Cache objects
  private iiwa_msgs.CartesianPose cp;
  private iiwa_msgs.CartesianWrench cw;
  private iiwa_msgs.JointPosition jp;
  private iiwa_msgs.JointPositionVelocity jpv;
  private iiwa_msgs.JointTorque jt;
  private iiwa_msgs.JointTorque ejt;
  private sensor_msgs.JointState js;
  private iiwa_msgs.JointVelocity jv;
  private std_msgs.Time t;

  // private std_msgs.Bool flangeButton; // MEDIAFLANGEIO

  /**
   * Create a ROS node with publishers for a robot state. <br>
   * Node will be running when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
   * 
   * @param robotName : name of the robot, topics will be created accordingly : <robot name>/state/<iiwa_msgs
   *          type> (e.g. MyIIWA/state/CartesianPosition)
   */
  public iiwaPublisher(LBR robot, String robotName, TimeProvider timeProvider) {
    this.robot = robot;
    this.robotName = robotName;
    helper = new MessageGenerator(robotName, timeProvider);

    cp = helper.buildMessage(iiwa_msgs.CartesianPose._TYPE);
    cw = helper.buildMessage(iiwa_msgs.CartesianWrench._TYPE);
    jp = helper.buildMessage(iiwa_msgs.JointPosition._TYPE);
    jpv = helper.buildMessage(iiwa_msgs.JointPositionVelocity._TYPE);
    jt = helper.buildMessage(iiwa_msgs.JointTorque._TYPE);
    ejt = helper.buildMessage(iiwa_msgs.JointTorque._TYPE);
    jv = helper.buildMessage(iiwa_msgs.JointVelocity._TYPE);
    js = helper.buildMessage(sensor_msgs.JointState._TYPE);
    t = helper.buildMessage(std_msgs.Time._TYPE);
    // flangeButton = helper.buildMessage(std_msgs.Bool._TYPE); // MEDIAFLANGEIO
  }

  /**
   * Set if also joint_states should be published
   * 
   * @param publishJointState
   */
  public void setPublishJointStates(boolean publishJointState) {
    this.publishJointState = publishJointState;
  }

  /**
   * Returns the current name used to compose the ROS topics' names for the publishers.
   * <p>
   * e.g. returning "dummy" means that the topics' names will be "dummy/state/...". <br>
   * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is
   * called.
   * 
   * @return the current name to use for ROS topics.
   */
  public String getIIWAName() {
    return robotName;
  }

  /**
   * @see org.ros.node.NodeMain#getDefaultNodeName()
   */
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(robotName + "/publisher");
  }

  /**
   * This method is called when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
   * Do <b>NOT</b> manually call this.
   * <p>
   * 
   * @see org.ros.node.AbstractNodeMain#onStart(org.ros.node.ConnectedNode)
   */
  @Override
  public void onStart(final ConnectedNode connectedNode) {
    node = connectedNode;

    cartesianPosePublisher = connectedNode.newPublisher(robotName + "/state/CartesianPose", iiwa_msgs.CartesianPose._TYPE);
    cartesianWrenchPublisher = connectedNode.newPublisher(robotName + "/state/CartesianWrench", iiwa_msgs.CartesianWrench._TYPE);

    jointPositionPublisher = connectedNode.newPublisher(robotName + "/state/JointPosition", iiwa_msgs.JointPosition._TYPE);
    jointPositionVelocityPublisher = connectedNode.newPublisher(robotName + "/state/JointPositionVelocity", iiwa_msgs.JointPositionVelocity._TYPE);
    jointTorquePublisher = connectedNode.newPublisher(robotName + "/state/JointTorque", iiwa_msgs.JointTorque._TYPE);
    externalJointTorquePublisher = connectedNode.newPublisher(robotName + "/state/ExternalJointTorque", iiwa_msgs.JointTorque._TYPE);
    jointVelocityPublisher = connectedNode.newPublisher(robotName + "/state/JointVelocity", iiwa_msgs.JointVelocity._TYPE);

    iiwaButtonPublisher = connectedNode.newPublisher(robotName + "/state/buttonEvent", std_msgs.String._TYPE);
    jointStatesPublisher = connectedNode.newPublisher(robotName + "/joint_states", sensor_msgs.JointState._TYPE);

    destinationReachedPublisher = connectedNode.newPublisher(robotName + "/state/DestinationReached", std_msgs.Time._TYPE);

    // mediaFlangeButtonPublisher = connectedNode.newPublisher(robotName + "/state/MFButtonState",
    // std_msgs.Bool._TYPE); // MEDIAFLANGEIO
  }

  /**
   * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.
   * <p>
   * Only the nodes that currently have subscribers will publish the messages.<br>
   * <b>Cartesian information published will be relative to the robot's flange</b>
   * 
   * @param robot : the state of this robot will be published
   * @param motion : the dynamic of this motion will be published
   * @throws InterruptedException
   */
  public void publishCurrentState() throws InterruptedException {
    publishCurrentState(robot.getFlange() /* , null */);
  }

  /**
   * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.
   * <p>
   * Only the nodes that currently have subscribers will publish the messages.<br>
   * 
   * @param robot : the state of this robot will be published
   * @param motion : the dynamic of this motion will be published
   * @param frame : the Cartesian information published will be relative to this frame
   * @throws InterruptedException
   */
  public void publishCurrentState(ObjectFrame frame/* , MediaFlangeIOGroup mediaFlange */) throws InterruptedException { // MEDIAFLANGEIO
    if (cartesianPosePublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentCartesianPose(cp, robot, frame);
      helper.incrementSeqNumber(cp.getPoseStamped().getHeader());
      cartesianPosePublisher.publish(cp);
    }
    if (cartesianWrenchPublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentCartesianWrench(cw, robot, frame);
      helper.incrementSeqNumber(cw.getHeader());
      cartesianWrenchPublisher.publish(cw);
    }
    if (jointPositionPublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentJointPosition(jp, robot);
      helper.incrementSeqNumber(jp.getHeader());
      jointPositionPublisher.publish(jp);
    }
    if (jointPositionVelocityPublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentJointPositionVelocity(jpv, robot);
      helper.incrementSeqNumber(jpv.getHeader());
      jointPositionVelocityPublisher.publish(jpv);
    }
    if (jointVelocityPublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentJointVelocity(jv, robot);
      helper.incrementSeqNumber(jv.getHeader());
      jointVelocityPublisher.publish(jv);
    }
    if (jointTorquePublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentJointTorque(jt, robot);
      helper.incrementSeqNumber(jt.getHeader());
      jointTorquePublisher.publish(jt);
    }
    if (externalJointTorquePublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentExternalJointTorque(ejt, robot);
      helper.incrementSeqNumber(ejt.getHeader());
      externalJointTorquePublisher.publish(ejt);
    }

    if (publishJointState && jointStatesPublisher.getNumberOfSubscribers() > 0) {
      helper.getCurrentJointState(js, robot);
      helper.incrementSeqNumber(js.getHeader());
      jointStatesPublisher.publish(js);
    }

    // Uncomment if using a Media Flange IO. // MEDIAFLANGEIO
    // if (mediaFlange != null && mediaFlangeButtonPublisher.getNumberOfSubscribers() > 0) {
    // flangeButton.setData(mediaFlange.getUserButton());
    // mediaFlangeButtonPublisher.publish(flangeButton);
    // }
  }

  /**
   * Publishes the current timestamp on the destinationReached topic.
   */
  public synchronized void publishDestinationReached() {
    if (destinationReachedPublisher.getNumberOfSubscribers() > 0) {
      t.setData(node.getCurrentTime());
      destinationReachedPublisher.publish(t);
    }
  }

  /**
   * Publishes the event of a button on the SmartPad toolbar being <b>pressed</b>
   * 
   * @param name : name of the button
   */
  public void publishButtonPressed(String name) {
    final std_msgs.String msg = iiwaButtonPublisher.newMessage();
    msg.setData(name + "_pressed");
    iiwaButtonPublisher.publish(msg);
  }

  /**
   * Publishes the event of a button on the SmartPad toolbar being <b>released</b>
   * 
   * @param name : name of the button
   */
  public void publishButtonReleased(String name) {
    final std_msgs.String msg = iiwaButtonPublisher.newMessage();
    msg.setData(name + "_released");
    iiwaButtonPublisher.publish(msg);
  }
}
