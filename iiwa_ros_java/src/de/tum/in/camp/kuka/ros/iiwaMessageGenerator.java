/** 
 * Technische Universitaet Muenchen
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultaet fuer Informatik / I16, Boltzmannstrasse 3, 85748 Garching bei Muenchen, Germany
 * http://campar.in.tum.de
 * 
 * @author Salvatore Virga
 * 
 * TODO LICENSE
 * 
 */

// IIWAROS import
package de.tum.in.camp.kuka.shared.ros;

// ROS import
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

// KUKA import
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

/**
 * This class helps the building of iiwa_msgs ROS messages,
 * it gives a collection of methods to build the messages from the current state of a LBR iiwa Robot.
 * For Cartesian messages, it's possible to pass a reference frames to acquire the message values.
 * If no reference frames is passed, the flange frame is used.
 */
public class iiwaMessageGenerator {
	private static final String FLANGE_FRAME_ID = "iiwa_link_7";

	// Objects to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time = new WallTimeProvider();

	// Converts 3x3 Matrix into a 9 elements vector
	private static void reshapeRotation(Matrix m, double[] array) {
		for(int i = 0; i < 9; ++i) array[i] = m.get(i/3, i%3);
	}

	/**
	 * Builds a CartesianPosition message given a LBR iiwa Robot.<p>
	 * The Cartesian position will be the obtained from current Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianPosition message.
	 */
	public iiwa_msgs.CartesianPosition buildCartesianPosition(LBR robot) {
		geometry_msgs.Point point = messageFactory.newFromType(geometry_msgs.Point._TYPE);
		point.setX(robot.getCurrentCartesianPosition(robot.getFlange()).getX());
		point.setY(robot.getCurrentCartesianPosition(robot.getFlange()).getY());
		point.setZ(robot.getCurrentCartesianPosition(robot.getFlange()).getZ());

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(FLANGE_FRAME_ID);
		header.setStamp(time.getCurrentTime());

		geometry_msgs.PointStamped ps = messageFactory.newFromType(geometry_msgs.PointStamped._TYPE);
		ps.setPoint(point);
		ps.setHeader(header);

		iiwa_msgs.CartesianPosition cp = messageFactory.newFromType(iiwa_msgs.CartesianPosition._TYPE);
		cp.setPosition(ps);

		return cp;
	}

	/**
	 * Builds a CartesianPosition message given a LBR iiwa Robot and a frame of reference.<p>
	 * The Cartesian position will be the obtained from the given Frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame to set the values of the Cartesian position.
	 * @return built CartesianPosition message.
	 */
	public iiwa_msgs.CartesianPosition buildCartesianPosition(LBR robot, ObjectFrame frame) {
		geometry_msgs.Point point = messageFactory.newFromType(geometry_msgs.Point._TYPE);
		point.setX(robot.getCurrentCartesianPosition(frame).getX());
		point.setY(robot.getCurrentCartesianPosition(frame).getY());
		point.setZ(robot.getCurrentCartesianPosition(frame).getZ());

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(frame.getName());
		header.setStamp(time.getCurrentTime());

		geometry_msgs.PointStamped ps = messageFactory.newFromType(geometry_msgs.PointStamped._TYPE);
		ps.setPoint(point);
		ps.setHeader(header);

		iiwa_msgs.CartesianPosition cp = messageFactory.newFromType(iiwa_msgs.CartesianPosition._TYPE);
		cp.setPosition(ps);

		return cp;
	}

	/**
	 * Builds a CartesianRotation message given a LBR iiwa Robot.<p>
	 * The rotation will refer to the current Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianRotation message.
	 */
	public iiwa_msgs.CartesianRotation buildCartesianRotation(LBR robot) {
		double[] rotation = new double[9];
		Transformation transform = robot.getCurrentCartesianPosition(robot.getFlange()).transformationFromWorld();
		Matrix rotationMatrix = transform.getRotationMatrix();
		reshapeRotation(rotationMatrix,rotation);

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(FLANGE_FRAME_ID);
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.CartesianRotation cr = messageFactory.newFromType(iiwa_msgs.CartesianRotation._TYPE);
		cr.setRotation(rotation);
		cr.setHeader(header);

		return cr;
	}

	/**
	 * Builds a CartesianRotation message given a LBR iiwa Robot and a frame of reference.<p>
	 * The rotation will refer to the given frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the rotation refers to.
	 * @return built CartesianRotation message.
	 */
	public iiwa_msgs.CartesianRotation buildCartesianRotation(LBR robot, ObjectFrame frame) {
		double[] rotation = new double[9];
		Transformation transform = robot.getCurrentCartesianPosition(frame).transformationFromWorld();
		Matrix rotationMatrix = transform.getRotationMatrix();
		reshapeRotation(rotationMatrix,rotation);

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(frame.getName());
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.CartesianRotation cr = messageFactory.newFromType(iiwa_msgs.CartesianRotation._TYPE);
		cr.setRotation(rotation);
		cr.setHeader(header);

		return cr;
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot.<p>
	 * The <b>velocity will be set to zero</b>, 
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianVelocity message.
	 */
	public iiwa_msgs.CartesianVelocity buildCartesianVelocity(LBR robot) {
		double[] velocity = new double[9];

		// Velocity is set to zero!
		// TODO: Compute Cartesian velocity !

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(FLANGE_FRAME_ID);
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.CartesianVelocity cv = messageFactory.newFromType(iiwa_msgs.CartesianVelocity._TYPE);
		cv.setVelocity(velocity);
		cv.setHeader(header);

		return cv;
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot and a frame of reference.<p>
	 * The <b>velocity will be set to zero</b>, 
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the velocity refers to.
	 * @return built CartesianVelocity message.
	 */
	public iiwa_msgs.CartesianVelocity buildCartesianVelocity(LBR robot, ObjectFrame frame) {
		double[] velocity = new double[9];

		// Velocity is set to zero!
		// TODO: Compute Cartesian velocity !

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(frame.getName());
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.CartesianVelocity cv = messageFactory.newFromType(iiwa_msgs.CartesianVelocity._TYPE);
		cv.setVelocity(velocity);
		cv.setHeader(header);

		return cv;
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot.<p>
	 * The wrench values will refer to the Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianWrench message.
	 */
	public iiwa_msgs.CartesianWrench buildCartesianWrench(LBR robot) {
		geometry_msgs.Vector3 force = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		force.setX(robot.getExternalForceTorque(robot.getFlange()).getForce().getX());
		force.setY(robot.getExternalForceTorque(robot.getFlange()).getForce().getY());
		force.setZ(robot.getExternalForceTorque(robot.getFlange()).getForce().getZ());

		geometry_msgs.Vector3 torque = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		torque.setX(robot.getExternalForceTorque(robot.getFlange()).getTorque().getX());
		torque.setY(robot.getExternalForceTorque(robot.getFlange()).getTorque().getY());
		torque.setZ(robot.getExternalForceTorque(robot.getFlange()).getTorque().getZ());

		geometry_msgs.Wrench wrench = messageFactory.newFromType(geometry_msgs.Wrench._TYPE);
		wrench.setForce(force);
		wrench.setTorque(torque);

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(FLANGE_FRAME_ID);
		header.setStamp(time.getCurrentTime());

		geometry_msgs.WrenchStamped ws = messageFactory.newFromType(geometry_msgs.WrenchStamped._TYPE);
		ws.setWrench(wrench);
		ws.setHeader(header);

		iiwa_msgs.CartesianWrench cw = messageFactory.newFromType(iiwa_msgs.CartesianWrench._TYPE);
		cw.setWrench(ws);
		return cw;
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot and a frame of reference.<p>
	 * The wrench values will refer to the given frame,
	 * the message header is set to current time and according frame name.<br> 
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the wrench refers to.
	 * @return built CartesianWrench message.
	 */
	public iiwa_msgs.CartesianWrench buildCartesianWrench(LBR robot, ObjectFrame frame) {
		geometry_msgs.Vector3 force = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		force.setX(robot.getExternalForceTorque(frame).getForce().getX());
		force.setY(robot.getExternalForceTorque(frame).getForce().getY());
		force.setZ(robot.getExternalForceTorque(frame).getForce().getZ());

		geometry_msgs.Vector3 torque = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		torque.setX(robot.getExternalForceTorque(frame).getTorque().getX());
		torque.setY(robot.getExternalForceTorque(frame).getTorque().getY());
		torque.setZ(robot.getExternalForceTorque(frame).getTorque().getZ());

		geometry_msgs.Wrench wrench = messageFactory.newFromType(geometry_msgs.Wrench._TYPE);
		wrench.setForce(force);
		wrench.setTorque(torque);

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId(frame.getName());
		header.setStamp(time.getCurrentTime());

		geometry_msgs.WrenchStamped ws = messageFactory.newFromType(geometry_msgs.WrenchStamped._TYPE);
		ws.setWrench(wrench);
		ws.setHeader(header);

		iiwa_msgs.CartesianWrench cw = messageFactory.newFromType(iiwa_msgs.CartesianWrench._TYPE);
		cw.setWrench(ws);
		return cw;
	}

	/**
	 * Builds a JointPosition message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointPosition message.
	 */
	public iiwa_msgs.JointPosition buildJointPosition(LBR robot) {
		double[] position = new double[7];
		position = robot.getCurrentJointPosition().getInternalArray();

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId("Robot");
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.JointPosition jp = messageFactory.newFromType(iiwa_msgs.JointPosition._TYPE);
		jp.setPosition(position);
		jp.setHeader(header);
		return jp;
	}

	/**
	 * Builds a JointTorque message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointTorque message.
	 */
	public iiwa_msgs.JointTorque buildJointTorque(LBR robot) {
		double[] torque = new double[7];
		torque = robot.getMeasuredTorque().getTorqueValues(); // TODO: check if correct values

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId("Robot");
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.JointTorque jt = messageFactory.newFromType(iiwa_msgs.JointTorque._TYPE);
		jt.setTorque(torque);
		jt.setHeader(header);
		return jt;
	}

	/**
	 * Builds a JointVelocity message given a LBR iiwa Robot.<p>
	 * The <b>velocity will be set to zero</b>,<br>
	 * the message header is set to current time.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointVelocity message.
	 */
	public iiwa_msgs.JointVelocity buildJointVelocity(LBR robot) {
		double[] velocity = new double[7];
		// TODO: build velocity vector

		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
		header.setFrameId("Robot");
		header.setStamp(time.getCurrentTime());

		iiwa_msgs.JointVelocity jv = messageFactory.newFromType(iiwa_msgs.JointVelocity._TYPE);
		jv.setVelocity(velocity);
		jv.setHeader(header);
		return jv;
	}
}
