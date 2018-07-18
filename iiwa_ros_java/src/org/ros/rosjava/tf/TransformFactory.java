package org.ros.rosjava.tf;
import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import tf2_msgs.TFMessage;
import org.ros.rosjava.tf.StampedTransform;

public class TransformFactory {
	private static NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
	private static MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
	
	public static Collection<StampedTransform> fromTfMessage(TFMessage msg) {
		ArrayList<StampedTransform> transforms = new ArrayList<StampedTransform>(msg.getTransforms().size());
		for(int i = 0; i < msg.getTransforms().size(); i++) {
			transforms.add(TransformFactory.msg2transform(msg.getTransforms().get(i)));
		}
		return transforms;
	}

	public static StampedTransform msg2transform(geometry_msgs.TransformStamped msg) {
		return new StampedTransform(
							msg.getHeader().getStamp().totalNsecs(),
							msg.getHeader().getFrameId(),
							msg.getChildFrameId(),
							msg2vector(msg.getTransform().getTranslation()),
							msg2quaternion(msg.getTransform().getRotation())
						);
	}
	
	public static Quat4d msg2quaternion(geometry_msgs.Quaternion q) {
		return new Quat4d(q.getX(), q.getY(), q.getZ(), q.getW());
	}

	public static Vector3d msg2vector(geometry_msgs.Vector3 v) {
		return new Vector3d(v.getX(), v.getY(), v.getZ());
	}

	public static geometry_msgs.TransformStamped tx2msg(StampedTransform tx) {
		geometry_msgs.TransformStamped msg = messageFactory.newFromType(geometry_msgs.TransformStamped._TYPE);
		msg.getHeader().setFrameId(tx.parentFrame);
		msg.setChildFrameId(tx.childFrame);
		msg.getHeader().setStamp(org.ros.message.Time.fromNano(tx.timestamp));

		geometry_msgs.Transform transform = messageFactory.newFromType(geometry_msgs.Transform._TYPE);
		transform.setTranslation(vector2msg(tx.translation));
		transform.setRotation(quaternion2msg(tx.rotation));
		msg.setTransform(transform);

		return msg;
	}
	
	public static geometry_msgs.Vector3 vector2msg(Vector3d v) {
		geometry_msgs.Vector3 msg = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		msg.setX(v.x);
		msg.setY(v.y);
		msg.setZ(v.z);
		return msg;
	}
	
	public static geometry_msgs.Quaternion quaternion2msg(Quat4d q) {
		geometry_msgs.Quaternion msg = messageFactory.newFromType(geometry_msgs.Quaternion._TYPE);
		msg.setX(q.x);
		msg.setY(q.y);
		msg.setZ(q.z);
		msg.setW(q.w);
		return msg;
	}
	
}
