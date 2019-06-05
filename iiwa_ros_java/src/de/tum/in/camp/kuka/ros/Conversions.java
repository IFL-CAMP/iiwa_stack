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

import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

public final class Conversions {

  private Conversions() {}

  /**
   * Generates a iiwa_msgs.JointQuantity message from a double vector
   * 
   * @param vec : double vector
   * @param q : resulting JointQuantity message
   */
  public static void vectorToJointQuantity(double[] vec, iiwa_msgs.JointQuantity q) {
    q.setA1((float) vec[0]);
    q.setA2((float) vec[1]);
    q.setA3((float) vec[2]);
    q.setA4((float) vec[3]);
    q.setA5((float) vec[4]);
    q.setA6((float) vec[5]);
    q.setA7((float) vec[6]);
  }

  /**
   * Generate a double vector from a iiwa_msgs.JointQuantity message
   * 
   * @param q : a JointQuantity message
   * @return a double vector
   */
  public static double[] jointQuantityToVector(iiwa_msgs.JointQuantity q) {
    double[] ret = new double[7];
    ret[0] = q.getA1();
    ret[1] = q.getA2();
    ret[2] = q.getA3();
    ret[3] = q.getA4();
    ret[4] = q.getA5();
    ret[5] = q.getA6();
    ret[6] = q.getA7();
    return ret;
  }

  /**
   * Generates a MatrixRotation from a Quaternion
   * 
   * @param x
   * @param y
   * @param z
   * @param w
   * @return a MatrixRotation
   */
  public static MatrixRotation quatToMatrix(double x, double y, double z, double w) {
    return quatToMatrix((float) x, (float) y, (float) z, (float) w);
  }

  /**
   * Generates a MatrixRotation from a Quaternion
   * 
   * @param x
   * @param y
   * @param z
   * @param w
   * @return a MatrixRotation
   */
  public static MatrixRotation quatToMatrix(float x, float y, float z, float w) throws IllegalArgumentException {
    double sqw = w * w;
    double sqx = x * x;
    double sqy = y * y;
    double sqz = z * z;

    if ((sqx + sqy + sqz + sqw) < 10e-8) { // This is a fairly safe assumption,
                                           // we are expecting the norm to be
                                           // around 1 anyway.
      throw new IllegalArgumentException("Commanded geometry_msgs.Pose contains an invalid quaternion. The norm of the given quaternion has to be 1.");
    }

    MatrixBuilder mb = new MatrixBuilder();

    // invs (inverse square length) is only required if quaternion is not
    // already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    mb.setElement00((sqx - sqy - sqz + sqw) * invs); // since sqw + sqx + sqy +
                                                     // sqz =1/invs*invs
    mb.setElement11((-sqx + sqy - sqz + sqw) * invs);
    mb.setElement22((-sqx - sqy + sqz + sqw) * invs);

    double tmp1 = x * y;
    double tmp2 = z * w;
    mb.setElement10(2.0 * (tmp1 + tmp2) * invs);
    mb.setElement01(2.0 * (tmp1 - tmp2) * invs);

    tmp1 = x * z;
    tmp2 = y * w;
    mb.setElement20(2.0 * (tmp1 - tmp2) * invs);
    mb.setElement02(2.0 * (tmp1 + tmp2) * invs);

    tmp1 = y * z;
    tmp2 = x * w;
    mb.setElement21(2.0 * (tmp1 + tmp2) * invs);
    mb.setElement12(2.0 * (tmp1 - tmp2) * invs);

    return MatrixRotation.of(mb.toMatrix());
  }

  /**
   * Generates a quaternion from a Matrix.
   * <p>
   * Mercilessly copied <url>from https://github.com/libgdx/libgdx/blob/master/gdx
   * /src/com/badlogic/gdx/math/Quaternion.java</url>
   * 
   * @param matrix : the starting matrix
   * @param quaternion : the resulting quaternion
   */
  public static void matrixToQuat(Matrix matrix, Quaternion quaternion) {

    double xx = matrix.getElement00();
    double xy = matrix.getElement01();
    double xz = matrix.getElement02();
    double yx = matrix.getElement10();
    double yy = matrix.getElement11();
    double yz = matrix.getElement12();
    double zx = matrix.getElement20();
    double zy = matrix.getElement21();
    double zz = matrix.getElement22();

    double x, y, z, w; // return

    final double t = xx + yy + zz;

    // we protect the division by s by ensuring that s>=1
    if (t >= 0) { // |w| >= .5
      float s = (float) Math.sqrt(t + 1); // |s|>=1 ...
      w = 0.5f * s;
      s = 0.5f / s; // so this division isn't bad
      x = (zy - yz) * s;
      y = (xz - zx) * s;
      z = (yx - xy) * s;
    }
    else if ((xx > yy) && (xx > zz)) {
      float s = (float) Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
      x = s * 0.5f; // |x| >= .5
      s = 0.5f / s;
      y = (yx + xy) * s;
      z = (xz + zx) * s;
      w = (zy - yz) * s;
    }
    else if (yy > zz) {
      float s = (float) Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
      y = s * 0.5f; // |y| >= .5
      s = 0.5f / s;
      x = (yx + xy) * s;
      z = (zy + yz) * s;
      w = (xz - zx) * s;
    }
    else {
      float s = (float) Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
      z = s * 0.5f; // |z| >= .5
      s = 0.5f / s;
      x = (xz + zx) * s;
      y = (zy + yz) * s;
      w = (yx - xy) * s;
    }

    quaternion.setX(x);
    quaternion.setY(y);
    quaternion.setZ(z);
    quaternion.setW(w);
  }

  /**
   * Converts a geometry_msgs.Pose message to a Transformation object in KUKA APIs
   * 
   * @param rosPose : starting Pose
   * @return resulting Transformation
   * @throws InvalidArgumentException
   */
  public static Transformation rosPoseToKukaTransformation(geometry_msgs.Pose rosPose) throws IllegalArgumentException {
    if (rosPose == null) { return null; }

    float tx = (float) rosTranslationToKuka(rosPose.getPosition().getX());
    float ty = (float) rosTranslationToKuka(rosPose.getPosition().getY());
    float tz = (float) rosTranslationToKuka(rosPose.getPosition().getZ());

    float x = (float) rosPose.getOrientation().getX();
    float y = (float) rosPose.getOrientation().getY();
    float z = (float) rosPose.getOrientation().getZ();
    float w = (float) rosPose.getOrientation().getW();

    MatrixRotation rot = quatToMatrix(x, y, z, w);
    Vector transl = Vector.of(tx, ty, tz);

    Transformation t = Transformation.of(transl, rot);

    return t;
  }

  /**
   * Converts a geometry_msgs.Pose message to a Frame object in KUKA APIs
   * 
   * @param rosPose : starting Pose
   * @return resulting Frame
   * @throws InvalidArgumentException
   */
  public static Frame rosPoseToKukaFrame(geometry_msgs.Pose rosPose) throws IllegalArgumentException {
    return new Frame(rosPoseToKukaTransformation(rosPose));
  }

  /**
   * Converts a geometry_msgs.Pose message to a Frame object in KUKA APIs
   * 
   * @param parent : parent Frame
   * @param rosPose : starting Pose
   * @return resulting Frame
   * @throws InvalidArgumentException
   */
  public static Frame rosPoseToKukaFrame(AbstractFrame parent, geometry_msgs.Pose rosPose) throws IllegalArgumentException {
    return new Frame(parent, rosPoseToKukaTransformation(rosPose));
  }

  /**
   * Converts a Transformation object from KUKA APIs to a geometry_msgs.Pose message
   * 
   * @param kukaTransf : starting Trasnformation
   * @param pose : resulting Pose
   */
  public static void kukaTransformationToRosPose(Transformation kukaTransf, Pose pose) {
    pose.getPosition().setX(kukaTranslationToRos(kukaTransf.getX()));
    pose.getPosition().setY(kukaTranslationToRos(kukaTransf.getY()));
    pose.getPosition().setZ(kukaTranslationToRos(kukaTransf.getZ()));

    Matrix rotationMatrix = kukaTransf.getRotationMatrix();
    matrixToQuat(rotationMatrix, pose.getOrientation());
  }

  /**
   * Converts an iiwa_msgs.JointQuantity to a JointPosition in KUKA APIs
   * 
   * @param rosJointPos : starting JointQuantity
   * @param kukaJointPos : resulting JointPosition
   */
  public static void rosJointQuantityToKuka(iiwa_msgs.JointQuantity rosJointPos, JointPosition kukaJointPos) {
    rosJointQuantityToKuka(rosJointPos, kukaJointPos, 1.0);
  }

  /**
   * Converts an iiwa_msgs.JointQuantity to a JointPosition in KUKA APIs
   * 
   * @param rosJointPos : starting JointQuantity
   * @param kukaJointPos : resulting JointPosition
   * @param scaleFactor : each element of rosJointPos with be scaled using this value
   */
  public static void rosJointQuantityToKuka(iiwa_msgs.JointQuantity rosJointPos, JointPosition kukaJointPos, double scaleFactor) {
    kukaJointPos.set(rosJointPos.getA1() * scaleFactor, rosJointPos.getA2() * scaleFactor, rosJointPos.getA3() * scaleFactor, rosJointPos.getA4() * scaleFactor,
        rosJointPos.getA5() * scaleFactor, rosJointPos.getA6() * scaleFactor, rosJointPos.getA7() * scaleFactor);
  }

  /**
   * Converts an iiwa_msgs.JointQuantity to a double vector.
   * 
   * @param rosJointQuant : starting jointQuantity
   * @return a double vector
   */
  public static double[] rosJointQuantityToArray(iiwa_msgs.JointQuantity rosJointQuant) {
    double[] ret = { rosJointQuant.getA1(), rosJointQuant.getA2(), rosJointQuant.getA3(), rosJointQuant.getA4(), rosJointQuant.getA5(), rosJointQuant.getA6(),
        rosJointQuant.getA7() };
    return ret;
  }

  /**
   * Converts a ROS geometry_msgs.Vector3 to a Java array of doubles.
   */
  public static double[] rosVectorToArray(geometry_msgs.Vector3 vector) {
    double[] ret = { vector.getX(), vector.getY(), vector.getZ() };
    return ret;
  }

  /**
   * Converts the value of a translational element from meters (ROS) to millimeters (KUKA).
   */
  public static double rosTranslationToKuka(double value) {
    return value * 1000.0;
  }

  /**
   * Converts the value of a translational element from millimeters (KUKA) to meters (ROS).
   */
  public static double kukaTranslationToRos(double value) {
    return value / 1000.0;
  }
}
