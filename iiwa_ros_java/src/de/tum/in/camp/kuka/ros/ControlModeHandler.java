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

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.task.ITaskLogger;

public class ControlModeHandler {

  private LBR robot = null;
  private Tool tool = null;
  private ObjectFrame endpointFrame = null;
  private ITaskLogger logger = null;
  private iiwaPublisher publisher = null;
  private Configuration configuration = null;

  private iiwa_msgs.ConfigureControlModeRequest lastSmartServoRequest = null;

  private MessageGenerator helper = null;
  private GoalReachedEventListener handler = null;

  private Workpiece workpiece = null;

  IMotionControlMode currentControlMode = null;

  public ControlModeHandler(LBR robot, Tool tool, ObjectFrame endpointFrame, iiwaPublisher publisher, iiwaActionServer actionServer, Configuration configuration) {
    this.robot = robot;
    this.tool = tool;
    this.endpointFrame = endpointFrame;
    this.publisher = publisher;
    this.configuration = configuration;
    helper = new MessageGenerator(configuration.getRobotName(), configuration.getTimeProvider());
    handler = new GoalReachedEventListener(this.publisher, actionServer);
  }

  public void setEndpointFrame(ObjectFrame endpointFrame) {
    this.endpointFrame = endpointFrame;
  }

  public void setWorkpiece(Workpiece workpiece) {
    this.workpiece = workpiece;
  }

  public void setLastSmartServoRequest(iiwa_msgs.ConfigureControlModeRequest request) {
    this.lastSmartServoRequest = request;
  }

  public SmartServo changeSmartServoControlMode(SmartServo motion, IMotionControlMode controlMode) {
    SmartServo oldMotion = motion;
    motion = createSmartServoMotion();
    if (controlMode == null || controlMode instanceof PositionControlMode) {
      motion.setMode(new PositionControlMode());
    }
    else {
      validateForImpedanceMode();
      motion.setMode(controlMode);
    }
    switchMotion(motion, oldMotion);
    return motion;
  }

  public SmartServoLIN changeSmartServoControlMode(SmartServoLIN motion, IMotionControlMode controlMode) {
    SmartServoLIN oldMotion = motion;
    motion = createSmartServoLinMotion();
    if (controlMode == null || controlMode instanceof PositionControlMode) {
      motion.setMode(new PositionControlMode());
    }
    else {
      validateForImpedanceMode();
      motion.setMode(controlMode);
    }
    switchMotion(motion, oldMotion);
    return motion;
  }

  public SmartServo changeSmartServoControlMode(SmartServo motion, iiwa_msgs.ConfigureControlModeRequest request) {
    SmartServo oldMotion = motion;
    motion = createSmartServoMotion();
    if (request != null) {
      if (request.getControlMode() != iiwa_msgs.ControlMode.POSITION_CONTROL) {
        validateForImpedanceMode();
      }
      motion.setMode(buildMotionControlMode(request));
    }
    else if (lastSmartServoRequest != null) {
      if (lastSmartServoRequest.getControlMode() != iiwa_msgs.ControlMode.POSITION_CONTROL) {
        validateForImpedanceMode();
      }
      motion.setMode(buildMotionControlMode(lastSmartServoRequest));
    }
    else {
      motion.setMode(new PositionControlMode());
    }
    switchMotion(motion, oldMotion);
    return motion;
  }

  public SmartServoLIN changeSmartServoControlMode(SmartServoLIN motion, iiwa_msgs.ConfigureControlModeRequest request) {
    SmartServoLIN oldMotion = motion;
    motion = createSmartServoLinMotion();
    if (request != null) {
      if (request.getControlMode() != iiwa_msgs.ControlMode.POSITION_CONTROL) {
        validateForImpedanceMode();
      }
      motion.setMode(buildMotionControlMode(request));
    }
    else if (lastSmartServoRequest != null) {
      if (lastSmartServoRequest.getControlMode() != iiwa_msgs.ControlMode.POSITION_CONTROL) {
        validateForImpedanceMode();
      }
      motion.setMode(buildMotionControlMode(lastSmartServoRequest));
    }
    else {
      motion.setMode(new PositionControlMode());
    }
    switchMotion(motion, oldMotion);
    return motion;
  }

  // TODO: doc
  @SuppressWarnings("rawtypes")
  public void switchMotion(ServoMotion motion, ServoMotion oldMotion) {
    if (oldMotion != null) {
      oldMotion.getRuntime().stopMotion();
    }

    while (((SunriseExecutionService) robot.getController().getExecutionService()).isPaused()) {
      ThreadUtil.milliSleep(10);
    }

    endpointFrame.moveAsync(motion);
    motion.getRuntime(true).updateWithRealtimeSystem();
    motion.getRuntime().setGoalReachedEventHandler(handler);
  }

  /**
   * Given a current SmartServoLin motion, it return a SmartServo motion with the same control mode.
   */
  public SmartServo switchToSmartServo(SmartServoLIN linearMotion) {
    IMotionControlMode currentMode = linearMotion.getMode();
    if (currentMode == null) {
      currentMode = new PositionControlMode();
    }
    if (!(currentMode instanceof PositionControlMode)) {
      validateForImpedanceMode();
    }

    SmartServo newMotion = createSmartServoMotion();
    newMotion.setMode(currentMode);
    currentControlMode = currentMode;
    switchMotion(newMotion, linearMotion);
    return newMotion;
  }

  /**
   * Given a current SmartServo motion, it return a SmartServoLIN motion with the same control mode.
   */
  public SmartServoLIN switchToSmartServoLIN(SmartServo motion) {
    IMotionControlMode currentMode = motion.getMode();
    if (currentMode == null) {
      currentMode = new PositionControlMode();
    }

    if (!(currentMode instanceof PositionControlMode)) {
      validateForImpedanceMode();
    }

    SmartServoLIN newMotion = createSmartServoLinMotion();
    newMotion.setMode(currentMode);
    currentControlMode = currentMode;
    switchMotion(newMotion, motion);
    return newMotion;
  }

  /**
   * Validates the robot for Impedance control, it something fails here Impedance control is not possible.
   * Reasons might be: - too much force is applied on the robot when the Impedance control mode is selected -
   * the robot is in a singularity
   */
  private void validateForImpedanceMode() {
    if (workpiece != null) {
      ServoMotion.validateForImpedanceMode(workpiece);
    }
    else if (tool != null) {
      ServoMotion.validateForImpedanceMode(tool);
    }
    else {
      ServoMotion.validateForImpedanceMode(robot);
    }
  }

  /**
   * Generates a new smartServoMotion with the current parameters.
   * 
   * @return
   */
  public SmartServo createSmartServoMotion() {
    SmartServo motion = new SmartServo(robot.getCurrentJointPosition());
    motion.setMinimumTrajectoryExecutionTime(configuration.getMinTrajExecTime());
    motion.setTimeoutAfterGoalReach(configuration.getTimeoutAfterGoalReach());
    SpeedLimits.applySpeedLimits(motion);
    return motion;
  }

  public SmartServoLIN createSmartServoLinMotion() {
    SmartServoLIN linearMotion = new SmartServoLIN(robot.getCurrentCartesianPosition(endpointFrame));
    linearMotion.setReferenceFrame(World.Current.getRootFrame());
    linearMotion.setMinimumTrajectoryExecutionTime(configuration.getMinTrajExecTime());
    linearMotion.setTimeoutAfterGoalReach(configuration.getTimeoutAfterGoalReach());
    SpeedLimits.applySpeedLimits(linearMotion);
    return linearMotion;
  }

  /**
   * Given the parameters from the SmartServo service, it builds up the new control mode to use.
   * 
   * @param request : parameters from the ConfigureSmartServo service
   * @return resulting control mode
   */
  public IMotionControlMode buildMotionControlMode(iiwa_msgs.ConfigureControlModeRequest request) {
    currentControlMode = null;

    if (request == null) {
      currentControlMode = new PositionControlMode(true);
    }
    else {
      switch (request.getControlMode()) {

        case iiwa_msgs.ControlMode.POSITION_CONTROL: {
          currentControlMode = new PositionControlMode(true);
          break;
        }

        case iiwa_msgs.ControlMode.JOINT_IMPEDANCE: {
          currentControlMode = buildJointImpedanceControlMode(request);
          break;
        }

        case iiwa_msgs.ControlMode.CARTESIAN_IMPEDANCE: {
          currentControlMode = buildCartesianImpedanceControlMode(request);
          break;
        }

        case iiwa_msgs.ControlMode.DESIRED_FORCE: {
          CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
          CartDOF direction = selectDegreeOfFreedom(request.getDesiredForce().getCartesianDof());

          if (direction != null && request.getDesiredForce().getDesiredStiffness() >= 0) {
            cscm = CartesianSineImpedanceControlMode.createDesiredForce(direction, request.getDesiredForce().getDesiredForce(), request.getDesiredForce().getDesiredStiffness());
            addControlModeLimits(cscm, request.getLimits());
            currentControlMode = cscm;
          }
          break;
        }

        case iiwa_msgs.ControlMode.SINE_PATTERN: {
          CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
          CartDOF direction = selectDegreeOfFreedom(request.getSinePattern().getCartesianDof());

          if (direction != null && request.getSinePattern().getFrequency() >= 0 && request.getSinePattern().getAmplitude() >= 0 && request.getSinePattern().getStiffness() >= 0) {
            cscm = CartesianSineImpedanceControlMode.createSinePattern(direction, request.getSinePattern().getFrequency(), request.getSinePattern().getAmplitude(), request
                .getSinePattern().getStiffness());
            addControlModeLimits(cscm, request.getLimits());
            currentControlMode = cscm;
          }
          break;
        }

        default:
          logger.error("Control Mode not supported.");
          throw new UnsupportedControlModeException(); // this should just not
                                                       // happen
      }
    }

    if (currentControlMode != null) {
      return currentControlMode;
    }
    else {
      throw new UnsupportedControlModeException();
    }
  }

  /**
   * Generates a JointImpedanceControlMode from the given request.
   * 
   * @param request
   * @return
   */
  private JointImpedanceControlMode buildJointImpedanceControlMode(iiwa_msgs.ConfigureControlModeRequest request) {
    JointImpedanceControlMode jcm = new JointImpedanceControlMode(robot.getJointCount());

    iiwa_msgs.JointQuantity stiffness = request.getJointImpedance().getJointStiffness();
    if (helper.isJointQuantityGreaterEqualThan(stiffness, 0)) {
      jcm.setStiffness(Conversions.jointQuantityToVector(stiffness));
    }

    iiwa_msgs.JointQuantity damping = request.getJointImpedance().getJointDamping();
    if (helper.isJointQuantityGreaterEqualThan(damping, 0)) {
      jcm.setDamping(Conversions.jointQuantityToVector(damping));
    }
    return jcm;
  }

  /**
   * Generates a CartesianImpedanceControlMode from the given request.
   * 
   * @param request
   * @return
   */
  private CartesianImpedanceControlMode buildCartesianImpedanceControlMode(iiwa_msgs.ConfigureControlModeRequest request) {
    CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

    iiwa_msgs.CartesianQuantity stiffness = request.getCartesianImpedance().getCartesianStiffness();
    if (stiffness.getX() >= 0) {
      ccm.parametrize(CartDOF.X).setStiffness(stiffness.getX());
    }
    if (stiffness.getY() >= 0) {
      ccm.parametrize(CartDOF.Y).setStiffness(stiffness.getY());
    }
    if (stiffness.getZ() >= 0) {
      ccm.parametrize(CartDOF.Z).setStiffness(stiffness.getZ());
    }
    if (stiffness.getA() >= 0) {
      ccm.parametrize(CartDOF.A).setStiffness(stiffness.getA());
    }
    if (stiffness.getB() >= 0) {
      ccm.parametrize(CartDOF.B).setStiffness(stiffness.getB());
    }
    if (stiffness.getC() >= 0) {
      ccm.parametrize(CartDOF.C).setStiffness(stiffness.getC());
    }

    iiwa_msgs.CartesianQuantity damping = request.getCartesianImpedance().getCartesianDamping();
    if (damping.getX() > 0) {
      ccm.parametrize(CartDOF.X).setDamping(damping.getX());
    }
    if (damping.getY() > 0) {
      ccm.parametrize(CartDOF.Y).setDamping(damping.getY());
    }
    if (damping.getZ() > 0) {
      ccm.parametrize(CartDOF.Z).setDamping(damping.getZ());
    }
    if (damping.getA() > 0) {
      ccm.parametrize(CartDOF.A).setDamping(damping.getA());
    }
    if (damping.getB() > 0) {
      ccm.parametrize(CartDOF.B).setDamping(damping.getB());
    }
    if (damping.getC() > 0) {
      ccm.parametrize(CartDOF.C).setDamping(damping.getC());
    }

    if (request.getCartesianImpedance().getNullspaceStiffness() >= 0) {
      ccm.setNullSpaceStiffness(request.getCartesianImpedance().getNullspaceStiffness());
    }
    if (request.getCartesianImpedance().getNullspaceDamping() > 0) {
      ccm.setNullSpaceDamping(request.getCartesianImpedance().getNullspaceDamping());
    }

    addControlModeLimits(ccm, request.getLimits());
    return ccm;
  }

  /**
   * Transforms a iiwa_msgs.DOF to a KUKA CartDOF object
   * 
   * @param dof
   * @return
   */
  private CartDOF selectDegreeOfFreedom(int dof) {
    CartDOF direction = null;
    switch (dof) {
      case iiwa_msgs.DOF.X: {
        direction = CartDOF.X;
        break;
      }
      case iiwa_msgs.DOF.Y: {
        direction = CartDOF.Y;
        break;
      }
      case iiwa_msgs.DOF.Z: {
        direction = CartDOF.Z;
        break;
      }
      case iiwa_msgs.DOF.A: {
        direction = CartDOF.A;
        break;
      }
      case iiwa_msgs.DOF.B: {
        direction = CartDOF.B;
        break;
      }
      case iiwa_msgs.DOF.C: {
        direction = CartDOF.C;
        break;
      }
      case iiwa_msgs.DOF.ROT: {
        direction = CartDOF.ROT;
        break;
      }
      case iiwa_msgs.DOF.TRANSL: {
        direction = CartDOF.TRANSL;
        break;
      }
      case iiwa_msgs.DOF.ALL: {
        direction = CartDOF.ALL;
        break;
      }
      default: {
        logger.error("Wrong direction given, use [1,2,3,4,5,6,7,8,9] for directions [X,Y,Z,A,B,C,ROT,TRANSL,ALL] respectively.");
        break;
      }
    }
    return direction;
  }

  /**
   * Adds Cartesian limits - maxPathDeviation, maxCartesianVelocity, maxControlForce - to a
   * CartesianImpedanceControlMode
   * 
   * @param controlMode
   * @param limits
   */
  private void addControlModeLimits(CartesianImpedanceControlMode controlMode, iiwa_msgs.CartesianControlModeLimits limits) {
    iiwa_msgs.CartesianQuantity maxPathDeviation = limits.getMaxPathDeviation();
    if (helper.isCartesianQuantityGreaterThan(maxPathDeviation, 0)) {
      controlMode.setMaxPathDeviation(maxPathDeviation.getX(), maxPathDeviation.getY(), maxPathDeviation.getZ(), maxPathDeviation.getA(), maxPathDeviation.getB(),
          maxPathDeviation.getC());
    }

    iiwa_msgs.CartesianQuantity maxControlForce = limits.getMaxControlForce();
    if (helper.isCartesianQuantityGreaterThan(maxControlForce, 0)) {
      controlMode.setMaxControlForce(maxControlForce.getX(), maxControlForce.getY(), maxControlForce.getZ(), maxControlForce.getA(), maxControlForce.getB(),
          maxControlForce.getC(), limits.getMaxControlForceStop());
    }

    iiwa_msgs.CartesianQuantity maxCartesianVelocity = limits.getMaxCartesianVelocity();
    if (helper.isCartesianQuantityGreaterThan(maxCartesianVelocity, 0)) {
      controlMode.setMaxCartesianVelocity(maxCartesianVelocity.getX(), maxCartesianVelocity.getY(), maxCartesianVelocity.getZ(), maxCartesianVelocity.getA(),
          maxCartesianVelocity.getB(), maxCartesianVelocity.getC());
    }
  }

  /**
   * Checks if a SmartServoMode is of the same type as a MotionControlMode from KUKA APIs
   * 
   * @return boolean
   */
  public boolean isSameControlMode(IMotionControlMode kukacm, int roscm) {
    if (kukacm == null) { return false; }
    String roscmname = null;
    switch (roscm) {
      case iiwa_msgs.ControlMode.POSITION_CONTROL:
        roscmname = "PositionControlMode";
        break;
      case iiwa_msgs.ControlMode.JOINT_IMPEDANCE:
        roscmname = "JointImpedanceControlMode";
        break;
      case iiwa_msgs.ControlMode.CARTESIAN_IMPEDANCE:
        roscmname = "CartesianImpedanceControlMode";
        break;
      case iiwa_msgs.ControlMode.DESIRED_FORCE:
        roscmname = "CartesianSineImpedanceControlMode";
        break;
      case iiwa_msgs.ControlMode.SINE_PATTERN:
        roscmname = "CartesianSineImpedanceControlMode";
        break;
    }
    String kukacmname = kukacm.getClass().getSimpleName();

    return roscmname.equals(kukacmname);
  }

  public IMotionControlMode getControlMode() {
    currentControlMode = buildMotionControlMode(lastSmartServoRequest);
    return currentControlMode;
  }

  public void disableSmartServo(SmartServo motion) {
    if (currentControlMode != null) {
      if (!(currentControlMode instanceof PositionControlMode)) {
        changeSmartServoControlMode(motion, new PositionControlMode(true));
      }
    }
    motion.getRuntime().stopMotion();
  }

  public void disableSmartServo(SmartServoLIN motion) {
    if (currentControlMode != null) {
      if (!(currentControlMode instanceof PositionControlMode)) {
        changeSmartServoControlMode(motion, new PositionControlMode(true));
      }
    }
    motion.getRuntime().stopMotion();
  }

  public SmartServo enableSmartServo(SmartServo motion) {
    return changeSmartServoControlMode(motion, lastSmartServoRequest);
  }

  public SmartServoLIN enableSmartServo(SmartServoLIN linearMotion) {
    return changeSmartServoControlMode(linearMotion, lastSmartServoRequest);
  }
}
