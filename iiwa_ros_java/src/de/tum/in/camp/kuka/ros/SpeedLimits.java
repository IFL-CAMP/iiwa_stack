/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de 
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems 
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany 
 * http://www6.in.tum.de 
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

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.SplineMotionJP;

import iiwa_msgs.SetPTPCartesianSpeedLimitsRequest;
import iiwa_msgs.SetPTPJointSpeedLimitsRequest;
import iiwa_msgs.SetSmartServoJointSpeedLimitsRequest;
import iiwa_msgs.SetSmartServoLinSpeedLimitsRequest;

public class SpeedLimits {
  /**
   * Changes the robot's override velocity by 'slowly' adjusting the speed until the target override value has
   * been reached
   */
  protected static class OverrideRampThread extends Thread {
    private long timeLength;
    private int steps;
    private boolean running = false;
    private double targetOverride;

    public OverrideRampThread(double targetOverride, long timeLength, int steps) {
      this.targetOverride = targetOverride;
      this.timeLength = timeLength;
      this.steps = steps;
    }

    /**
     * Request thread to end
     * 
     * @param asynchronous: If true the method will block until the thread has terminated
     */
    public void requestStop(boolean asynchronous) {
      synchronized (this) {
        running = false;
      }

      if (asynchronous) { return; }

      while (running) {
        waitUntil(1);
      }
    }

    /**
     * True if thread has not yet finished execution
     * 
     * @return
     */
    public boolean isRunning() {
      return running;
    }

    /**
     * 
     */
    @Override
    public void run() {
      synchronized (this) {
        running = true;
      }

      double currentOverride = appControl.getApplicationOverride();
      double overrideStep = (targetOverride - currentOverride) / steps;

      while (Math.abs(targetOverride - currentOverride) < 1.e-3 && running) {
        double nextOverrride = currentOverride + overrideStep;

        if (overrideStep < 0 && nextOverrride < targetOverride) {
          // we are reducing the speed and the nextOverride is smaller than the target override
          nextOverrride = targetOverride;
        }
        else if (overrideStep > 0 && targetOverride > nextOverrride) {
          // we are increasing the speed and the nextOverride is greate than the target override
          nextOverrride = targetOverride;
        }

        appControl.setApplicationOverride(nextOverrride);
        currentOverride = appControl.getApplicationOverride();
        waitUntil(timeLength / steps);
      }

      synchronized (this) {
        running = false;
      }
    }

    /**
     * Sleeps until a specific number of Milliseconds have passed
     * 
     * @param ms
     */
    private void waitUntil(long ms) {
      long _before = System.currentTimeMillis();
      long _now = 0;

      do {
        _now = System.currentTimeMillis();
        try {
          Thread.sleep(1);
        }
        catch (InterruptedException e) {
          // ignore - we simply loop until the requested duration is over...
        }
      }
      while ((_now - _before) < ms && running);
    }
  }

  protected static final long OVERRIDE_RAMP_TIME_LENGTH_MS = 200;
  protected static final int OVERRIDE_RAMP_NUM_STEPS = 10;
  private static IApplicationControl appControl;
  protected static OverrideRampThread rampThread = null;

  // Overall override factor
  private static double overrideReduction = 1.0; // relative

  // Joint motion limits
  private static double ss_relativeJointVelocity = 1.0; // relative
  private static double ss_relativeJointAcceleration = 1.0; // relative
  private static double ss_overrideJointAcceleration = 1.0; // relative, between 0.0 and 10.0

  // SmartServo Cartesian motion limits
  private static double[] ss_maxTranslationalVelocity = { 1.0, 1.0, 1.0 }; // m/s
  private static double[] ss_maxRotationalVelocity = { 0.5, 0.5, 0.5 }; // rad/s

  // TODO:
  // public static double ss_maxTranslationalAcceleration =
  // public static double ss_maxRotationalAcceleration =
  // public static double ss_maxNullSpaceVelocity = ???;
  // public static double ss_maxNullSpaceAcceleration = ???;

  // PTP Joint motion limits
  private static double ptp_relativeJointVelocity = 1.0; // relative
  private static double ptp_relativeJointAcceleration = 1.0; // relative

  // PTP Cartesian motion limits
  private static double ptp_maxCartesianVelocity = 1.0; // m/s
  private static double ptp_maxOrientationVelocity = 0.5; // rad/s
  private static double ptp_maxCartesianAcceleration = 0.2; // m/s^2
  private static double ptp_maxOrientationAcceleration = 0.1; // rad/s^2
  private static double ptp_maxCartesianJerk = -1.0; // m/s^3
  private static double ptp_maxOrientationJerk = -1.0; // rad/s^3

  public static void init(Configuration configuration, IApplicationControl appControl) {
    SpeedLimits.appControl = appControl;
    overrideReduction = appControl.getApplicationOverride();

    Logger.debug("Loading speed limits from configuration");

    ss_relativeJointVelocity = configuration.getSSRelativeJointVelocity();
    ss_relativeJointAcceleration = configuration.getSSRelativeJointAcceleration();

    ss_maxTranslationalVelocity = configuration.getSSMaxTranslationVelocity();
    ss_maxRotationalVelocity = configuration.getSSmaxOrientationVelocity();

    Logger.debug("SmartServo relativeJointVelocity: " + ss_relativeJointVelocity);
    Logger.debug("SmartServo relativeJointAcceleration: " + ss_relativeJointAcceleration);
    Logger.debug("SmartServo maxTranslationalVelocity: [" + ss_maxTranslationalVelocity[0] + ", " + ss_maxTranslationalVelocity[1] + ", " + ss_maxTranslationalVelocity[2] + "]");
    Logger.debug("SmartServo maxRotationalVelocity: [" + ss_maxRotationalVelocity[0] + ", " + ss_maxRotationalVelocity[1] + ", " + ss_maxRotationalVelocity[2] + "]");

    ptp_relativeJointVelocity = configuration.getPTPRelativeJointVelocity();
    ptp_relativeJointAcceleration = configuration.getPTPJointAcceleration();

    ptp_maxCartesianVelocity = configuration.getPTPMaxCartesianVelocity();
    ptp_maxOrientationVelocity = configuration.getPTPMaxOrientationVelocity();
    ptp_maxCartesianAcceleration = configuration.getPTPMaxCartesianAcceleration();
    ptp_maxOrientationAcceleration = configuration.getPTPMaxOrientationAccelration();
    ptp_maxCartesianJerk = configuration.getPTPMaxCartesianJerk();
    ptp_maxOrientationJerk = configuration.getPTPMaxOrientationJerk();

    Logger.debug("PTP relativeJointVelocity: " + ptp_relativeJointVelocity);
    Logger.debug("PTP relativeJointAcceleration: " + ptp_relativeJointAcceleration);
    Logger.debug("PTP maxCartesianVelocity: " + ptp_maxCartesianVelocity);
    Logger.debug("PTP maxOrientationVelocity: " + ptp_maxOrientationVelocity);
    Logger.debug("PTP maxCartesianAcceleration: " + ptp_maxCartesianAcceleration);
    Logger.debug("PTP maxOrientationAcceleration: " + ptp_maxOrientationAcceleration);
    Logger.debug("PTP maxCartesianJerk: " + ptp_maxCartesianJerk);
    Logger.debug("PTP maxOrientationJerk: " + ptp_maxOrientationJerk);
  }

  public static void setOverrideRecution(double override, boolean ramp) {
    if (override < 0.0) {
      SpeedLimits.overrideReduction = 0.0;
    }
    else if (override > 1.0) {
      SpeedLimits.overrideReduction = 1.0;
    }
    else {
      SpeedLimits.overrideReduction = override;
    }

    if (rampThread != null && rampThread.isRunning()) {
      rampThread.requestStop(true);
    }

    if (ramp) {
      rampThread = new OverrideRampThread(overrideReduction, OVERRIDE_RAMP_TIME_LENGTH_MS, OVERRIDE_RAMP_NUM_STEPS);
      rampThread.start();
    }
    else {
      appControl.setApplicationOverride(overrideReduction);
    }
  }

  public double getOverrideReduction() {
    return overrideReduction;
  }

  /**
   * Set PTP joint speed limits based on ROS service request
   * 
   * @param srvReq
   */
  public static void setPTPJointSpeedLimits(SetPTPJointSpeedLimitsRequest srvReq) {
    ptp_relativeJointVelocity = srvReq.getJointRelativeVelocity();
    ptp_relativeJointAcceleration = srvReq.getJointRelativeAcceleration();
  }

  /**
   * Set Cartesian PTP speed values based on ROS service request
   * 
   * @param srvReq
   */
  public static void setPTPCartesianSpeedLimits(SetPTPCartesianSpeedLimitsRequest srvReq) {
    ptp_maxCartesianVelocity = srvReq.getMaxCartesianVelocity();
    ptp_maxOrientationVelocity = srvReq.getMaxOrientationVelocity();
    ptp_maxCartesianAcceleration = srvReq.getMaxCartesianAcceleration();
    ptp_maxOrientationAcceleration = srvReq.getMaxOrientationAcceleration();
    ptp_maxCartesianJerk = srvReq.getMaxCartesianJerk();
    ptp_maxOrientationJerk = srvReq.getMaxOrientationJerk();
  }

  /**
   * Set SmartServo joint speed values based on ROS service request
   * 
   * @param srvReq
   */
  public static void setSmartServoJointSpeedLimits(SetSmartServoJointSpeedLimitsRequest srvReq) {
    ss_relativeJointVelocity = srvReq.getJointRelativeVelocity();
    ss_relativeJointAcceleration = srvReq.getJointRelativeAcceleration();
    ss_overrideJointAcceleration = srvReq.getOverrideJointAcceleration();
  }

  /**
   * Set SmartServoLin speed values based on ROS service request
   * 
   * @param srvReq
   */
  public static void setSmartServoLinSpeedLimits(SetSmartServoLinSpeedLimitsRequest srvReq) {
    ss_maxTranslationalVelocity = Conversions.rosVectorToArray(srvReq.getMaxCartesianVelocity().getLinear());
    ss_maxRotationalVelocity = Conversions.rosVectorToArray(srvReq.getMaxCartesianVelocity().getAngular());
  }

  /**
   * Applies the configured speed limits on a given SmartServo motion container
   * 
   * @param motion
   */
  public static void applySpeedLimits(SmartServo motion) {
    if (ss_relativeJointVelocity > 0) {
      motion.setJointVelocityRel(ss_relativeJointVelocity);
    }
    if (ss_relativeJointAcceleration > 0) {
      motion.setJointAccelerationRel(ss_relativeJointAcceleration);
    }
    if (ss_overrideJointAcceleration > 0) {
      motion.overrideJointAcceleration(ss_overrideJointAcceleration);
    }
  }

  public static void applySpeedLimits(SplineMotionJP<?> motion) {
    if (ptp_relativeJointVelocity > 0) {
      motion.setJointVelocityRel(ptp_relativeJointVelocity);
    }
    if (ptp_relativeJointAcceleration > 0) {
      motion.setJointAccelerationRel(ptp_relativeJointAcceleration);
    }
  }

  /**
   * Applies the configured speed limits on a given Cartesian PTP motion container
   * 
   * @param cartesianMotion
   */
  public static void applySpeedLimits(SplineMotionCP<?> cartesianMotion) {
    if (ptp_maxCartesianVelocity > 0) {
      // Transform to mm/s.
      cartesianMotion.setCartVelocity(Conversions.rosTranslationToKuka(ptp_maxCartesianVelocity));
    }
    if (ptp_maxOrientationVelocity > 0) {
      cartesianMotion.setOrientationVelocity(ptp_maxOrientationVelocity);
    }
    if (ptp_maxCartesianAcceleration > 0) {
      // Transform to mm/s.
      cartesianMotion.setCartAcceleration(Conversions.rosTranslationToKuka(ptp_maxCartesianAcceleration));
    }
    if (ptp_maxOrientationAcceleration > 0) {
      cartesianMotion.setOrientationAcceleration(ptp_maxOrientationAcceleration);
    }
    if (ptp_maxCartesianJerk > 0) {
      // Transform to mm/s.
      cartesianMotion.setCartJerk(Conversions.rosTranslationToKuka(ptp_maxCartesianJerk));
    }
    if (ptp_maxOrientationJerk > 0) {
      cartesianMotion.setOrientationJerk(ptp_maxOrientationJerk);
    }
  }

  /**
   * Applies the configured speed limits on a given SmartServoLIN motion container
   * 
   * @param linMotion
   */
  public static void applySpeedLimits(SmartServoLIN linMotion) {
    if (ss_maxTranslationalVelocity[0] > 0 || ss_maxTranslationalVelocity[1] > 0 || ss_maxTranslationalVelocity[2] > 0) {
      // Transform to mm/s before setting the values.
      linMotion.setMaxTranslationVelocity(new double[] { Conversions.rosTranslationToKuka(ss_maxTranslationalVelocity[0]),
          Conversions.rosTranslationToKuka(ss_maxTranslationalVelocity[1]), Conversions.rosTranslationToKuka(ss_maxTranslationalVelocity[2]) });
    }
    if (ss_maxRotationalVelocity[0] > 0 || ss_maxRotationalVelocity[1] > 0 || ss_maxRotationalVelocity[2] > 0) {
      linMotion.setMaxOrientationVelocity(new double[] { ss_maxRotationalVelocity[0], ss_maxRotationalVelocity[1], ss_maxRotationalVelocity[2] });

    }

    // linMotion.setMaxTranslationAcceleration(value);
    // linMotion.setMaxNullSpaceAcceleration(value);
    // linMotion.setMaxNullSpaceVelocity(value);
    // linMotion.setMaxOrientationAcceleration(value);
  }
}