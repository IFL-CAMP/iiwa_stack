/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de Technische Universität München Chair for Robotics,
 * Artificial Intelligence and Embedded Systems Fakultät für Informatik / I6, Boltzmannstraße 3, 85748
 * Garching bei München, Germany http://www6.in.tum.de All rights reserved.
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
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

public class PTPCheckForGoalReachedThread implements Runnable {

  protected iiwaPublisher publisher;
  protected iiwaActionServer actionServer;
  protected LBR robot;
  protected int rate = 1000 / 30; // 30hz
  protected IMotionContainer motionContainer;

  public PTPCheckForGoalReachedThread(IMotionContainer motion, LBR robot, iiwaPublisher publisher,
      iiwaActionServer actionServer) {
    this.motionContainer = motion;
    this.robot = robot;
    this.publisher = publisher;
    this.actionServer = actionServer;
  }

  @Override
  public void run() {
    while (!((SunriseExecutionService) robot.getController().getExecutionService()).isPausedAndStopped()
        || !robot.isReadyToMove()) {
      // while (robot.hasActiveMotionCommand() ||
      // ((SunriseExecutionService)robot.getController().getExecutionService()).isPaused())
      // {
      if (motionContainer.isFinished()) {
        Logger.info("Motion finished");

        if (publisher != null) {
          publisher.publishDestinationReached();
        }
        if (actionServer != null && actionServer.hasCurrentGoal()) {
          actionServer.markCurrentGoalReached();
        }
        return;
      }
      else if (motionContainer.hasError()) {
        Logger.error("Motion failed: " + motionContainer.getErrorMessage());

        if (actionServer != null && actionServer.hasCurrentGoal()) {
          actionServer.markCurrentGoalFailed(motionContainer.getErrorMessage());
        }
        return;
      }

      ThreadUtil.milliSleep(rate);
    }

    Logger.info("Execution paused and stopped. Ending PTPcheckForGoalReachedThread.");
  }
}
