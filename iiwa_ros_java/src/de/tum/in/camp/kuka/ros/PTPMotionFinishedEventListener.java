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

import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;

public class PTPMotionFinishedEventListener implements IMotionContainerListener {

  protected iiwaPublisher publisher;
  protected iiwaActionServer actionServer;

  public PTPMotionFinishedEventListener(iiwaPublisher publisher, iiwaActionServer actionServer) {
    this.publisher = publisher;
    this.actionServer = actionServer;
  }

  @Override
  public void onStateChanged(IExecutionContainer container, ExecutionState state) {
    // not used
  }

  @Override
  public void containerFinished(IMotionContainer container) {
    Logger.debug("Motion finished");
    if (publisher != null) {
      publisher.publishDestinationReached();
    }
    if (actionServer != null && actionServer.hasCurrentGoal()) {
      actionServer.markCurrentGoalReached();
    }
  }

  @Override
  public void motionFinished(IMotion motion) {
    // not used
  }

  @Override
  public void motionStarted(IMotion motion) {
    // not used
  }
}
