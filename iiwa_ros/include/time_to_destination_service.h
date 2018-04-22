/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <iiwa_msgs/TimeToDestination.h>
#include <iiwa_services.hpp>

namespace iiwa_ros {
  
  /**
   * @brief This class provides a wrapper for service call of the TimeToDestination service.
   * Once an object of this class is created using the appropriate service name (e.g. /iiwa/state/timeToDestination), 
   * it is possible to just call its functions to get the time left - in <b>seconds</b> - to reach the last given destination.
   * If the robot already reached its last commanded destination, a negative number will be returned. That is the time - in <b>seconds</b> - since the robot reached that destination.
   * The value <b>-999</b> will be returned if an error occurs.
   */
  class TimeToDestinationService : public iiwaServices<iiwa_msgs::TimeToDestination> {
  public:
    
    TimeToDestinationService();
    
    /**
     * @brief Creates a Service client given the name of the timeToDestination service: e.g. "/iiwa/state/timeToDestination"
     * 
     * @param service_name Name of the ROS Service server to connect to.
     * @param verbose If true some ROS_INFO messages will be printed out during service calls.
     */
    TimeToDestinationService(const std::string& service_name, const bool verbose = true);
    
    /**
     * @brief Returns the time left to reach the last commanded destination in <b>seconds</b>.
     * A negative value represents the time since the robot reached its last commanded destination.
     * The value <b>-999</b> will be returned if an error occurs.
     * 
     * @return double
     */
    double getTimeToDestination();
  protected:
    virtual bool callService();
  private:
    double time_to_destination_;
  };
  
}
