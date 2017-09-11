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

#include <iiwa_msgs/SetPathParameters.h>
#include <iiwa_ros/iiwa_services.hpp>

namespace iiwa_ros {
  
  class PathParametersService : public iiwaServices<iiwa_msgs::SetPathParameters> {
  public:
    
    PathParametersService();
    
    /**
     * @brief ...
     * 
     * @param service_name ...
     * @param verbose ...
     */
    PathParametersService(const std::string& service_name, const bool verbose = true);
    /**
     * @brief ...
     * 
     * @param joint_relative_velocity ...
     * @return bool
     */
    bool setJointRelativeVelocity(const double joint_relative_velocity);
    
    /**
     * @brief ...
     * 
     * @param joint_relative_acceleration ...
     * @return bool
     */
    bool setJointRelativeAcceleration(const double joint_relative_acceleration);
    
    /**
     * @brief ...
     * 
     * @param override_joint_acceleration ...
     * @return bool
     */
    bool setOverrideJointAcceleration(const double override_joint_acceleration);
    
    /**
     * @brief ...
     * 
     * @param joint_relative_velocity ...
     * @param joint_relative_acceleration ...
     * @return bool
     */
    bool setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration);
    
    /**
     * @brief ...
     * 
     * @param joint_relative_velocity ...
     * @param joint_relative_acceleration ...
     * @param override_joint_acceleration ...
     * @return bool
     */
    bool setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration, const double override_joint_acceleration);
    
  protected:
    virtual bool callService();
  };
  
}