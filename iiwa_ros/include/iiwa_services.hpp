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

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>

namespace iiwa_ros {
  
  template<typename T>
  /**
   * @brief This class provides a templated base class for service wrappers.
   */
  class iiwaServices {
  public:
    
    /**
     * @brief Creates a ROS Service client for the Service server with the given name.
     */
    iiwaServices() : service_name_(""), verbose_(true), service_ready_(false) {}
    
    /**
     * @brief Creates a ROS Service client for the Service server with the given name.
     * 
     * @param service_name Name of the ROS Service server to connect to.
     * @param verbose If true some ROS_INFO messages will be printed out during service calls.
     */
    iiwaServices(const std::string& service_name, const bool verbose = true) : service_name_(service_name), verbose_(verbose), service_ready_(false) {
      initService();
    }
    
    /**
     * @brief Initializes a ROS Service client for the Service server with the name set using setServiceName.
     * A client is created by the class constructor, this function might be useful to switch to another Service server with another name on the fly.
     * To do that, first set the new server name with setServiceName.
     */
    virtual void initService() { 		
      ros::NodeHandle nh_;
      client_ = nh_.serviceClient<T>(service_name_); service_ready_ = true;
    };
    
    /**
     * @brief Sets the name of the ROS Service serve to connect to. Use initService to create a Service client to a service with this name.
     */
    virtual void setServiceName(const std::string& service_name) { service_name_ = service_name; initService();};
    
    /**
     * @brief Sets the verbosity level.
     * If true some ROS_INFO messages will be printed out during service calls.
     */
    virtual void setVerbosity(const bool verbose) { verbose_ = verbose; }
    
    /**
     * @brief Returns the error string obtained from the last service call that produced an error.
     * Available only if the implementation of the service call produces a string error in case of failure.
     * 
     * @return std::string 
     */
    virtual std::string getLastError() { return service_error_; }
    
  protected:
    /**
     * @brief Implements the actuall call to the service
     * 
     * @return bool
     */
    virtual bool callService() = 0;
    
    std::string service_name_ = "";
    ros::ServiceClient client_;
    T config_;
    bool verbose_ = true;
    std::string service_error_;
    bool service_ready_;
  };
  
}
