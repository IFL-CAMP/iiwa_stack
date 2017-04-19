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

#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_ros/iiwa_services.hpp>

namespace iiwa_ros {
  
  /**
   * @brief This class provides a wrapper for service call of the ConfigureSmartServo service.
   * Once an object of this class is created using the appropriate service name (e.g. /iiwa/configuration/configureSmartServo), 
   * it is possible to just call its functions to set the desired control mode on the robot.
   */
  class SmartServoService : public iiwaServices<iiwa_msgs::ConfigureSmartServo> {
  public:
    
    
    SmartServoService();
    
    /**
     * @brief Creates a Service client given the name of the configureSmartServo service: e.g. "/iiwa/configuration/configureSmartServo"
     * 
     * @param service_name Name of the ROS Service server to connect to.
     * @param verbose If true some ROS_INFO messages will be printed out during service calls.
     */
    SmartServoService(const std::string& service_name, const bool verbose = true);
    
    /**
     * @brief Sets the control mode to PositionControl.
     * 
     * @return bool - false is service call fails.
     */
    bool setPositionControlMode();
    
    /**
     * @brief Sets the control mode to JointImpedance.
     * 
     * @param joint_stiffnes Desired stiffness of the single joints. Single values must be: 0 <= value <= 5000.
     * @param joint_damping Desired damping of the single joints. Single values must be: 0 <= value <= 1.
     * @return bool
     */
    bool setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping);
    
    /**
     * @brief Sets the control mode to JointImpedance.
     * 
     * @param joint_stiffnes Desired joint stiffness, this value will be applied to all the joints. 0 <= value <= 5000.
     * @param joint_damping Desired joint damping, this value will be applied to all the joints. 0 <= value <= 1.
     * @return bool
     */
    bool setJointImpedanceMode(const double joint_stiffnes, const double joint_damping);
    
    /**
     * @brief Sets the control mode to CartesianImpedance
     * 
     * Unset values like nullspace stiffness, nullspace damping and the cartesian limits are all set to their default values.
     * 
     * @param cartesian_stiffness Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
     * @param cartesian_damping Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c]. Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
     * @return bool
     */
    bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping);
    
    /**
     * @brief Sets the control mode to CartesianImpedance. With Cartesian limits.
     * 
     * Unset values like nullspace stiffness, nullspace damping are all set to their default values.
     * 
     * @param cartesian_stiffness Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
     * @param cartesian_damping Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c]. Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
     * @param max_path_deviation Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. [x, y, z] in [mm]. Precondition: value > 0.0. [a, b, c] in [rad]. Precondition: value > 0.0.
     * @param max_cartesian_velocity Maximum Cartesian velocity parameter. [x, y, z] in [mm/s]. Precondition: value > 0.0. [a, b, c] in [rad/s]. Precondition: value > 0.0.
     * @param max_control_force The maximum control force parameter. [x, y, z] in [N]. Precondition: value > 0.0. [a, b, c] in [Nm]. Precondition: value > 0.0.
     * @param max_control_force_stop Indicates whether a stop condition is fired if the maximum control force is exceeded.
     * @return bool
     */
    bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping,
                                   const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                                   const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
    /**
     * @brief  Sets the control mode to CartesianImpedance.
     * Cartesian limits are all set to their default values.
     * 
     * @param cartesian_stiffness Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
     * @param cartesian_damping Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c]. Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
     * @param nullspace_stiffness The stiffness value for null space [Nm/rad]. Precondition: 0.0 <= value.
     * @param nullspace_damping he damping parameter for null space [Nm*s/rad]. Precondition: value >= 0.3 and value <= 1.0. - A good damping value is 0.7.
     * @return bool
     */
    bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                   const double nullspace_stiffness, const double nullspace_damping);
    
    /**
     * @brief Sets the control mode to CartesianImpedance. With Cartesian limits.
     * 
     * @param cartesian_stiffness Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
     * @param cartesian_damping Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c]. Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
     * @param nullspace_stiffness The stiffness value for null space [Nm/rad]. Precondition: 0.0 <= value.
     * @param nullspace_damping he damping parameter for null space [Nm*s/rad]. Precondition: value >= 0.3 and value <= 1.0. - A good damping value is 0.7.
     * @param max_path_deviation Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. [x, y, z] in [mm]. Precondition: value > 0.0. [a, b, c] in [rad]. Precondition: value > 0.0.
     * @param max_cartesian_velocity Maximum Cartesian velocity parameter. [x, y, z] in [mm/s]. Precondition: value > 0.0. [a, b, c] in [rad/s]. Precondition: value > 0.0.
     * @param max_control_force The maximum control force parameter. [x, y, z] in [N]. Precondition: value > 0.0. [a, b, c] in [Nm]. Precondition: value > 0.0.
     * @param max_control_force_stop Indicates whether a stop condition is fired if the maximum control force is exceeded.
     * @return bool
     */
    
    bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                   const double nullspace_stiffness, const double nullspace_damping,
                                   const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                                   const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
    /**
     * @brief Sets the control mode to DesiredForce.
     * A constant force will be applied along the given cartesian direction.
     * 
     * @param cartesian_dof The degree of freedom on which the desired force is applied: 1 = X, 2 = Y, 3 = Z. Use iiwa_msgs.DOF messages!
     * @param desired_force The value of the desired force. In [N].
     * @param desired_stiffness # The value of the stiffness. In [N/m]
     * @return bool
     */
    bool setDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness);
    
    /**                                                               
     * @brief Sets the control mode to DesiredForce. With Cartesian limits.
     * A constant force will be applied along the given cartesian direction.
     * 
     * @param cartesian_dof The degree of freedom on which the desired force is applied: 1 = X, 2 = Y, 3 = Z. Use iiwa_msgs.DOF messages!
     * @param desired_force The value of the desired force. In [N].
     * @param desired_stiffness # The value of the stiffness. In [N/m].
     * @param max_path_deviation Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. [x, y, z] in [mm]. Precondition: value > 0.0. [a, b, c] in [rad]. Precondition: value > 0.0.
     * @param max_cartesian_velocity Maximum Cartesian velocity parameter. [x, y, z] in [mm/s]. Precondition: value > 0.0. [a, b, c] in [rad/s]. Precondition: value > 0.0.
     * @param max_control_force The maximum control force parameter. [x, y, z] in [N]. Precondition: value > 0.0. [a, b, c] in [Nm]. Precondition: value > 0.0.
     * @param max_control_force_stop Indicates whether a stop condition is fired if the maximum control force is exceeded.
     * @return bool
     */
    bool setDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness, 
                             const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                             const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
    /**
     * @brief Sets the control mode to SinePattern. With Cartesian limits.
     * 
     * @param cartesian_dof The degree of freedom for performing the sine oscillation. 1 = X, 2 = Y, 3 = Z. Use iiwa_msgs.DOF messages!
     * @param frequency The value of the frequency for the sine oscillation [Hz].
     * @param amplitude The value of the amplitude. In [N].
     * @param stiffness The value of the stiffness. In [N/m].
     * @return bool
     */
    bool setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness);
    
    /**
     * @brief Sets the control mode to SinePattern. With Cartesian limits.
     * 
     * @param cartesian_dof The degree of freedom for performing the sine oscillation. 1 = X, 2 = Y, 3 = Z. Use iiwa_msgs.DOF messages!
     * @param frequency The value of the frequency for the sine oscillation [Hz].
     * @param amplitude The value of the amplitude. In [N].
     * @param stiffness The value of the stiffness. In [N/m].
     * @param max_path_deviation Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs. [x, y, z] in [mm]. Precondition: value > 0.0. [a, b, c] in [rad]. Precondition: value > 0.0.
     * @param max_cartesian_velocity Maximum Cartesian velocity parameter. [x, y, z] in [mm/s]. Precondition: value > 0.0. [a, b, c] in [rad/s]. Precondition: value > 0.0.
     * @param max_control_force The maximum control force parameter. [x, y, z] in [N]. Precondition: value > 0.0. [a, b, c] in [Nm]. Precondition: value > 0.0.
     * @param max_control_force_stop Indicates whether a stop condition is fired if the maximum control force is exceeded.
     * @return bool
     */
    bool setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness,
                            const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                            const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
  protected:
    virtual bool callService();
    
    void initCartesianLimits(const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                             const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
    void initJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping);
    void initCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                    const double nullspace_stiffness, const double nullspace_damping,
                                    const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                                    const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    void initDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness, 
                              const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                              const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
    void initSinePatternMode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness,
                             const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                             const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop);
    
  };
}