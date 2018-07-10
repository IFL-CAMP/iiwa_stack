/**  
 * Copyright (C) 2018 Guido Breitenhuber - guido.breitenhuber@joanneum.at, Thomas Haspl - thomas.haspl@joanneum.at
 * JOANNEUM RESEARCH Forschungsgesellschaft mbH
 * ROBOTICS – Institute for Robotics and Mechatronics
 * Lakeside B08a, 9020 Klagenfurt am Wörthersee, Austria
 * http://www.joanneum.at/robotics
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

package de.tum.in.camp.kuka.ros.configuration;


public interface IConfigurationProvider {
	/**
	 * Get the robot name.
	 * 
	 * @return name of the robot
	 */
	public String getRobotName();
	
	/**
	 * Get the robot IP address.
	 * 
	 * @return Robot IP address
	 */
	public String getRobotIP();
	
	/**
	 * Get the ROS Master IP address.
	 * 
	 * @return ROS Master IP address
	 */
	public String getRosMasterIP();
	
	/**
	 * Get the ROS Master port number.
	 * 
	 * @return ROS Master port number
	 */
	public int getRosMasterPort();
	
	/**
	 * Get the ROS Master URI.
	 * 
	 * @return ROS Master URI (http://IP:port)
	 */
	public String getRosMasterUri();
	
	/**
	 * Return if an external NTP server should be used.
	 * 
	 * @return true if external NTP server should be used, false otherwise
	 */
	public boolean getNtpWithHost();
}
