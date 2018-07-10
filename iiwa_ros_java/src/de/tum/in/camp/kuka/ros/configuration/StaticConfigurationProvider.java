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


public class StaticConfigurationProvider implements IConfigurationProvider {

	private String robotName;
	private String robotIp;
	private boolean ntpWithHost;
	private String masterIp;
	private int masterPort;
	private String masterUri;

	public StaticConfigurationProvider(String robotName, String robotIp, String masterIp, int masterPort, boolean ntpWithHost)
	{
		throwIfStringNullOrEmpty(robotName, "Robot name");
		throwIfStringNullOrEmpty(robotIp, "Robot IP");
		throwIfStringNullOrEmpty(masterIp, "ROS master IP");
				
		this.robotName = robotName;
		this.robotIp = robotIp;
		this.ntpWithHost = ntpWithHost;
		
		this.masterIp = masterIp;
		this.masterPort = masterPort;
		this.masterUri = "http://" + masterIp + ":" + masterPort;
	}
	
	public static void throwIfStringNullOrEmpty(final String s, final String paramName) {
		if (s == null || s.trim().isEmpty()) {
			throw new IllegalArgumentException(paramName + " must not be null or empty");
		}
	}
	
	@Override
	public String getRobotName() {
		return robotName;
	}

	@Override
	public String getRobotIP() {
		return robotIp;
	}

	@Override
	public String getRosMasterIP() {
		return masterIp;
	}

	@Override
	public int getRosMasterPort() {
		return masterPort;
	}

	@Override
	public boolean getNtpWithHost() {
		return ntpWithHost;
	}

	@Override
	public String getRosMasterUri() {
		return masterUri;
	}

}
