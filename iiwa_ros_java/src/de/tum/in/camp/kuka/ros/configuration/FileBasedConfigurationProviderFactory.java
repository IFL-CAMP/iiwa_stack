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

package de.tum.in.camp.kuka.ros.configuration;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

import de.tum.in.camp.kuka.ros.Configuration;

public class FileBasedConfigurationProviderFactory {
	
	/**
	 * Parses the config.txt and creates a configuration provider.
	 * 
	 * @return Configuration provider filled with values from the config.txt
	 * @throws IOException
	 */
	public static IConfigurationProvider createFromDefaultConfigFile() throws IOException {
		
		// The config file is historically located on de/tum/in/camp/kuka/ros/config.txt
		return create(Configuration.class.getResourceAsStream("config.txt"));
	}
	
	/**
	 * Parses the content of the input stream and creates a configuration provider.
	 * 
	 * @param s
	 * @return Configuration provider filled with values from the given input stream
	 * @throws IOException
	 */
	public static IConfigurationProvider create(InputStream s) throws IOException {
		if (s == null) {
			throw new NullPointerException("s");
		}
		
		Map<String, String> config = read(s);
		
		// Obtain name of the robot from config file
		String robotName = config.get("robot_name");
		String robotIp = config.get("robot_ip");
		
		// Obtain if NTP server is used from config file
		boolean ntpWithHost  = config.get("ntp_with_host").equals("true");

		// Obtain IP:port of the ROS Master 
		String masterIp = config.get("master_ip");
		int masterPort = Integer.parseInt(config.get("master_port"));

		StaticConfigurationProvider configProvider = new StaticConfigurationProvider(robotName, robotIp, masterIp, masterPort, ntpWithHost);

		return configProvider;
	}
	
	/**
	 * Parses the key value pairs stored in the config.
	 * 
	 * @return Key-Value pairs
	 * @throws IOException if the stream could not be read
	 */
	private static HashMap<String, String> read(InputStream s) throws IOException {
		HashMap<String, String> config = new HashMap<String, String>();
		BufferedReader br = new BufferedReader(new InputStreamReader(s));
		
		String line = null;
		while((line = br.readLine()) != null) {
			String[] lineComponents = line.split(":");
			if (lineComponents.length != 2)
				continue;

			config.put(lineComponents[0].trim(), lineComponents[1].trim());
		}
		
		return config;
	}
}
