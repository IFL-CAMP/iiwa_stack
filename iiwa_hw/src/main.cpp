/**
 * This code is a porting of the work from the Centro E. Piaggio in Pisa : https://github.com/CentroEPiaggio/kuka-lwr
 * for the LBR IIWA. We acknowledge the good work of their main contributors :
 * Carlos J. Rosales - cjrosales@gmail.com
 * Enrico Corvaglia
 * Marco Esposito - marco.esposito@tum.de
 * Manuel Bonilla - josemanuelbonilla@gmail.com
 *
 * LICENSE :
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include "iiwa_hw.h"

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

int main(int argc, char** argv)
{
  // initialize ROS
  ros::init(argc, argv, "iiwa_hw", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // construct the lbr iiwa
  ros::NodeHandle iiwa_nh;
  IIWA_HW iiwa_robot(iiwa_nh);

  // configuration routines
  iiwa_robot.start();

  ros::Time last(ros::Time::now());
  ros::Time now;
  ros::Duration period(1.0);

  // the controller manager
  controller_manager::ControllerManager manager(&iiwa_robot, iiwa_nh);

  // run as fast as possible
  while (!g_quit)
  {
    // get the time / period
    now = ros::Time::now();
    period = now - last;
    last = now;

    // read current robot position
    iiwa_robot.read(period);

    // update the controllers
    manager.update(now, period);

    // send command position to the robot
    iiwa_robot.write(period);

    // wait for some milliseconds defined in controlFrequency
    iiwa_robot.getRate()->sleep();
  }

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
