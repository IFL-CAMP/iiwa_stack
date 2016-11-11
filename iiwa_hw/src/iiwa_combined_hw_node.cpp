/** @file iiwa_combined_hw_node.cpp
 *  @brief A simple ROS node implementing a ros_control loop for the Kuka iiwa
 * robot arm compatiable with combined_robot_hw.
 *  @author Murilo F. Martins (murilo.martins@ocado.com)
 *
 * Copyright (c) 2016, Ocado Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Ocado Technology nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "iiwa_hw.h"

#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

constexpr int CTRL_FREQ = 1000; // Hz

bool g_quit = false;

void quitRequested(int sig) {
    g_quit = true;
}

int main( int argc, char** argv ) {
    // initialize ROS
    ros::init(argc, argv, "iiwa_combined_hw", ros::init_options::NoSigintHandler);

    ros::Rate loop_rate(CTRL_FREQ);

    // ros spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // custom signal handlers
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    // construct the lbr iiwa
    ros::NodeHandle nh;
    combined_robot_hw::CombinedRobotHW robot_hw;

    // configuration routines
    robot_hw.init(nh, nh);

    // timer variables
    struct timespec ts = {0, 0};
    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period(1.0);

    //the controller manager
    controller_manager::ControllerManager manager(&robot_hw, nh);


    // run as fast as possible
    while( !g_quit ) {
        // get the time / period
        if (!clock_gettime(CLOCK_REALTIME, &ts)) {
            now.sec = ts.tv_sec;
            now.nsec = ts.tv_nsec;
            period = now - last;
            last = now;
        } else {
            ROS_FATAL("Failed to poll realtime clock!");
            break;
        }

        // read current robot position
        robot_hw.read(now, period);

        // update the controllers
        manager.update(now, period);

        // send command position to the robot
        robot_hw.write(now, period);

        // wait for some milliseconds defined in controlFrequency
        loop_rate.sleep();

    }

    std::cerr << "Stopping spinner..." << std::endl;
    spinner.stop();

    std::cerr << "Terminating..." << std::endl;

    return 0;
}
