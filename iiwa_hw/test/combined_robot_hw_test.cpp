#include <time.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

TEST(IIWACombinedRobotHWTests, initialisation) {
    ros::NodeHandle n;

    combined_robot_hw::CombinedRobotHW iiwa_hw;

    bool init_ok = iiwa_hw.init(n, n);
    ASSERT_TRUE(init_ok);
}

TEST(IIWACombinedRobotHWTests, controlLoop1000steps) {
    rps::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle n;
    combined_robot_hw::CombinedRobotHW iiwa_robot;

    iiwa_robot.init(n, n);

    ros::Duration period(1.0);
    struct timespecs ts = {0, 0};
    ros::Time last(ts.tv_sec, ts.tv_nsec);
    ros::Time now(ts.tv_sec, ts.tv_nsec);

    controller_manager::ControllerManager manager(&iiwa_robot);

    for (unsigned int i = 0; i < 1000; ++i) {
        ASSERT_FALSE(clock_gettime(CLOCK_REALTIME, &ts));
        now.sec = ts.tv_sec;
        now.nsec = ts.tv_nsec;
        period = now - last;
        last = now;

        iiwa_robot.read(now, period);
        manager.update(now, period);
        iiwa_robot.write(now, period);

        iiwa_robot.getRate()->sleep();
    }
    spinner.stop();
}

int main (int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "iiwa_combined_robot_hw_test");

    return RUN_ALL_TESTS();
}
