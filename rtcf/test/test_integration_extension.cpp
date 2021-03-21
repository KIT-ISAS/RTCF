#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RTCFTestExtension, Parameters) {
    // this test uses an OROCOS component with the RTCF extensions header
    // to get and set some parameters on the ROS parameter server
    ros::NodeHandle nh;
    EXPECT_TRUE(nh.hasParam("/rtcf/dummy/parameter_in"));

    std::string value = "";
    EXPECT_TRUE(nh.getParam("/rtcf/dummy/parameter_out", value));
    EXPECT_EQ(value, "abcdef");
    value = "";

    EXPECT_TRUE(nh.getParam("/rtcf/parameter_out", value));
    EXPECT_EQ(value, "abcdef");
    value = "";

    EXPECT_TRUE(nh.getParam("/absolute/parameter_out", value));
    EXPECT_EQ(value, "abcdef");
    value = "";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_integration_extension");
    ros::Time::init();
    ros::Duration(5.0).sleep();
    return RUN_ALL_TESTS();
}
