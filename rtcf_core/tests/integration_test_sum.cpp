#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include <std_msgs/Float64.h>

class RTCFTest : public testing::Test {
   protected:
    virtual void SetUp() {
        node_handle_ = ros::NodeHandle("~");
        publisher_ =
            node_handle_.advertise<std_msgs::Float64>("/mapped/in_from_ros", 1, true);
        subscriber_ = node_handle_.subscribe("/mapped/to_ros", 1,
                                             &RTCFTest::handleMsg, this);
    }

    void handleMsg(const std_msgs::Float64 msg) {
        EXPECT_EQ(expected_value_, msg.data);
        finished = true;
    }

    bool finished = false;
    ros::Duration timeout = ros::Duration(5.0);

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    float expected_value_ = 0;

   public:
    void sendMsg(float value) {
        std_msgs::Float64 ros_msg;
        ros_msg.data = value;

        publisher_.publish(ros_msg);
    }

    void setExpectedValue(float value) { expected_value_ = value; }
};

TEST_F(RTCFTest, testIntegrationSum1) {
    const ros::Time start = ros::Time::now();
    setExpectedValue(10.0);
    sendMsg(5.0);
    do {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    } while (!finished && (ros::Time::now() - start) < timeout);
    EXPECT_TRUE(finished);
}

TEST_F(RTCFTest, testIntegrationSum2) {
    const ros::Time start = ros::Time::now();
    setExpectedValue(20.0);
    sendMsg(10.0);
    do {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    } while (!finished && (ros::Time::now() - start) < timeout);
    EXPECT_TRUE(finished);
}

TEST_F(RTCFTest, testIntegrationSum3) {
    const ros::Time start = ros::Time::now();
    setExpectedValue(-20.0);
    sendMsg(-10.0);
    do {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    } while (!finished && (ros::Time::now() - start) < timeout);
    EXPECT_TRUE(finished);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "integration_tests_sum");
    return RUN_ALL_TESTS();
}

