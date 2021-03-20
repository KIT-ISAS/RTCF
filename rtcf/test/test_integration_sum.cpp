#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <optional>

class RTCFTest : public testing::Test {
  protected:
    virtual void SetUp() {
        node_handle_ = ros::NodeHandle("~");
        publisher_   = node_handle_.advertise<std_msgs::Float64>("/rtcf/ros/in", 1, true);
        subscriber_  = node_handle_.subscribe("/rtcf/ros/out", 1, &RTCFTest::handleMsg, this);
    }

  private:
    void handleMsg(const std_msgs::Float64 msg) { received_value_ = msg.data; }

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    std::optional<double> received_value_;

  public:
    void sendMsg(double value) {
        std_msgs::Float64 ros_msg;
        ros_msg.data = value;
        publisher_.publish(ros_msg);
    }

    std::optional<double> getReceivedValue() { return received_value_; }

    static constexpr double TIMEOUT      = 1.0;
    static constexpr double POLLING_TIME = 0.01;
};

TEST(RTCFTestStatic, AdvertisedTopics) {
    // NOTE: do not use test fixture for this test as we want to see all topics provided by the runner

    // get all topics available at master
    std::set<std::string> topic_names;
    // ros::master::getTopics() only gets us published topics, so we need to work around this

    // https://answers.ros.org/question/53267/get-all-topics-in-c/
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    if (!ros::master::execute("getTopicTypes", args, result, payload, true)) {
        FAIL();
        return;
    }
    for (int i = 0; i < payload.size(); ++i) {
        topic_names.insert(payload[i][0]);
    }

    // check if remappings work and that whitelisting and blacklisting works as expected
    EXPECT_TRUE(topic_names.find("/rtcf/ros/in") != topic_names.end());
    EXPECT_TRUE(topic_names.find("/rtcf/ros/out") != topic_names.end());
    // tmp is on blacklist
    EXPECT_FALSE(topic_names.find("/rtcf/ros/tmp") != topic_names.end());
}

TEST_F(RTCFTest, DataFlow) {
    // send something
    sendMsg(0.0);

    ros::Time start = ros::Time::now();
    bool received   = false;
    while (ros::ok() && ros::Time::now() - start < ros::Duration(TIMEOUT)) {
        ros::spinOnce();
        if (getReceivedValue()) {
            received = true;
            break;
        }
        ros::Duration(POLLING_TIME).sleep();
    }
    // check if something was received at all
    EXPECT_TRUE(received);
}

TEST_F(RTCFTest, ComponentOperation) {
    std::initializer_list<double> values{-10.0, 0.0, 20.0, 1234.56};

    // iterate over multiple values
    for (auto value : values) {
        sendMsg(value);
        ros::Time start       = ros::Time::now();
        bool correct_appeared = false;
        while (ros::ok()) {  //} && ros::Time::now() - start < ros::Duration(TIMEOUT)) {
            ros::spinOnce();
            auto rx_val = getReceivedValue();
            if (rx_val) {
                // the correct result must at least appear once
                if (*rx_val == value * 3) {
                    correct_appeared = true;
                    break;
                }
            }
            ros::Duration(POLLING_TIME).sleep();
        }
        EXPECT_TRUE(correct_appeared);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_integration_sum");
    return RUN_ALL_TESTS();
}
