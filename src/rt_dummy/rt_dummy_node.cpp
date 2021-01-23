#include "rt_dummy_node.hpp"

RTDummyNode::RTDummyNode(const ros::NodeHandle &node_handle){

};

int RTDummyNode::loop() {
    ros::spin();
    shutdown();
    ros::spinOnce();

    return 0;
};

void RTDummyNode::configure(){

};

void RTDummyNode::shutdown(){
    shutdownROS();
};

    int loop();
    
    void setupROS();
void RTDummyNode::shutdownROS(){

};
    void loadROSParameters();

    void registerAtRTRunner();
    void unregisterAtRTRunner();

    void handleRemapping(int argc, char **argv);


int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "RTDummy");
    ros::NodeHandle nh("~");

    RTDummyNode node = RTDummyNode(nh);
    node.configure();

    return node.loop();

    return 0;
}  // end main()

