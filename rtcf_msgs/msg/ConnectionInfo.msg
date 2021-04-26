# OROCOS connections are very similiar to ROS topics and hence have a name and a type
string name
string type
string[] publishing_components # internal OROCOS connections
string[] subscribing_components # internal OROCOS connections
bool connected_to_ROS # indicate whether the topic is bridged to ROS