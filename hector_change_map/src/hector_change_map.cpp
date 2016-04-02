#include <ros/ros.h>
#include <hector_change_map/hector_change_map.h>

namespace hector_change_map{
HectorChangeMap::HectorChangeMap(){
    ROS_INFO ("change map node started");
    ros::NodeHandle nh("");

    map_pub_=  nh.advertise<nav_msgs::OccupancyGrid>("/map", 100, true);
    stairs_information_reset_pub_=  nh.advertise<std_msgs::Bool>("/hector_change_map/reset_stairs_information", 100, true);
    stairs_information_pub_=  nh.advertise<hector_stair_detection_msgs::BorderAndOrientationOfStairs>("/hector_stair_detection/border_and_orientation_of_stairs", 100, true);
    stairs_information_pub_debug_=  nh.advertise<visualization_msgs::MarkerArray>("/hector_change_map/stairs_border_debug", 100, true);

    change_layer_sub_ = nh.subscribe<hector_change_layer_msgs::Change_layer_msg>("/hector_change_map/change_layer", 1, &HectorChangeMap::ChangeLayerCB, this);

    nh.param("map_0_file", map_0_file_, std::string(""));
    nh.param("map_1_file", map_1_file_, std::string(""));
    nh.param("map_2_file", map_2_file_, std::string(""));
    nh.param("stairs_info_file", stairs_info_file_, std::string(""));
    nh.param("frame_id", frame_id_, std::string("/world"));
    nh.param("current_robot_layer", current_robot_layer_, 0);

    //load maps
    loadMap(map_0_file_);
    loadMap(map_1_file_);
    loadMap(map_2_file_);

    //load stairs information
    loadStairsInfo(stairs_info_file_);

    //provide initial map
    map_pub_.publish(all_layer_information_.at(0).map);

}

HectorChangeMap::~HectorChangeMap()
{}

void HectorChangeMap::ChangeLayerCB(const hector_change_layer_msgs::Change_layer_msg layer_msg){
    current_robot_layer_=layer_msg.layerNumber;
    publishMapForLayer();
}

void HectorChangeMap::publishMapForLayer(){
    //stair traversal layer are always numberd uneven
    if(!all_layer_information_.empty()){
        //publish map for layer
        ROS_DEBUG("provide map for layer, %i", current_robot_layer_);
        map_pub_.publish(all_layer_information_.at(current_robot_layer_).map);

        if(!all_layer_information_.at(current_robot_layer_).staircases.size()==0){
            //publish stairs information for layer
            ROS_DEBUG("publish staircase information for layer: %i", current_robot_layer_);
            publishStairsInformationForLayer(current_robot_layer_);
        }
    }
}

void HectorChangeMap::publishStairsInformationForLayer(int layer){
    layer_information layer_info= all_layer_information_.at(layer);
    for(int i=0; i<layer_info.staircases.size(); i++){
        visualization_msgs::MarkerArray stairs_boarder_marker;

        visualization_msgs::Marker marker0;
        marker0.header.frame_id = "/world";
        marker0.type = visualization_msgs::Marker::SPHERE;
        marker0.action = visualization_msgs::Marker::ADD;
        marker0.ns = "change_layer";
        marker0.id = 0;
        marker0.pose.position.x = layer_info.staircases.at(i).bottom1(0);
        marker0.pose.position.y = layer_info.staircases.at(i).bottom1(1);
        marker0.pose.position.z = layer_info.staircases.at(i).bottom1(2);
        marker0.scale.x = 0.2;
        marker0.scale.y = 0.2;
        marker0.scale.z = 0.2;
        marker0.color.a = 1.0;
        marker0.color.r = 0.0;
        marker0.color.g = 1.0;
        marker0.color.b = 0.0;
        stairs_boarder_marker.markers.push_back(marker0);

        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "/world";
        marker1.type = visualization_msgs::Marker::SPHERE;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.ns = "change_layer";
        marker1.id = 1;
        marker1.pose.position.x = layer_info.staircases.at(i).bottom2(0);
        marker1.pose.position.y = layer_info.staircases.at(i).bottom2(1);
        marker1.pose.position.z = layer_info.staircases.at(i).bottom2(2);
        marker1.scale.x = 0.2;
        marker1.scale.y = 0.2;
        marker1.scale.z = 0.2;
        marker1.color.a = 1.0;
        marker1.color.r = 0.0;
        marker1.color.g = 1.0;
        marker1.color.b = 0.0;
        stairs_boarder_marker.markers.push_back(marker1);

        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "/world";
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.ns = "change_layer";
        marker2.id = 2;
        marker2.pose.position.x = layer_info.staircases.at(i).top1(0);
        marker2.pose.position.y = layer_info.staircases.at(i).top1(1);
        marker2.pose.position.z = layer_info.staircases.at(i).top1(2);
        marker2.scale.x = 0.2;
        marker2.scale.y = 0.2;
        marker2.scale.z = 0.2;
        marker2.color.a = 1.0;
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;
        marker2.color.b = 0.0;
        stairs_boarder_marker.markers.push_back(marker2);

        visualization_msgs::Marker marker3;
        marker3.header.frame_id = "/world";
        marker3.type = visualization_msgs::Marker::SPHERE;
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.ns = "change_layer";
        marker3.id = 3;
        marker3.pose.position.x = layer_info.staircases.at(i).top2(0);
        marker3.pose.position.y = layer_info.staircases.at(i).top2(1);
        marker3.pose.position.z = layer_info.staircases.at(i).top2(2);
        marker3.scale.x = 0.2;
        marker3.scale.y = 0.2;
        marker3.scale.z = 0.2;
        marker3.color.a = 1.0;
        marker3.color.r = 0.0;
        marker3.color.g = 1.0;
        marker3.color.b = 0.0;
        stairs_boarder_marker.markers.push_back(marker3);

        hector_stair_detection_msgs::BorderAndOrientationOfStairs border_and_orientation_msg;
        border_and_orientation_msg.header.frame_id=layer_info.staircases.at(i).orientation.header.frame_id;
        border_and_orientation_msg.border_of_stairs= stairs_boarder_marker;
        border_and_orientation_msg.orientation_of_stairs=layer_info.staircases.at(i).orientation;
        border_and_orientation_msg.number_of_points= layer_info.staircases.at(i).number_of_points;
        border_and_orientation_msg.directionX=layer_info.staircases.at(i).direction(0);
        border_and_orientation_msg.directionY=layer_info.staircases.at(i).direction(1);
        border_and_orientation_msg.directionZ=layer_info.staircases.at(i).direction(2);
        //reset old stairs information
        border_and_orientation_msg.clearStairsInformation=true;

        if(stairs_information_pub_debug_.getNumSubscribers()>0){
            stairs_information_pub_debug_.publish(stairs_boarder_marker);
        }
        if(stairs_information_pub_.getNumSubscribers()>0){
            stairs_information_pub_.publish(border_and_orientation_msg);
        }
    }
}


void HectorChangeMap::insertStairs(Eigen::Vector3f bottom1, Eigen::Vector3f bottom2, Eigen::Vector3f top1, Eigen::Vector3f top2, Eigen::Vector3f direction, float yaw, float pitch, float number_of_points, geometry_msgs::PoseStamped orientation, int layer){
    ROS_INFO("Insert new Staircase at Layer %i", layer);
    staircase newStaircase;
    newStaircase.bottom1=bottom1;
    newStaircase.bottom2=bottom2;
    newStaircase.top1=top1;
    newStaircase.top2=top2;
    newStaircase.direction=direction;
    newStaircase.yaw=yaw;
    newStaircase.pitch=pitch;
    newStaircase.number_of_points=number_of_points;
    newStaircase.orientation=orientation;

    if(all_layer_information_.size() > layer){
        //layer is already existing
        all_layer_information_.at(layer).staircases.push_back(newStaircase);
    }else{
        //add new layer
        layer_information layer_info;
        while(all_layer_information_.size() < layer){
            all_layer_information_.push_back(layer_info);
        }
        layer_info.staircases.push_back(newStaircase);
        all_layer_information_.push_back(layer_info);
    }

}

void HectorChangeMap::loadMap(std::string file_to_load){
    double res;
    std::string mapfname = "";
    double origin[3];
    int negate;
    double occ_th, free_th;

    YAML::Node file = YAML::LoadFile(file_to_load.c_str());

    res= file["resolution"].as<double>();
    mapfname= file["image"].as<std::string>();
    negate=file["negate"].as<int>();
    occ_th=file["occupied_thresh"].as<double>();
    free_th=file["free_thresh"].as<double>();

    origin[0]=file["origin"][0].as<double>();
    origin[1]=file["origin"][1].as<double>();
    origin[2]=file["origin"][2].as<double>();
    origin[3]=file["origin"][3].as<double>();

    mapfname=file["image"].as<std::string>();
    if(mapfname[0] != '/')
    {
        char* fname_copy = strdup(file_to_load.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
    }

    ROS_INFO("[hector_change_map] file loaded");

    layer_information layer_info;
    nav_msgs::GetMap::Response map_resp_;
    double origin_for_mapserver[]= {origin[0], origin[1], origin[3]};
    map_server::loadMapFromFile(&map_resp_, mapfname.c_str(),res,negate,occ_th,free_th, origin_for_mapserver);
    layer_info.map=map_resp_.map;
    layer_info.map.info.origin.position.x= origin[0];
    layer_info.map.info.origin.position.y= origin[1];
    layer_info.map.info.origin.position.z= origin[2];
    layer_info.map.info.map_load_time = ros::Time::now();
    layer_info.map.header.frame_id = frame_id_;
    layer_info.map.header.stamp = ros::Time::now();
    all_layer_information_.push_back(layer_info);
}

void HectorChangeMap::loadStairsInfo(std::string file_to_load){
    std::string frame = "";
    float layer;
    Eigen::Vector3f bottom1;
    Eigen::Vector3f bottom2;
    Eigen::Vector3f top1;
    Eigen::Vector3f top2;
    Eigen::Vector3f direction;
    Eigen::Vector3f position;
    float yaw;
    float pitch;
    float number_of_points;
    geometry_msgs::PoseStamped orientation;

    YAML::Node file = YAML::LoadFile(file_to_load.c_str());

    frame= file["frame"].as<std::string>();

    layer= file["layer"].as<float>();

    bottom1(0)=file["bottom1"][0].as<float>();
    bottom1(1)=file["bottom1"][1].as<float>();
    bottom1(2)=file["bottom1"][2].as<float>();

    bottom2(0)=file["bottom2"][0].as<float>();
    bottom2(1)=file["bottom2"][1].as<float>();
    bottom2(2)=file["bottom2"][2].as<float>();

    top1(0)=file["top1"][0].as<float>();
    top1(1)=file["top1"][1].as<float>();
    top1(2)=file["top1"][2].as<float>();

    top2(0)=file["top2"][0].as<float>();
    top2(1)=file["top2"][1].as<float>();
    top2(2)=file["top2"][2].as<float>();

    position(0)=file["position"][0].as<float>();
    position(1)=file["position"][1].as<float>();
    position(2)=file["position"][2].as<float>();

    direction(0)=file["direction"][0].as<float>();
    direction(1)=file["direction"][1].as<float>();
    direction(2)=file["direction"][2].as<float>();

    yaw= file["yaw"].as<double>();
    pitch= file["pitch"].as<double>();
    number_of_points= file["number_of_points"].as<double>();

    ROS_INFO("[hector_change_map] stairs info file loaded");

    orientation.header.frame_id=frame;
    orientation.pose.position.x=position(0);
    orientation.pose.position.y=position(1);
    orientation.pose.position.z=position(2);
    tf::Quaternion temp;
    temp.setEulerZYX(yaw,pitch,0.0);
    orientation.pose.orientation.x=temp.getX();
    orientation.pose.orientation.y=temp.getY();
    orientation.pose.orientation.z=temp.getZ();
    orientation.pose.orientation.w=temp.getW();

    insertStairs(bottom1, bottom2, top1, top2, direction, yaw, pitch, number_of_points, orientation, layer);
}

}
