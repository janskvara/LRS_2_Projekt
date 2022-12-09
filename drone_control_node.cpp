#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <unistd.h>
#include <iostream>
#include <string>

/*
z = 1.25

start
x = 13,6
y = 1,5

goal
x = 13,6
y = 11

v mojom podani su je x, y naopak
*/

const float HARD = 0.08;
const float HEIGHT_FLY = 1.25;
const float TARGET_POINT[2] = { 1.5 - 11, 13.6 - 13.6}; //x, y
const int SIZE_OF_TABLE = 800;
// y = 1 => robot ide do prava

float x[SIZE_OF_TABLE][SIZE_OF_TABLE];
float y[SIZE_OF_TABLE][SIZE_OF_TABLE];
float z[SIZE_OF_TABLE][SIZE_OF_TABLE];
bool start = true;

mavros_msgs::State current_state;
nav_msgs::Odometry current_position;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void position_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

void PcCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                 iter_y(*msg, "y"),
                                                 iter_z(*msg, "z");
    int col=0;
    int row=0;
    if(start){
        while (iter_x != iter_x.end()) {
                x[row][col]=*iter_x;
                y[row][col]=*iter_y;
                z[row][col]=*iter_z;

                col=col+1;
                if(col==800){
                        row=row+1;
                        col=0;
                }
                ++iter_x; ++iter_y; ++iter_z;
        }
        //start= true;
    }
}

void flyToXY(float x,float y,float z,ros::Publisher posePublisher);
void flyToZ(double z, ros::Publisher posePublisher);
void takeOff(double altitude,ros::ServiceClient takeOff_client, ros::ServiceClient arming_client, ros::ServiceClient set_mode_client, ros::Rate rate);  
void land(ros::ServiceClient land_client);   
double getDistanceFromXYTarget(double x, double y);
double getDistanceFromZTarget(double height);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Set up ROS publishers, subscribers and service clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // TODO: Add other required topics/services
    ros::Subscriber position_sub  = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, position_cb);
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::Subscriber p2_sub_ = nh.subscribe("/fei_lrs_drone/stereo_camera/points2", 1, PcCallback);

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    int stage = 0;
    bool sendCommand = false;
    int prelet = 0; // 1 -> do lava 2 -> po nad
    float sirka_prekazky = 0.0;
    float vyska_prekazky = 0.0;
    float vzdialenost_obstacle = 0.0;
    float actual_x = 0.0;
    float actual_y = 0.0;
    float actual_z = 0.0;

    takeOff(HEIGHT_FLY, takeoff_client, arming_client, set_mode_client, rate);
    while (ros::ok())
    {
        if(getDistanceFromXYTarget(TARGET_POINT[0], TARGET_POINT[1]) < HARD){
            ROS_INFO_STREAM("DOSIAHLI SME KONCOVI BOD");
            land(land_client);
            ROS_INFO_STREAM("KONCIME");
            break;
        }

        //Commands
        //priamy let
        if(stage == 2 && sendCommand && prelet == 0){
            flyToXY(TARGET_POINT[0], TARGET_POINT[1], HEIGHT_FLY, local_pos_pub);
            sendCommand = false;
        }

        //nad prekazkou
        if(sendCommand && prelet == 2){
            if(stage == 2){
                ROS_INFO_STREAM("1.Vzlietame nad prekazku");
                actual_z = HEIGHT_FLY + vyska_prekazky + 0.3;
                flyToZ(actual_z, local_pos_pub);
                sendCommand = false;
            }
            if(stage == 3){
                ROS_INFO_STREAM("2.Letime za prekazku");
                actual_x = current_position.pose.pose.position.x -(vzdialenost_obstacle + 1.2);
                actual_y = 0.0;
                flyToXY(actual_x, actual_y, actual_z,local_pos_pub);
                sendCommand = false;
            }
            if(stage == 4){
                ROS_INFO_STREAM("3.Vzlietame na defaultnu vysku");
                actual_z = HEIGHT_FLY;
                flyToZ(actual_z, local_pos_pub);
                sendCommand = false;
            }
        }

        //obyjst prekazku
        if(sendCommand && prelet == 1){
            if(stage == 2){
                ROS_INFO_STREAM("1.Ideme vedla prekazky");
                actual_x = current_position.pose.pose.position.x;
                actual_y = sirka_prekazky + 0.5;
                flyToXY(actual_x, actual_y, HEIGHT_FLY,local_pos_pub);
                sendCommand = false;
            }
            if(stage == 3){
                ROS_INFO_STREAM("2.Letime vedla za prekazku");
                actual_y = current_position.pose.pose.position.y;
                actual_x = current_position.pose.pose.position.x -(vzdialenost_obstacle + 1.2);
                flyToXY(actual_x, actual_y, HEIGHT_FLY,local_pos_pub);
                sendCommand = false;
            }
            if(stage == 4){
                ROS_INFO_STREAM("3.Ideme za prekazkou");
                actual_y = 0;
                actual_x = current_position.pose.pose.position.x;
                flyToXY(actual_x, actual_y, HEIGHT_FLY,local_pos_pub);
                sendCommand = false;
            }
        }


        //Query
        if(getDistanceFromZTarget(HEIGHT_FLY) < HARD && stage == 0 ){
            ROS_INFO_STREAM("SME NA DEFAULTNEJ VYSKE");
            stage = 1;
            sendCommand = true;
        }

        //Nad prekazkou
        if(prelet == 2 && sendCommand == false){
            if(getDistanceFromZTarget(actual_z) < HARD && stage == 2){
                ROS_INFO_STREAM("1.Sme nad prekazkou");
                stage = 3;
                //actual_z = 0.0;
                sendCommand = true;
                continue;
            }
            if(getDistanceFromXYTarget(actual_x, actual_y) < HARD && stage == 3){
                ROS_INFO_STREAM("2.Sme za prekazkou");
                stage = 4;
                actual_x = 0.0;
                actual_y = 0.0;
                sendCommand = true;
                continue;
            }
            if(getDistanceFromZTarget(actual_z) < HARD && stage == 4){
                ROS_INFO_STREAM("3.SME NA DEFAULTNEJ VYSKE");
                stage = 1;
                actual_z = 0.0;
                sendCommand = false;
            }
        }

        //vedla prekazky
        if(prelet == 1 && sendCommand == false){
            if(getDistanceFromXYTarget(actual_x, actual_y) < HARD && stage == 2){
                ROS_INFO_STREAM("1.Sme vedla prekazky");
                stage = 3;
                sendCommand = true;
                continue;
            }
            if(getDistanceFromXYTarget(actual_x, actual_y) < HARD && stage == 3){
                ROS_INFO_STREAM("2.Sme za vedla prekazky");
                stage = 4;
                sendCommand = true;
                continue;
            }
            if(getDistanceFromXYTarget(actual_x, actual_y) < HARD && stage == 4){
                ROS_INFO_STREAM("3. SME NA y = 0");
                stage = 1;
                sendCommand = false;
            }
        }

        //na handlovanie prekazky
        if(stage == 1){
            ROS_INFO_STREAM("Vzdialenost od prekazky");
            vzdialenost_obstacle = z[400][400];
            ROS_INFO_STREAM(vzdialenost_obstacle);
            int index_edge_vertical = 0;
            int index_edge_horizontal = 0;
            prelet = 0;
            //ak je pred nami prekazka
            if(z[400][400] < abs(TARGET_POINT[0])){

                //nacitanie horizontalnej linie
                float z_horizontal[SIZE_OF_TABLE/2];
                for(int i =800; i > SIZE_OF_TABLE/2; i--){
                    z_horizontal[800-i] = z[400][i];
                }
                //nacitanie vertikalnej linie
                float z_vertical[SIZE_OF_TABLE/2];
                for(int i =0; i < SIZE_OF_TABLE/2; i++){
                    z_vertical[i] = z[i][400];
                }
                //hladanie hrany vo vertikalnom smere
                for(int i = 0; i < 400; i++){
                    if((z[400][400] - 0.1) <= z_vertical[i] && (z[400][400] + 0.1) > z_vertical[i]){
                        index_edge_vertical = i;
                        break;
                    }
                }
                //hladanie hrany v horizontalnom smere
                for(int i = 0; i < 400; i++){
                    if((z[400][400] - 0.1) <= z_horizontal[i] && (z[400][400] + 0.1) > z_horizontal[i]){
                        index_edge_horizontal = i;
                        break;
                    }
                }

                //urcenie letu -> nad alebo do strany
                sirka_prekazky = abs(x[400][800-index_edge_horizontal]);
                vyska_prekazky = abs(y[index_edge_vertical][400]);
                ROS_INFO_STREAM("Sirka prekazky:");
                ROS_INFO_STREAM(sirka_prekazky);
                ROS_INFO_STREAM("Vyska prekazky:");
                ROS_INFO_STREAM(vyska_prekazky);
                if(sirka_prekazky < vyska_prekazky){
                    ROS_INFO_STREAM("Idem do strany");
                    prelet = 1;
                }
                else{
                    ROS_INFO_STREAM("Idem ponad");
                    prelet = 2;
                }
                if(prelet ==0){
                    ROS_INFO_STREAM("Idem priamo na ciel");
                }
            }
            stage = 2;
            sendCommand = true;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


void flyToXY(float x,float y, float z, ros::Publisher posePublisher){

        mavros_msgs::PositionTarget desired_pose;
        desired_pose.coordinate_frame = 1; 
        desired_pose.position.x = x;
        //ROS_INFO_STREAM(x);
        desired_pose.position.y = y;
        //ROS_INFO_STREAM(y);
        desired_pose.position.z = z;
        desired_pose.yaw = 3.142;
        //ROS_INFO_STREAM(agree);

        posePublisher.publish(desired_pose);
        //ROS_INFO_STREAM("NA BOD XY");
}

void flyToZ(double z, ros::Publisher posePublisher){
        mavros_msgs::PositionTarget desired_pose;
        desired_pose.coordinate_frame = 1; 
        desired_pose.position.z = z;
        desired_pose.position.x = current_position.pose.pose.position.x;
        desired_pose.position.y = current_position.pose.pose.position.y;
        desired_pose.yaw = 3.142;

        posePublisher.publish(desired_pose);
        //ROS_INFO_STREAM("NA BOD Z");
}

void takeOff(double altitude,ros::ServiceClient takeOff_client, ros::ServiceClient arming_client, ros::ServiceClient set_mode_client, ros::Rate rate){

        //set guided mode
        mavros_msgs::SetMode guided_set_mode;
        guided_set_mode.request.custom_mode = "GUIDED";
        set_mode_client.call(guided_set_mode);
        ROS_INFO("Waiting for GUIDED mode");
        while(ros::ok() && !current_state.guided){
                ros::spinOnce();
                rate.sleep();
        }
        ros::Duration(1).sleep();

        //set arming
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
        ros::Duration(1).sleep();

        //call takeoff
        mavros_msgs::CommandTOL takeOff;
        takeOff.request.altitude = altitude;
        takeOff_client.call(takeOff);
        ROS_INFO_STREAM("VZLIETNUTIE");
}

void land(ros::ServiceClient land_client){

        mavros_msgs::CommandTOL land;
        land.request.altitude = 0;
        land_client.call(land);
        ROS_INFO_STREAM("PRISTATIE");
}

double getDistanceFromXYTarget(double x, double y)
{
        return sqrt(pow(current_position.pose.pose.position.x - x,2)+pow(current_position.pose.pose.position.y - y,2));
}

double getDistanceFromZTarget(double height)
{
        double actual_distance = abs(current_position.pose.pose.position.z - height);
        return actual_distance;
}