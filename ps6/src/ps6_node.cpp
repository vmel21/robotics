#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam1_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if (g_take_new_snapshot) {
        ROS_INFO_STREAM("image from cam1: " << message_holder << endl);
        g_cam1_data = message_holder;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

	//start up the competition
    startup_srv.response.success = false;
    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    
    ROS_INFO("got success response from startup service");
	//start up the conveyor
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;

    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
   
    
    ROS_INFO("got success response from conveyor service");
	
	//begin monitoring the logical camera
    g_take_new_snapshot = true;
	bool box_not_found = true;
	bool box_is_under = false;
   
	while (box_not_found) {
		//look for box
	    while (g_cam1_data.models.size() < 1) {
    		ros::spinOnce();
    		ros::Duration(0.5).sleep();
	    }

		//found box
	    box_not_found = false;
	    ROS_INFO("I see a box");

		//have to keep looking for the right location 
	    while (!box_is_under) {

    		ros::spinOnce();
    		ros::Duration(0.5).sleep();
		
		//the box is directly under the camera within reason
	    	if (abs(0 - g_cam1_data.models[0].pose.position.z) < .10) {
    			box_is_under = true;
    			ROS_INFO("box is under camera");
			cout << "The z value of the box is ";
			cout << endl;
			cout << g_cam1_data.models[0].pose.position.z; 
			
			//kill the conveyor and wait for 5 sec
    			conveyor_srv.request.power = 0.0;
    		    	conveyor_srv.response.success = false;

    		    while (!conveyor_srv.response.success) {
        			ROS_WARN("not successful stopping conveyor yet...");
        			conveyor_client.call(conveyor_srv);
        			ros::Duration(0.5).sleep();
    		    }

    		    conveyor_srv.request.power = 100.0;
                    ros::Duration(5.0).sleep();
    		    conveyor_srv.response.success = false;
			
			//restart the conveyor
    		    while (!conveyor_srv.response.success) {
        			ROS_WARN("not successful starting conveyor yet...");
        			conveyor_client.call(conveyor_srv);
        			ros::Duration(0.5).sleep();
		        }
		 
	        }
	    }
		//we have gotten what we wanted, we no longer need to take photos
   		g_take_new_snapshot = false;
	}
    


 //call the drone to pick up the package
    drone_srv.request.shipment_type = "order_0_shipment_0";
    drone_srv.response.success = false;

    while (!drone_srv.response.success) {
        ROS_WARN("not successful starting drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from drone service");

}
