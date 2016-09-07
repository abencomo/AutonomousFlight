#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <planner/tf_datatypes.h>

using namespace std;

geometry_msgs::PoseStamped DesiredPoseStamped;
float current_x = 0.0, current_y = 0.0, current_z = 0.0;
double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;

float desired_x = 0.0, desired_y = 0.0, desired_z = 1.5;

float obst_x = 0.0, obst_y = 0.0, obst_dist = 0.0;
double obst_thresh = 2.0; // threshold for obstacle distance from origin
int no_obst_timer = 0;
bool return_to_origin = false;

float range_array[3][8] = {0.0};
float average_range[8] = {0.0};
const float range_yaw[8] = {180-22.5, 180+22.5, 270-22.5, 270+22.5, -22.5, 22.5, 90-22.5, 90+22.5 };
float min_range; int min_id;

string desired_mode = "0";

int takeoff_timer = 0;
const float desired_takeoff_time = 1;

int land_timer = 0;
const float desired_land_time = 2;

void TerarangerMessageReceived(const sensor_msgs::LaserScan& rangesMsg) {

	//rangesMsg.ranges[i]
	min_range = 100.0;	min_id = 1;
	for(int i = 1; i<=7; i++)
	{
		range_array[0][i] = range_array[1][i];
		range_array[1][i] = range_array[2][i];
		range_array[2][i] = rangesMsg.ranges[i];

		average_range[i] = (range_array[0][i] + range_array[1][i] + range_array[2][i])/3.0;
	
		if(average_range[i] < min_range)
		{	min_range = average_range[i];	min_id = i;	}
	}

	ROS_INFO("ID: %d, Range: %0.2f", min_id, min_range);

	//from current pose and yaw of A100, get the position from min range sensor
	obst_x = current_x - min_range*sin(current_yaw + range_yaw[min_id]*M_PI/180);	//E
	obst_y = current_y + min_range*cos(current_yaw + range_yaw[min_id]*M_PI/180);	//N
	obst_dist = sqrt(obst_x*obst_x + obst_y*obst_y);

	ROS_INFO("total_yaw: %0.2f", current_yaw + range_yaw[min_id]*M_PI/180);
	ROS_INFO("obstX: %0.2f, obstY: %0.2f", obst_x, obst_y);

	//cases:
	if(obst_dist < obst_thresh)
	{
		desired_x = current_x + (obst_thresh - min_range)*sin(current_yaw + range_yaw[min_id]*M_PI/180);
		desired_y = current_y - (obst_thresh - min_range)*cos(current_yaw + range_yaw[min_id]*M_PI/180);
	}
	else
	{
		desired_x = 0.0;	desired_y = 0.0;
	}

	ROS_INFO("desX: %0.2f, desY: %0.2f", desired_x, desired_y);

/*	DesiredPoseStamped.pose.position.x = desired_x;
	DesiredPoseStamped.pose.position.y = desired_y;
	DesiredPoseStamped.pose.position.z = desired_z;
*/
}

//not activated for now
void ModeMessageReceived(const std_msgs::String& ModeMsg) {
	desired_mode = ModeMsg.data;
	takeoff_timer = 0;
}

void CurrentPoseMessageReceived(const geometry_msgs::PoseStamped& poseMsg) {

	current_x = poseMsg.pose.position.x;
	current_y = poseMsg.pose.position.y;
	current_z = poseMsg.pose.position.z;

	tf::Quaternion quat(poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);
	tf::Matrix3x3 m(quat);
	m.getRPY(current_roll, current_pitch, current_yaw);
	current_yaw = -(current_yaw - M_PI/2);	//yaw with up axis
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "planner");
   ros::NodeHandle n;
   ros::Publisher desired_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Subscriber sub_mode = n.subscribe("/navigation_mode", 1000, &ModeMessageReceived);
	 ros::Subscriber sub_sensor_msgs = n.subscribe("/scan", 1000, &TerarangerMessageReceived);
   ros::Subscriber sub_geometry_msgs = n.subscribe("/mavros/vision_pose/pose", 1000, &CurrentPoseMessageReceived);
   ros::Rate loop_rate(10);
   ros::spinOnce();

   fill( &range_array[0][0], &range_array[0][0] + sizeof(range_array)/sizeof(range_array[0][0]), 14.0); 

   DesiredPoseStamped.pose.position.x = 0.0;		
   DesiredPoseStamped.pose.position.y = 0.0;		
   DesiredPoseStamped.pose.position.z = -1.0;		

   int count = 0;

   while(ros::ok()){

       DesiredPoseStamped.header.stamp = ros::Time::now();
       DesiredPoseStamped.header.seq=count;
       DesiredPoseStamped.header.frame_id = 1;
       DesiredPoseStamped.pose.orientation.x = 0;
       DesiredPoseStamped.pose.orientation.y = 0;
       DesiredPoseStamped.pose.orientation.z = 0.7071;
       DesiredPoseStamped.pose.orientation.w = 0.7071;

       if(desired_mode.compare("O")==0)
	{
	       DesiredPoseStamped.pose.position.x = 0.0;
	       DesiredPoseStamped.pose.position.y = 0.0;
	       DesiredPoseStamped.pose.position.z = 1.5;			
	}
	else if(desired_mode.compare("T")==0)
	{

	       DesiredPoseStamped.pose.position.x = current_x;
	       DesiredPoseStamped.pose.position.y = current_y;
	       DesiredPoseStamped.pose.position.z = 1.5*(float)takeoff_timer/(10.0*desired_takeoff_time);			
	
		if(DesiredPoseStamped.pose.position.z  > 1.5)
		{	
			DesiredPoseStamped.pose.position.x = 0.0;
			DesiredPoseStamped.pose.position.y = 0.0;
			DesiredPoseStamped.pose.position.z = 1.5;
		}
	
		takeoff_timer++;

	}
	else if(desired_mode.compare("L")==0)
	{
	       DesiredPoseStamped.pose.position.x = current_x;
	       DesiredPoseStamped.pose.position.y = current_y;
	       DesiredPoseStamped.pose.position.z = 1.5 - 2.5*(float)land_timer/(10.0*desired_land_time);

		
		if(DesiredPoseStamped.pose.position.z  < -10.0)
		{	
			DesiredPoseStamped.pose.position.x = 0.0;
			DesiredPoseStamped.pose.position.y = 0.0;
			DesiredPoseStamped.pose.position.z = -10.0;
		}
	
		land_timer++;			
	}
	else if(desired_mode.compare("A")==0)
	{
	       DesiredPoseStamped.pose.position.x = desired_x;
	       DesiredPoseStamped.pose.position.y = desired_y;
	       DesiredPoseStamped.pose.position.z = desired_z;			
	}
	else if(desired_mode.compare("N")==0)
	{
	       DesiredPoseStamped.pose.position.x = 0.0;
	       DesiredPoseStamped.pose.position.y = 1.0;
	       DesiredPoseStamped.pose.position.z = 1.5;			
	}

	else if(desired_mode.compare("S")==0)
	{
	       DesiredPoseStamped.pose.position.x = 0.0;
	       DesiredPoseStamped.pose.position.y = -1.0;
	       DesiredPoseStamped.pose.position.z = 1.5;			
	}

	else if(desired_mode.compare("E")==0)
	{
	       DesiredPoseStamped.pose.position.x = 1.0;
	       DesiredPoseStamped.pose.position.y = 0.0;
	       DesiredPoseStamped.pose.position.z = 1.5;			
	}

	else if(desired_mode.compare("W")==0)
	{
	       DesiredPoseStamped.pose.position.x = -1.0;
	       DesiredPoseStamped.pose.position.y = 0.0;
	       DesiredPoseStamped.pose.position.z = 1.5;			
	}
	else
	{
	       DesiredPoseStamped.pose.position.x = 0.0;
	       DesiredPoseStamped.pose.position.y = 0.0;
	       DesiredPoseStamped.pose.position.z = -10.0;			
	}

//	cout << desired_mode << endl;
	count++;

       //MAVROS expects position in ENU frame
       desired_pub.publish(DesiredPoseStamped);

       ros::spinOnce();
       loop_rate.sleep();
   }
}
