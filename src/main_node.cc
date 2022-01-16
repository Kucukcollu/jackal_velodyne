#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace Robot
{
    class Motion
    {
        private:
            ros::NodeHandle nh;
            
            ros::Publisher pub;
            ros::Subscriber sub;

            nav_msgs::Odometry robot_odom;
            geometry_msgs::Twist robot_velocity;

            double x,y,z,roll,pitch,yaw;

            double x_goal = 4.0;
            double y_goal = 4.0;

            double K_linear = 0.8;
            double K_angular = 1.2;

            float distance;
            float desired_angle;
            
            float linear_speed;
            float angular_speed;

        public:
            Motion()
            {
                pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
                sub = nh.subscribe<nav_msgs::Odometry>("odometry/filtered",1000,&Motion::odom_callback,this);
            }

            void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
            {
                x = msg->pose.pose.position.x;
                y = msg->pose.pose.position.y;
                z = msg->pose.pose.position.z;

                tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);

                tf::Matrix3x3 m(q);

                m.getRPY(roll, pitch, yaw);
                
                robot_move();
            }

            void robot_move()
            {
                distance = abs(pow(x_goal-x,2)+pow(y_goal-y,2));
                linear_speed = distance * K_linear;

                desired_angle = atan2(y_goal-y, x_goal-x);
                angular_speed = (desired_angle-yaw)*K_angular;

                robot_velocity.linear.x = linear_speed;
                robot_velocity.angular.z = angular_speed;

                if(distance<0.01)
                {
                    robot_velocity.linear.x=0.0;
                    robot_velocity.angular.z=0.0;
                    pub.publish(robot_velocity);
                    ros::shutdown();
                }

                ROS_INFO("X: %f Y: %f Theta: %f",x,y,yaw);

                pub.publish(robot_velocity);

            }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"main_node");

    Robot::Motion jackal;

    ros::spin();

    return 0;
}
