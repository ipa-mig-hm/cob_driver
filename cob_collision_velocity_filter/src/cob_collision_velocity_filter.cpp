/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <cob_collision_velocity_filter.h>

#include <visualization_msgs/Marker.h>

// Constructor
CollisionVelocityFilter::CollisionVelocityFilter()
{
  // create node handle
  nh_ = ros::NodeHandle("~");

  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  // node handle to get footprint from parameter server
  std::string costmap_parameter_source;
  if(!nh_.hasParam("costmap_parameter_source")) ROS_WARN("Checking default source [/local_costmap_node/costmap] for costmap parameters");
  nh_.param("costmap_parameter_source",costmap_parameter_source, std::string("/local_costmap_node/costmap"));

  ros::NodeHandle local_costmap_nh_(costmap_parameter_source); 	

  // implementation of topics to publish (command for base and list of relevant obstacles)
  topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
  topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
  // implementation of topic to publish potential field of the local costmap
  topic_pub_potential_field_forbidden_ = nh_.advertise<nav_msgs::GridCells>("potential_field_forbidden",1);
  topic_pub_potential_field_warn_ = nh_.advertise<nav_msgs::GridCells>("potential_field_warn",1);  

  // subscribe to twist-movement of teleop 
  joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&CollisionVelocityFilter::joystickVelocityCB, this, _1));
  // subscribe to the costmap to receive inflated cells
  obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&CollisionVelocityFilter::obstaclesCB, this, _1));

  // create service client
  srv_client_get_footprint_ = nh_.serviceClient<cob_footprint_observer::GetFootprint>("/get_footprint");

  // create Timer and call getFootprint Service periodically
  double footprint_update_frequency;
  if(!nh_.hasParam("footprint_update_frequency")) ROS_WARN("Used default parameter for footprint_update_frequency [1.0 Hz].");
  nh_.param("footprint_update_frequency",footprint_update_frequency,1.0);
  get_footprint_timer_ = nh_.createTimer(ros::Duration(1/footprint_update_frequency), boost::bind(&CollisionVelocityFilter::getFootprintServiceCB, this, _1));

  // read parameters from parameter server
  // parameters from costmap
  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/global_frame")) ROS_WARN("Used default parameter for global_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/global_frame", global_frame_, std::string("/base_link"));

  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/robot_base_frame", robot_frame_, std::string("/base_link"));

  if(!nh_.hasParam("influence_radius")) ROS_WARN("Used default parameter for influence_radius [1.5 m]");
  nh_.param("influence_radius", influence_radius_, 1.5);
  closest_obstacle_dist_ = influence_radius_;
  closest_obstacle_angle_ = 0.0;

  // parameters for obstacle avoidence and velocity adjustment
  if(!nh_.hasParam("stop_threshold")) ROS_WARN("Used default parameter for stop_threshold [0.1 m]");
  nh_.param("stop_threshold", stop_threshold_, 0.10);

  if(!nh_.hasParam("obstacle_damping_dist")) ROS_WARN("Used default parameter for obstacle_damping_dist [5.0 m]");
  nh_.param("obstacle_damping_dist", obstacle_damping_dist_, 5.0);
  if(obstacle_damping_dist_ <= stop_threshold_) {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    ROS_WARN("obstacle_damping_dist <= stop_threshold -> robot will stop without decceleration!");
  }

  if(!nh_.hasParam("pot_ctrl_vmax")) ROS_WARN("Used default parameter for pot_ctrl_vmax [0.6]");
  nh_.param("pot_ctrl_vmax", v_max_, 0.6);

  if(!nh_.hasParam("pot_ctrl_vtheta_max")) ROS_WARN("Used default parameter for pot_ctrl_vtheta_max [0.8]");
  nh_.param("pot_ctrl_vtheta_max", vtheta_max_, 0.8);

  if(!nh_.hasParam("pot_ctrl_kv")) ROS_WARN("Used default parameter for pot_ctrl_kv [1.0]");
  nh_.param("pot_ctrl_kv", kv_, 1.0);

  if(!nh_.hasParam("pot_ctrl_kp")) ROS_WARN("Used default parameter for pot_ctrl_kp [2.0]");
  nh_.param("pot_ctrl_kp", kp_, 2.0);

  if(!nh_.hasParam("pot_ctrl_virt_mass")) ROS_WARN("Used default parameter for pot_ctrl_virt_mass [0.8]");
  nh_.param("pot_ctrl_virt_mass", virt_mass_, 0.8);

  //parameters for Class PotentialFieldCostMap  
  if(!nh_.hasParam("costmap_resolution")) ROS_WARN("Used default parameter for resolution [0.07]");
  nh_.param("costmap_resolution",potential_field_.resolution_,0.07); 
  
  if(!nh_.hasParam("map_height")) ROS_WARN("Used default parameter for map_height [5.0]");
  nh_.param("map_height",potential_field_.map_height_,5.0);
  
  if(!nh_.hasParam("map_width")) ROS_WARN("Used default parameter for map_width [5.0]");
  nh_.param("map_width",potential_field_.map_width_,5.0);
  
  if(!nh_.hasParam("max_potential_value")) ROS_WARN("Used default parameter for max_potential_value [250]");
  nh_.param("map_potential_value",potential_field_.max_potential_value_,250);
 
  potential_field_.influence_radius_ = influence_radius_ ;
  potential_field_.stop_threshold_ = stop_threshold_;
  potential_field_.initial();
  //end of parameters configuration for Class PotentialFieldCostMap
 
  //load the robot footprint from the parameter server if its available in the local costmap namespace
  robot_footprint_ = loadRobotFootprint(local_costmap_nh_);
  if(robot_footprint_.size() > 4) 
    ROS_WARN("You have set more than 4 points as robot_footprint, cob_collision_velocity_filter can deal only with rectangular footprints so far!");

  // try to geht the max_acceleration values from the parameter server
  if(!nh_.hasParam("max_acceleration")) ROS_WARN("Used default parameter for max_acceleration [0.5, 0.5, 0.7]");
  XmlRpc::XmlRpcValue max_acc;
  if(nh_.getParam("max_acceleration", max_acc)) {
    ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ax_max_ = (double)max_acc[0];
    ay_max_ = (double)max_acc[1];
    atheta_max_ = (double)max_acc[2];
  } else {
    ax_max_ = 0.5;
    ay_max_ = 0.5;
    atheta_max_ = 0.7;
  }

  max_warn_value_ = 0;
  last_time_ = ros::Time::now().toSec();
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
  // dynamic reconfigure
  dynCB_ = boost::bind(&CollisionVelocityFilter::dynamicReconfigureCB, this, _1, _2);
  dyn_server_.setCallback(dynCB_);
}

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter(){}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
  pthread_mutex_lock(&m_mutex);
  robot_twist_linear_ = twist->linear;
  robot_twist_angular_ = twist->angular;

  pthread_mutex_unlock(&m_mutex);
  // generate the potential field grid map
  generatePotentialField();
  
  if(collisionPreCalculate() == false) {
    //printf("no move\n");
    stopMovement();
  }
  else{ 
    performControllerStep();
  }
}


void CollisionVelocityFilter::generatePotentialField(){
  potential_field_.initCostMap();
  pthread_mutex_lock(&m_mutex);
  potential_field_.getCostMap(last_costmap_received_);
  pthread_mutex_unlock(&m_mutex);
  potential_field_.cellPFLinearGeneration();
  potential_field_.getPotentialWarn();
  potential_field_.getPotentialForbidden();  
  topic_pub_potential_field_warn_.publish(potential_field_.potential_field_warn_);
  topic_pub_potential_field_forbidden_.publish(potential_field_.potential_field_forbidden_);
}



//return true when collision can be avoided
bool CollisionVelocityFilter::collisionPreCalculate(){

  bool has_collision = true;
  bool in_range = true;
  std::vector<geometry_msgs::Point> robot_footprint;
  geometry_msgs::Vector3 robot_twist_linear = robot_twist_linear_;
  double vel_angle = atan2(robot_twist_linear_.y, robot_twist_linear_.x);
  double vel_length = sqrt(robot_twist_linear_.y * robot_twist_linear_.y + robot_twist_linear_.x * robot_twist_linear_.x);  
  double rotate_angle = 0;
  pthread_mutex_lock(&m_mutex);
  robot_footprint = robot_footprint_; 
  pthread_mutex_unlock(&m_mutex);

  //no command
  if(robot_twist_linear_.x==0 && robot_twist_linear_.y==0 && robot_twist_angular_.z ==0) return true;
  //stop rotation when translation exists
  if(robot_twist_linear_.x!=0 || robot_twist_linear_.y!=0){
    robot_twist_angular_.z = 0;
    //check the footprint in 0 to -75 degree
    while(has_collision && in_range){
      has_collision = potential_field_.collisionByTranslation(robot_twist_linear,robot_footprint,stop_threshold_);
      if(has_collision){
        rotate_angle = rotate_angle - (M_PI / 180.0f);
        if(rotate_angle < -M_PI/2.4) in_range = false; 
        else modifyCommand(rotate_angle,robot_twist_linear,vel_angle,vel_length);
      }
    }    
    //now try the footprint in 0 to 75 degree
    in_range = true;
    rotate_angle = 0;
    while(has_collision && in_range){
      rotate_angle = rotate_angle + (M_PI / 180.0f);
      if(rotate_angle > M_PI/2.4) in_range = false;
      else{
        modifyCommand(rotate_angle,robot_twist_linear,vel_angle,vel_length);
        has_collision = potential_field_.collisionByTranslation(robot_twist_linear,robot_footprint,stop_threshold_);
      }
    }
   
    //if in_range is true, the robot is allow to move
    if(in_range) {
      robot_twist_linear_ = robot_twist_linear;
      max_warn_value_ =  potential_field_.findWarnValue(robot_twist_linear_);
     }
    
    return in_range;    
  }

  //only rotation exists
  else{
    //check collision after rotation
    if(potential_field_.collisionByRotation(robot_twist_angular_.z,robot_footprint)){ 
    return false; 
    }    
    return true;
  }     
  
}


void CollisionVelocityFilter::modifyCommand(double rotate_angle, geometry_msgs::Vector3& robot_twist_linear,double vel_angle,double vel_length){
  double new_vel_angle = vel_angle + rotate_angle;
  robot_twist_linear.x = vel_length * cos(new_vel_angle);
  robot_twist_linear.y = vel_length * sin(new_vel_angle);
  //ROS_INFO("modifeid command x:%f,y:%f",robot_twist_linear.x,robot_twist_linear.y); 
}

// obstaclesCB reads obstacles from costmap
void CollisionVelocityFilter::obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
  pthread_mutex_lock(&m_mutex);

  if(obstacles->cells.size()!=0) costmap_received_ = true;
  last_costmap_received_ = * obstacles;

  if(stop_threshold_ < obstacles->cell_width / 2.0f || stop_threshold_ < obstacles->cell_height / 2.0f)
    ROS_WARN("You specified a stop_threshold that is smaller than resolution of received costmap!");

  pthread_mutex_unlock(&m_mutex);
}

// timer callback for periodically checking footprint
void CollisionVelocityFilter::getFootprintServiceCB(const ros::TimerEvent&) 
{
  cob_footprint_observer::GetFootprint srv = cob_footprint_observer::GetFootprint();
  // check if service is reachable
  if (srv_client_get_footprint_.call(srv))
  {
    // adjust footprint
    geometry_msgs::PolygonStamped footprint_poly = srv.response.footprint;
    std::vector<geometry_msgs::Point> footprint;
    geometry_msgs::Point pt;

    for(unsigned int i=0; i<footprint_poly.polygon.points.size(); ++i) {
      pt.x = footprint_poly.polygon.points[i].x;
      pt.y = footprint_poly.polygon.points[i].y;
      pt.z = footprint_poly.polygon.points[i].z;
      footprint.push_back(pt);
    }

    pthread_mutex_lock(&m_mutex);

    footprint_front_ = footprint_front_initial_;
    footprint_rear_ = footprint_rear_initial_;
    footprint_left_ = footprint_left_initial_;
    footprint_right_ = footprint_right_initial_;

    robot_footprint_ = footprint;
    //read the new footprint
    //potential_field_.getFootPrintCells(robot_footprint_);
    for(unsigned int i=0; i<footprint.size(); i++) {
      if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
      if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
      if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
      if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
    }

    pthread_mutex_unlock(&m_mutex);
  
  } else {
    ROS_WARN("Cannot reach service /get_footprint");
  }

}

void
CollisionVelocityFilter::dynamicReconfigureCB(const cob_collision_velocity_filter::CollisionVelocityFilterConfig &config,
                                              const uint32_t level)
{
  pthread_mutex_lock(&m_mutex);

  stop_threshold_ = config.stop_threshold;
  obstacle_damping_dist_ = config.obstacle_damping_dist;
  if(obstacle_damping_dist_ <= stop_threshold_) {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    ROS_WARN("obstacle_damping_dist <= stop_threshold -> robot will stop without decceleration!");
  }

  if(obstacle_damping_dist_ > config.influence_radius || stop_threshold_ > config.influence_radius)
  {
    ROS_WARN("Not changing influence_radius since obstacle_damping_dist and/or stop_threshold is bigger!");
  } else {
    influence_radius_ = config.influence_radius;
  }

  if (stop_threshold_ <= 0.0 || influence_radius_ <=0.0)
    ROS_WARN("Turned off obstacle avoidance!");
  pthread_mutex_unlock(&m_mutex);
}


void CollisionVelocityFilter::performControllerStepNew() {
 
  double vx_max,vy_max;
  geometry_msgs::Twist cmd_vel,cmd_vel_in;
  cmd_vel_in.linear = robot_twist_linear_;
  cmd_vel_in.angular = robot_twist_angular_;
  
  double vel_angle = atan2(cmd_vel.linear.y,cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max < fabs(cmd_vel.linear.x)) cmd_vel.linear.x = sign(cmd_vel.linear.x) * vx_max;
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max < fabs(cmd_vel.linear.y)) cmd_vel.linear.y = sign(cmd_vel.linear.y) * vy_max;
    
}


void CollisionVelocityFilter::performControllerStep() {

  if(max_warn_value_ == potential_field_.max_potential_value_) ROS_WARN("find footprint inside the obstacle!");
  //else  ROS_INFO("max_value is now %d",max_warn_value_);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular = robot_twist_angular_; 
  cmd_vel.linear.x = robot_twist_linear_.x * 0.2 + (robot_twist_linear_.x * ((potential_field_.forbidden_value_ - max_warn_value_)/(double)potential_field_.forbidden_value_)); 
  cmd_vel.linear.y = robot_twist_linear_.y * 0.2 + (robot_twist_linear_.y * ((potential_field_.forbidden_value_ - max_warn_value_)/(double)potential_field_.forbidden_value_));
  cmd_vel.linear.z = 0;
  if(fabs(cmd_vel.linear.x)>fabs(robot_twist_linear_.x))  cmd_vel.linear.x = robot_twist_linear_.x;
  if(fabs(cmd_vel.linear.y)>fabs(robot_twist_linear_.y))  cmd_vel.linear.y = robot_twist_linear_.y;
  
  //ROS_INFO("publish command is %f,%f\n",cmd_vel.linear.x,cmd_vel.linear.y);
  topic_pub_command_.publish(cmd_vel); 

}

void CollisionVelocityFilter::stopMovement() {
  geometry_msgs::Twist stop_twist;
  ROS_INFO("cannot move anymore");
  stop_twist.linear.x = 0.0f; stop_twist.linear.y = 0.0f; stop_twist.linear.z = 0.0f;
  stop_twist.angular.x = 0.0f; stop_twist.angular.y = 0.0f; stop_twist.linear.z = 0.0f;
  topic_pub_command_.publish(stop_twist);
}

// load robot footprint from costmap_2d_ros to keep same footprint format
std::vector<geometry_msgs::Point> CollisionVelocityFilter::loadRobotFootprint(ros::NodeHandle node){
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;
  double padding;

  std::string padding_param, footprint_param;
  if(!node.searchParam("footprint_padding", padding_param))
    padding = 0.01;
  else
    node.param(padding_param, padding, 0.01);

  //grab the footprint from the parameter server if possible
  XmlRpc::XmlRpcValue footprint_list;
  std::string footprint_string;
  std::vector<std::string> footstring_list;
  if(node.searchParam("footprint", footprint_param)){
    node.getParam(footprint_param, footprint_list);
    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      footprint_string = std::string(footprint_list);

      //if there's just an empty footprint up there, return
      if(footprint_string == "[]" || footprint_string == "")
        return footprint;

      boost::erase_all(footprint_string, " ");

      boost::char_separator<char> sep("[]");
      boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
      footstring_list = std::vector<std::string>(tokens.begin(), tokens.end());
    }
    //make sure we have a list of lists
    if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) && !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5)){
      ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", footprint_param.c_str(), std::string(footprint_list).c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < footprint_list.size(); ++i){
        //make sure we have a list of lists of size 2
        XmlRpc::XmlRpcValue point = footprint_list[i];
        if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
          ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
        }

        //make sure that the value we're looking at is either a double or an int
        if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
        pt.x += sign(pt.x) * padding;

        //make sure that the value we're looking at is either a double or an int
        if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
        pt.y += sign(pt.y) * padding;

        footprint.push_back(pt);

        node.deleteParam(footprint_param);
        std::ostringstream oss;
        bool first = true;
        BOOST_FOREACH(geometry_msgs::Point p, footprint) {
          if(first) {
            oss << "[[" << p.x << "," << p.y << "]";
            first = false;
          }
          else {
            oss << ",[" << p.x << "," << p.y << "]";
          }
        }
        oss << "]";
        node.setParam(footprint_param, oss.str().c_str());
        node.setParam("footprint", oss.str().c_str());
      }
    }

    else if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::vector<geometry_msgs::Point> footprint_spec;
      bool valid_foot = true;
      BOOST_FOREACH(std::string t, footstring_list) {
        if( t != "," ) {
          boost::erase_all(t, " ");
          boost::char_separator<char> pt_sep(",");
          boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
          std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

          if(point.size() != 2) {
            ROS_WARN("Each point must have exactly 2 coordinates");
            valid_foot = false;
            break;
          }

          std::vector<double>tmp_pt;
          BOOST_FOREACH(std::string p, point) {
            std::istringstream iss(p);
            double temp;
            if(iss >> temp) {
              tmp_pt.push_back(temp);
            }
            else {
              ROS_WARN("Each coordinate must convert to a double.");
              valid_foot = false;
              break;
            }
          }
          if(!valid_foot)
            break;

          geometry_msgs::Point pt;
          pt.x = tmp_pt[0];
          pt.y = tmp_pt[1];

          footprint_spec.push_back(pt);
        }
      }
      if (valid_foot) {
        footprint = footprint_spec;
        node.setParam("footprint", footprint_string);
      }
      else {
        ROS_FATAL("This footprint is not vaid it must be specified as a list of lists with at least 3 points, you specified %s", footprint_string.c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }
    }
  }

  footprint_right_ = 0.0f; footprint_left_ = 0.0f; footprint_front_ = 0.0f; footprint_rear_ = 0.0f;
  //extract rectangular borders for simplifying:
  for(unsigned int i=0; i<footprint.size(); i++) {
    if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
    if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
    if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
    if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
  }
  ROS_DEBUG("Extracted rectangular footprint for cob_collision_velocity_filter: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

  return footprint;
}

double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double CollisionVelocityFilter::sign(double x) {
  if(x >= 0.0f) return 1.0f;
  else return -1.0f;
}



//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_collision_velocity_filter");

  // create nodeClass
  CollisionVelocityFilter collisionVelocityFilter;


  ros::spin();

  return 0;
}

