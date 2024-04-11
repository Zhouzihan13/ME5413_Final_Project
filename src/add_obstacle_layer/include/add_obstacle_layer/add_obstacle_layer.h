#ifndef ADD_OBSTACLE_LAYER_H
#define ADD_OBSTACLE_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int16.h>

 
namespace add_layer_namespace
{
 
class AddObstacleLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  AddObstacleLayer();
  virtual ~AddObstacleLayer() = default;

 
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void matchSize();
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  void createRandomObstacle(double x1, double y1, double x2, double y2);
  void clearSpecificObstacles();
 
private:
  void conePositionCallback(const geometry_msgs::Point::ConstPtr& msg);
  void respawnCallback(const std_msgs::Int16::ConstPtr& respawn_msg);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
 
  ros::NodeHandle nh_;
  ros::Subscriber cone_sub_;
  ros::Subscriber respawn_sub_;
  geometry_msgs::Point dynamic_obstacle_start_;
  std::vector<std::pair<unsigned int, unsigned int>> obstacles_;
};
}
#endif
