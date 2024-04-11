#include<add_obstacle_layer/add_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_math.h>
#include <tf/transform_listener.h>
 
PLUGINLIB_EXPORT_CLASS(add_layer_namespace::AddObstacleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
 
namespace add_layer_namespace
{
 
AddObstacleLayer::AddObstacleLayer() {}
 
void AddObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  matchSize();
  
  cone_sub_ = nh_.subscribe("/gazebo/cone_position", 1, &AddObstacleLayer::conePositionCallback, this);
  respawn_sub_ = nh_.subscribe("/rviz_panel/respawn_objects", 1, &AddObstacleLayer::respawnCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &AddObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
void AddObstacleLayer::conePositionCallback(const geometry_msgs::Point::ConstPtr& msg) 
{
  dynamic_obstacle_start_ = *msg;
  // Adjust the starting point as needed
  if (dynamic_obstacle_start_.x != 0){
    if (dynamic_obstacle_start_.x == 12.7){
    geometry_msgs::Point p1;
    // p1.x = 11.75;
    // p1.y = -4.4;
    // p1.z = 0.0;  carto

    p1.x = 11.7;
    p1.y = 2.8;
    p1.z = 0.0;

    geometry_msgs::Point p2;
    // p2.x = 13.2;
    // p2.y = -5.05;
    // p2.z = 0.0; carto

    p2.x = 13.3;
    p2.y = 2.8;
    p2.z = 0.0;
    createRandomObstacle(p1.x, p1.y, p2.x, p2.y);
    }
    else{
    geometry_msgs::Point p1;
    // p1.x = 15.32;
    // p1.y = -6.53;
    // p1.z = 0.0; carto

    p1.x = 15.8;
    p1.y = 3.0;
    p1.z = 0.0;

    geometry_msgs::Point p2;
    // p2.x = 16.8;
    // p2.y = -7.23;
    // p2.z = 0.0; carto

    p2.x = 17.5;
    p2.y = 3.1;
    p2.z = 0.0;
    createRandomObstacle(p1.x, p1.y, p2.x, p2.y);
    }
  }
  
}

void AddObstacleLayer::respawnCallback(const std_msgs::Int16::ConstPtr& respawn_msg)
{
  if(respawn_msg->data == 0) {
    clearSpecificObstacles();
  }
}

void AddObstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void AddObstacleLayer::matchSize()
{
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void AddObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
  // 整个地图范围

  Costmap2D* master = layered_costmap_->getCostmap();

  *min_x = -(master->getSizeInCellsX()/(master->getResolution()));
  *min_y = -(master->getSizeInCellsY()/(master->getResolution()));
  *max_x = (master->getSizeInCellsX()/(master->getResolution()));
  *max_y = (master->getSizeInCellsY()/(master->getResolution()));
}
 
void AddObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  // double x1 = 1.92;
  // double y1 = 0.04;
  // double x2 = 3.57;
  // double y2 = -0.66;

  double x1 = 1.4;
  double y1 = 0.99;
  double x2 = 3.03;
  double y2 = 1.02;

  unsigned int mx1, my1, mx2, my2;
  if (!worldToMap(x1, y1, mx1, my1) || !worldToMap(x2, y2, mx2, my2)) {
      ROS_WARN("Failed to convert world coordinates to map coordinates");
      return;
  }

  int dx = abs(static_cast<int>(mx2 - mx1)), dy = -abs(static_cast<int>(my2 - my1));
  int sx = mx1 < mx2 ? 1 : -1, sy = my1 < my2 ? 1 : -1;
  int err = dx + dy; /* error value e_xy */

  while (true) {  /* loop */
      if (master_grid.getCost(mx1, my1) != LETHAL_OBSTACLE) {
          setCost(mx1, my1, LETHAL_OBSTACLE);
      }

      if (mx1 == mx2 && my1 == my2) break;
      int e2 = 2 * err;  // Here's the declaration of e2
      if (e2 >= dy) { err += dy; mx1 += sx; } /* e_xy+e_x > 0 */
      if (e2 <= dx) { err += dx; my1 += sy; } /* e_xy+e_y < 0 */
  }

  for (int j = min_j; j < max_j; j++)
    {
        for (int i = min_i; i < max_i; i++)
        {
            int index = getIndex(i, j);
            if (costmap_[index] == NO_INFORMATION)
                continue;
            unsigned char existing_cost = master_grid.getCost(i, j);
            if (existing_cost == NO_INFORMATION || existing_cost < costmap_[index])
                master_grid.setCost(i, j, costmap_[index]);
        }
    }
     
}

void AddObstacleLayer::createRandomObstacle(double x1, double y1, double x2, double y2)
{
  Costmap2D* master = layered_costmap_->getCostmap();
  unsigned int mx1, my1, mx2, my2;
  if (!worldToMap(x1, y1, mx1, my1) || !worldToMap(x2, y2, mx2, my2)) {
      ROS_WARN("Failed to convert world coordinates to map coordinates");
      return;
  }

  int dx = abs(static_cast<int>(mx2 - mx1)), dy = -abs(static_cast<int>(my2 - my1));
  int sx = mx1 < mx2 ? 1 : -1, sy = my1 < my2 ? 1 : -1;
  int err = dx + dy; /* error value e_xy */

  while (true) {  /* loop */
      if (master->getCost(mx1, my1) != LETHAL_OBSTACLE) {
          setCost(mx1, my1, LETHAL_OBSTACLE);
          obstacles_.push_back(std::make_pair(mx1, my1)); 
      }

      if (mx1 == mx2 && my1 == my2) break;
      int e2 = 2 * err;  // Here's the declaration of e2
      if (e2 >= dy) { err += dy; mx1 += sx; } /* e_xy+e_x > 0 */
      if (e2 <= dx) { err += dx; my1 += sy; } /* e_xy+e_y < 0 */
  }
}

void AddObstacleLayer::clearSpecificObstacles() {
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    unsigned int mx = obstacles_[i].first;
    unsigned int my = obstacles_[i].second;
    setCost(mx, my, costmap_2d::FREE_SPACE); // 清除障碍物
  }
  obstacles_.clear(); // 清空障碍物列表
}
 
} // end namespace
