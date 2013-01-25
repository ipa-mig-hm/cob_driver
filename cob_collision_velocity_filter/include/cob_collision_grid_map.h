#ifndef COB_COLLISION_GRID_MAP_H
#define COB_COLLISION_GRID_MAP_H

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

#include <pthread.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/GridCells.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
 

///
/// @class PotentialFieldGridMap
/// @a 2D grid map that provides the infomation of the potential field of the obstacle.  It takes the original 
/// the data from costmap_2d  
///
class PotentialFieldGridMap
{  
  public:

    ///
    /// @brief  Constructor
    ///
    PotentialFieldGridMap();

    
    ///
    /// @brief  Destructor
    ///
    ~PotentialFieldGridMap();

    /// @brief generates potential field for the costmap cell linearly
    void cellPFLinearGeneration();


    ///
    /// @brief  reads obstacles from costmap and coverts it to grid obstacles
    ///         then store the map information in grid_map_
    /// @param  obstacles - 2D occupancy grid in rolling window mode!
    ///
    void getGridMap(const nav_msgs::GridCells& last_costmap_received);


    ///
    /// @brief  reads obstacles from costmap and store the map information in cost_map_
    /// @param  obstacles - 2D occupancy grid in rolling window mode!
    ///
    void getCostMap(const nav_msgs::GridCells& last_costmap_received);
    void testPrintOut();
    void initGridMap();
    void initCostMap();
  private:

    /* core functions */    
    
    /// 
    /// @brief return the potential field value of the cell grouped by rectangle
    /// 
    int getRectangleCellValue(int x, int y);
     
    ///
    /// @brief return the potential field value of the cell grouped by circle
    ///
    int getCircleCellValue(int x, int y); 



    /* helper functions */
    
    ///
    /// @brief  initialize the grid map, set all grid to '0'
    /// @param  size_x,size_y - grid size
    ///
    
    
    ///
    /// @brief  delete the grid map
    ///
    void deleteGridMap();
    
    ///
    /// @brief  initialize the cost map, set all cell to '0'
    /// @param  size_x,size_y - cell size
    
    
    ///
    /// @brief  delete the grid map
    ///
    void deleteCostMap();

    ///
    /// @bried return the index of grid map
    /// @param grid_x,grid_y - coordinates of grid map
    ///
    int getGridIndex( int grid_x,  int grid_y);

    ///
    /// @bried return the index of cost map
    /// @param cell_x,cell_y - coordinates of cost map
    ///    
    int getCellIndex( int cell_x,  int cell_y);



    pthread_mutex_t m_mutex;
    //nav_msgs::GridCells last_costmap_received_;
    bool costmap_received_;
    //resolution of the imported costmap
    double resolution_; 
    double map_width_;
    double map_height_;
    int cell_size_x_;
    int cell_size_y_;
    int* cost_map_;
    //grid map 
    int grid_resolution_;
    int grid_size_x_;
    int grid_size_y_;
    int* grid_map_;
    //potential field
    double influence_radius_; 
    int max_potential_value_;
    int value_step_;
 	
}; //PotentialFieldGridMap
#endif

