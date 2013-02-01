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


class FootPrintLine
{
  public:
    // constructor
    FootPrintLine(); 
    // destructor
    ~FootPrintLine();
     // the slope of the line
    double slope_;
    // the offset of the line
    double offset_;

    double x_start_;
    double x_end_;
    double y_start_;
    double y_end_;
    std::vector<int> index_x_;
    std::vector<int> index_y_;
    bool has_slope_;
};
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
    void getPotentialForbidden();  
    void getPotentialWarn();
    ///
    /// @brief  reads the robot footprint and calculate the related cells to the footprint
    void getFootPrintCells(const std::vector<geometry_msgs::Point>& footprint);

    nav_msgs::GridCells potential_field_forbidden_;
    nav_msgs::GridCells potential_field_warn_;
    std::vector<geometry_msgs::Point> footprint_;
    std::vector<FootPrintLine> footprint_line_;
  private: 
    /// 
    /// @brief return the potential field value of the cell grouped by rectangle
    /// 
    int getRectangleCellValue(int x, int y);
     
    ///
    /// @brief return the potential field value of the cell grouped by circle
    ///
    int getCircleCellValue(int x, int y); 
 
    ///
    /// @brief  initialize the grid map, set all grid to '0'
    /// @param  size_x,size_y - grid size
    ///
   
    /// 
    /// @brief  get the coordinate of x based on the index of the cell
    /// @param  index_x - x index of the cell array 
    ///
    double getCellCoordX(int index_x);
   

    /// 
    /// @brief  get the coordinate of y based on the index of the cell
    /// @param  index_y - y index of the cell array 
    ///
    double getCellCoordY(int index_y);


    ///
    /// @brief find the closet Cell on the footprint 
    ///        and store the result in closest_cell_x_ and closest_cell_y_
    void  findClosestCell();


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
    bool costmap_received_;
    //resolution of the imported costmap
    double resolution_; 
    double map_width_;
    double map_height_;
    int cell_size_x_;
    int cell_size_y_;
    int* cost_map_;
    nav_msgs::GridCells last_costmap_received_;
    //grid map 
    double grid_resolution_;
    int grid_size_x_;
    int grid_size_y_;
    int* grid_map_;
    //potential field
    double influence_radius_; 
    int max_potential_value_;
    int step_value_;
    double stop_threshold_;
    int closest_cell_x_;
    int closest_cell_y_;
    double closet_cell_angle_;	
}; //PotentialFieldGridMap
#endif

