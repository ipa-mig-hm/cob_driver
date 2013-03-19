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
/// @class FootPrintLine
/// @brief store the information of every segment of the robot footprint
/// 
class FootPrintLine
{
  public:
    // constructor
    FootPrintLine(); 
    // destructor
    ~FootPrintLine();

    // the slope and the offset of the line
    // Y = slope * X + offset
    double slope_;
    double offset_;
  
    // start and end coordinates of the line
    double x_start_;
    double x_end_;
    double y_start_;
    double y_end_;
   
    // cell indexes of the line in pf-costmap
    int index_x_start_;
    int index_y_start_;
    int index_x_end_;
    int index_y_end_;
    std::vector<int> index_x_;
    std::vector<int> index_y_;

    // check if the slope exists or not 
    bool has_slope_;

    // set if this footprint line is related to robot moving direction 
    bool related_;

   /* different types of lines is related to different directions 
       0 -- north
       1 -- east 
       2 -- south
       3 -- west 
       4 -- north and east
       5 -- south and east 
       6 -- south and west 
       7 -- north and west
       8 -- west,north and east  (higher on left  side)
       9 -- west,north and east  (higher on right side)
      10 -- north,east and south (bigger on left  side)
      11 -- north,east and south (bigger on right side)
      12 -- east,south and west  (higher on left  side)
      13 -- east,south and west  (higher on right side)
      14 -- south,west and north (bigger on left  side)
      15 -- south,west and north (bigger on right side)
      16 -- unknow */
    int related_type_;


};


///
/// @class PotentialFieldCostMap
/// @a 2D grid map that provides the infomation of the potential field of the obstacle.  It takes the original 
/// the data from costmap_2d  
///
class PotentialFieldCostMap
{  
  public:
 
    ///
    /// @brief  Constructor
    ///
    PotentialFieldCostMap();
    
    ///
    /// @brief  Destructor
    ///
    ~PotentialFieldCostMap();

    ///
    /// @brief initialize the parameters of the potential field costmap
    /// 
    void initial(); 



    /***core functions***/

    ///
    /// @brief generates potential field for the costmap cell linearly
    ///
    void cellPFLinearGeneration();

    ///
    /// @brief check if collision will happen when translates
    /// @param cmd_vel- raw velocity command    
    /// @param footprint - robot footprint          
    /// @return true if collision will happen
    ///        
    bool collisionByTranslation(const geometry_msgs::Vector3& cmd_vel, const std::vector<geometry_msgs::Point>& footprint,double predicted_distance); 

    ///
    /// @brief check if collision will happen when rotates
    /// @param angular - raw angular-velocity command
    /// @param footprint - robot footprint
    /// @return true if collision will happen
    /// 
    bool collisionByRotation(double& angular,const std::vector<geometry_msgs::Point>& footprint);

    ///
    /// @brief  reads the robot footprint and calculate the related cells of the footprint
    /// @param  footprint - robot footprint 
    ///
    void getFootPrintCells(const std::vector<geometry_msgs::Point>& footprint);

    ///
    /// @brief find the biggest potential value in related footpirnt line 
    /// @param cmd_vel - raw velocity command
    /// @return - the max cell value in related footprint line/lines
    ///       
    int findWarnValue(const geometry_msgs::Vector3& cmd_vel);

     
    ///
    /// @brief  reads obstacles from costmap and store the map information in cost_map_
    /// @param  obstacles - 2D occupancy grid in rolling window mode
    ///
    void getCostMap(const nav_msgs::GridCells& last_costmap_received);
  

    /***helper functions***/
    ///
    /// @brief initialize the costmap
    ///
    void initCostMap();
    
    ///
    /// @brief generate the msg for potential_field_forbidden_
    ///
    void getPotentialForbidden();  

    ///
    /// @brief generate the msg for potential_field_warn_
    /// 
    void getPotentialWarn();
 
    ///
    /// @brief find the closet line on the footprint, set the line number
    ///        and the start angle and end angle of the closest line
    ///
    void findClosestLine();
  
    ///
    /// @brief initialize the gridmap
    ///
    void initGridMap();

    ///
    /// @brief  reads obstacles from costmap and coverts it to grid obstacles
    ///         then store the map information in grid_map_
    /// @param  obstacles - 2D occupancy grid in rolling window mode
    ///
    void getGridMap(const nav_msgs::GridCells& last_costmap_received);

    
    nav_msgs::GridCells potential_field_forbidden_;
    nav_msgs::GridCells potential_field_warn_;
    double resolution_; 
    double influence_radius_; 
    double stop_threshold_;
    double map_width_;
    double map_height_;
    int max_potential_value_;
    int forbidden_value_;
    

  private: 

    ///
    /// @brief find and set the related lines to a certain velocity command 
    ///
    void setRelatedLine(const geometry_msgs::Vector3& cmd_vel);
   
    /// 
    /// @brief return the potential field value of the cell grouped by rectangle
    /// @param x,y - index of the cell in cost_map_
    ///
    int getRectangleCellValue(int x, int y);
     
    ///
    /// @brief return the potential field value of the cell grouped by circle
    /// @param x,y - index of the cell in cost_map_
    ///
    int getCircleCellValue(int x, int y); 
 
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
    /// @brief check if the expected footprint is overlapped with obstacles
    /// @return true if collision exists 
    ///    
    bool checkCollision();


    ///
    /// @brief  delete the grid map
    ///
    void deleteGridMap();
        
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

    //costmap
    bool costmap_received_;
    int cell_size_x_;
    int cell_size_y_;
    bool cell_size_x_odd_;
    bool cell_size_y_odd_; 
    int* cost_map_;
    nav_msgs::GridCells last_costmap_received_;
    //grid map 
    double grid_resolution_;
    int grid_size_x_;
    int grid_size_y_;
    int* grid_map_;
    //potential field
    int step_value_;
    int warn_value_;    
    //footprint
    double closest_line_orth_angle_;	
    int closest_line_num_;
    std::vector<FootPrintLine> footprint_line_; 
}; //PotentialFieldCostMap
#endif

