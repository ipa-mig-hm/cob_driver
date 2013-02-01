#include <cob_collision_grid_map.h>

PotentialFieldGridMap::PotentialFieldGridMap()
{
//turn to parameter later on 
  costmap_received_=false;
  resolution_ = 0.07;
  grid_resolution_ = 0.035;
  map_height_ = 5;
  map_width_ = 5; 
  cell_size_x_ = map_width_ / resolution_;
  cell_size_y_ = map_height_ / resolution_; 
  grid_size_x_ = map_width_ / grid_resolution_;
  grid_size_y_ = map_height_ / grid_resolution_; 
  grid_map_ = NULL;
  cost_map_ = NULL;
  influence_radius_ = 1.5;
  // obstacle is detected when potential field value equals to 250
  max_potential_value_ = 250;
  step_value_ = max_potential_value_ / ((influence_radius_ / resolution_) - 1);
  stop_threshold_ = 0.1;
  closet_cell_x_ = -1;
  closet_cell_y_ = -1;
  closet_cell_angle_
}

// Destructor
PotentialFieldGridMap::~PotentialFieldGridMap(){
  delete[] cost_map_; 
}

void PotentialFieldGridMap::getGridMap(const nav_msgs::GridCells& last_costmap_received){
  //index of gridMap
  int grid_x=0;
  int grid_y=0;
  //index of costMap
  int cell_x=0;
  int cell_y=0;
  double map_w = (cell_size_x_-1) * resolution_;
  double map_h = (cell_size_y_-1) * resolution_; 
  if(last_costmap_received.cells.size()!=0) costmap_received_ = true;
  if(!costmap_received_)ROS_WARN("No costmap is received!");
  if(costmap_received_){
    int zoom_in = resolution_ / grid_resolution_;
    for(unsigned int i=0;i<last_costmap_received.cells.size();i++){
      cell_x = (last_costmap_received.cells[i].x + map_w/2) / resolution_;
      cell_y = (last_costmap_received.cells[i].y + map_h/2) / resolution_; 
      for(int j=0;j<zoom_in;j++){
        for(int k=0;k<zoom_in;k++){
          grid_x = cell_x * zoom_in + j;
          grid_y = cell_y * zoom_in + k;
          grid_map_[getGridIndex(grid_x,grid_y)] = max_potential_value_;
	}
      }     	  
    }
  } 
}

void PotentialFieldGridMap::getCostMap(const nav_msgs::GridCells& last_costmap_received){
  last_costmap_received_ = last_costmap_received; 
  int cell_x=0;
  int cell_y=0;
  double map_w = (cell_size_x_ -1)* resolution_;
  double map_h = (cell_size_y_ -1)* resolution_;
  if(last_costmap_received.cells.size()!=0) costmap_received_ = true;
  if(!costmap_received_)ROS_WARN("No costmap is received!");
  if(costmap_received_){
    for(unsigned int i=0;i<last_costmap_received.cells.size();i++){
      cell_x = (last_costmap_received.cells[i].x + map_w/2) / resolution_;
      cell_y = (last_costmap_received.cells[i].y + map_h/2) / resolution_; 
      int index = getCellIndex(cell_x,cell_y);
      if (index!=-1)
      cost_map_[index] = max_potential_value_; 
      else ROS_WARN("nagative index x:%f,y:%f,cell_x= %d",last_costmap_received.cells[i].x,last_costmap_received.cells[i].y,cell_x);
    } 
   } 
}


void PotentialFieldGridMap::cellPFLinearGeneration(){
  int max_effect_number = influence_radius_ / resolution_;
  int* map = new int[cell_size_x_*cell_size_y_];
  int* map_tmp = new int[cell_size_x_*cell_size_y_];
  memset(map_tmp,0,cell_size_x_*cell_size_y_*sizeof(int));  
  // copy from the cost map
  for(int i=0; i<cell_size_x_*cell_size_y_; i++ ){
    map[i] = cost_map_[i]; 
  } 
  for(int i=0; i<cell_size_x_; i++){
    for(int j=0; j<cell_size_y_;j++){
      if(cost_map_[getCellIndex(i,j)]==max_potential_value_){
        for(int i_ob=-max_effect_number;i_ob<=max_effect_number;i_ob++){
          for(int j_ob=-max_effect_number;j_ob<=max_effect_number;j_ob++){
          int index = getCellIndex((i+i_ob),(j+j_ob));
            if(index!=-1){
              map_tmp[index] = getRectangleCellValue(i_ob,j_ob);
             // map_tmp[index] = getCircleCellValue(i_ob,j_ob);
              //update the map
              if(map_tmp[index]>map[index]) map[index] = map_tmp[index];             
	    } 
	  }      
	}
      }
    }
  }
  for(int i=0; i<cell_size_x_*cell_size_y_; i++){
  cost_map_[i] = map[i];
  }
  delete[] map_tmp;
  delete[] map;
}

void PotentialFieldGridMap::getPotentialForbidden(){
  potential_field_forbidden_.header = last_costmap_received_.header;
  potential_field_forbidden_.cell_width = last_costmap_received_.cell_width;
  potential_field_forbidden_.cell_height = last_costmap_received_.cell_height;
  potential_field_forbidden_.cells.clear();
  geometry_msgs::Point forbidden_cell;
  int forbidden_cell_length = ((stop_threshold_ / resolution_));
  int forbidden_value = max_potential_value_ - forbidden_cell_length * step_value_;
  for(int i=0;i<cell_size_x_;i++){
    for(int j=0;j<cell_size_y_;j++ ){
      int index = getCellIndex(i,j);
      if(cost_map_[index]>=forbidden_value){
      forbidden_cell.x = (i+1)*resolution_ - cell_size_x_*resolution_/2;
      forbidden_cell.y = (j+1)*resolution_ - cell_size_y_*resolution_/2;
      forbidden_cell.z = 0;
      potential_field_forbidden_.cells.push_back(forbidden_cell);
      }              
    }
  }
}

void PotentialFieldGridMap::getPotentialWarn(){
  potential_field_warn_.header = last_costmap_received_.header;
  potential_field_warn_.cell_width = last_costmap_received_.cell_width;
  potential_field_warn_.cell_height = last_costmap_received_.cell_height;
  potential_field_warn_.cells.clear();
  geometry_msgs::Point warn_cell;
  int forbidden_cell_length = ((stop_threshold_ / resolution_));
  int forbidden_value = max_potential_value_ - forbidden_cell_length * step_value_;
  int warn_cell_length = (influence_radius_ - 0.5) / resolution_;
  int warn_value = max_potential_value_ - warn_cell_length * step_value_;
  for(int i=0;i<cell_size_x_;i++){
    for(int j=0;j<cell_size_y_;j++){
      int index = getCellIndex(i,j);
      if((cost_map_[index]>=warn_value)&&(cost_map_[index]<forbidden_value)){
      warn_cell.x = getCellCoordX(i);
      warn_cell.y = getCellCoordY(j);
      warn_cell.z = 0;
      potential_field_warn_.cells.push_back(warn_cell);
      }
    }
  }
}


double PotentialFieldGridMap::getCellCoordX(int index_x){
  double cell_x;
  if((cell_size_x_%2)==1) cell_x = (index_x+1)*resolution_ - cell_size_x_*resolution_/2;
  else cell_x = (index_x+1)*resolution_ - (cell_size_x_+1)*resolution_/2;  
  return cell_x;
}

double PotentialFieldGridMap::getCellCoordY(int index_y){
  double cell_y;
  if((cell_size_y_%2)==1) cell_y = (index_y+1)*resolution_ - cell_size_y_*resolution_/2;
  else cell_y = (index_y+1)*resolution_ - (cell_size_y_+1)*resolution_/2;  
  return cell_y;
}


int PotentialFieldGridMap::getRectangleCellValue(int x, int y){
  int cell_value;
  if(fabs(x)>fabs(y)) cell_value = max_potential_value_ - fabs(x)*step_value_;
  else cell_value =  max_potential_value_ - fabs(y)*step_value_;
  return cell_value;
}

int PotentialFieldGridMap::getCircleCellValue(int x, int y){
  int cell_value;
  cell_value = max_potential_value_ - sqrt(x*x + y*y)*step_value_;
  return cell_value;
}

void PotentialFieldGridMap::initGridMap(){
  grid_map_ = new int[grid_size_x_*grid_size_y_];
  memset(grid_map_,0,grid_size_x_*grid_size_y_*sizeof(int));  
}


void PotentialFieldGridMap::deleteGridMap(){
  delete[] grid_map_; 
}

void PotentialFieldGridMap::initCostMap(){
  cost_map_ = new int[cell_size_x_*cell_size_y_];
  memset(cost_map_,0,cell_size_x_*cell_size_y_*sizeof(int));  
}


void PotentialFieldGridMap::deleteCostMap(){
  delete[] cost_map_; 
}


int PotentialFieldGridMap::getGridIndex(int grid_x, int grid_y){
  if((grid_x>=0)&&(grid_x<grid_size_x_)&&(grid_y>=0)&&(grid_y<grid_size_y_))
  return grid_y * grid_size_x_ + grid_x;
  else {
    return -1;
  }
}

int PotentialFieldGridMap::getCellIndex(int cell_x,int cell_y){
  if((cell_x>=0)&&(cell_x<cell_size_x_)&&(cell_y>=0)&&(cell_y<cell_size_y_))
  return cell_y * cell_size_x_ + cell_x; 
  else{
    return -1;
  } 
}

void PotentialFieldGridMap::testPrintOut(){
  for(int i=0;i<cell_size_x_;i++){
    for(int j=0;j<cell_size_y_;j++){         
     printf("%d ",cost_map_[getCellIndex(i,j)]);   
    }
     printf("\n");
  }   
}


void PotentialFieldGridMap::findClosestCell(){
 




}

FootPrintLine::FootPrintLine(){  
}

FootPrintLine::~FootPrintLine(){
}

void PotentialFieldGridMap::getFootPrintCells(const std::vector<geometry_msgs::Point>& footprint){
  //calculate the slope of the lines
  int line_num = footprint.size();
  footprint_line_.clear();
  //initialize all footprint lines
  double map_w = (cell_size_x_-1) * resolution_;
  double map_h = (cell_size_y_-1) * resolution_; 
  FootPrintLine fp_line;
  for(int i=0;i<line_num;i++){
     int i_next = (i+1) % line_num;
     fp_line.x_start_ = footprint[i].x;
     fp_line.x_end_ = footprint[i_next].x;
     fp_line.y_start_ = footprint[i].y;
     fp_line.y_end_ = footprint[i_next].y;
     if(footprint[i].x==footprint[i_next].x)  fp_line.has_slope_ = false;
     else fp_line.has_slope_ = true;
     if(fp_line.has_slope_ == true){
       fp_line.slope_ = (footprint[i_next].y - footprint[i].y ) / (footprint[i_next].x - footprint[i].x); 
       fp_line.offset_ = footprint[i].x * footprint[i_next].y - footprint[i_next].x * footprint[i].y;  
       fp_line.offset_ = fp_line.offset_ / (footprint[i].x - footprint[i_next].x); 
     }
      else {
        fp_line.offset_ = footprint[i].x; 
        fp_line.slope_ = 0;
      }
      fp_line.index_x_.clear();
      fp_line.index_y_.clear();
  footprint_line_.push_back(fp_line); 
  }
  for(unsigned int i=0;i<footprint_line_.size();i++){
    int index_x_start = 0;
    int index_y_start = 0; 
    int index_x_end = 0;
    int index_y_end = 0;
    index_x_start = (footprint_line_[i].x_start_ + map_w/2) / resolution_;
    index_y_start = (footprint_line_[i].y_start_ + map_h/2) / resolution_; 
    index_x_end = (footprint_line_[i].x_end_ + map_w/2) / resolution_;
    index_y_end = (footprint_line_[i].y_end_ + map_h/2) / resolution_; 
    if(index_x_start>index_x_end) std::swap(index_x_start,index_x_end);
    if(index_y_start>index_y_end) std::swap(index_y_start,index_y_end);
    double cell_front_x,cell_rear_x,cell_left_y,cell_right_y;
    if(footprint_line_[i].has_slope_ == true){
      for(int j=index_y_start;j<=index_y_end;j++){
        for(int k=index_x_start;k<=index_x_end;k++){
          bool is_corner = false;
          bool is_overlap = false;
          if(((j==index_y_start)&&(k==index_x_start))||((j==index_y_end)&&(k==index_x_end))) is_corner = true;    
          cell_left_y  = footprint_line_[i].slope_*(getCellCoordX(k)-resolution_/2) + footprint_line_[i].offset_;
          cell_right_y = footprint_line_[i].slope_*(getCellCoordX(k)+resolution_/2) + footprint_line_[i].offset_;
          cell_front_x = ((getCellCoordY(j)+resolution_/2) - footprint_line_[i].offset_) / footprint_line_[i].slope_; 
          cell_rear_x  = ((getCellCoordY(j)-resolution_/2) - footprint_line_[i].offset_) / footprint_line_[i].slope_;
          if(((cell_left_y>=(getCellCoordY(j)-resolution_/2))&&(cell_left_y<=(getCellCoordY(j)+resolution_/2)))||((cell_right_y>=(getCellCoordY(j)-resolution_/2))&&(cell_right_y<=(getCellCoordY(j)+resolution_/2)))||((cell_front_x>=(getCellCoordX(k)-resolution_/2))&&(cell_front_x<=(getCellCoordX(k)+resolution_/2)))||((cell_rear_x>=(getCellCoordX(k)-resolution_/2))&&(cell_front_x<=(getCellCoordX(k)+resolution_/2)))) is_overlap = true;
          if(is_overlap){
            if(!is_corner){
   	      footprint_line_[i].index_x_.push_back(k);
              footprint_line_[i].index_y_.push_back(j);
            }
          }
        }
      }
    }  
    else{
      for(int j=index_y_start+1;j<index_y_end;j++){
        footprint_line_[i].index_x_.push_back(index_x_start);
        footprint_line_[i].index_y_.push_back(j);
      }
    }     
  }

}





