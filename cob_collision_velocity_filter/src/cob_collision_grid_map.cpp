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
      warn_cell.x = (i+1)*resolution_ - cell_size_x_*resolution_/2;
      warn_cell.y = (j+1)*resolution_ - cell_size_y_*resolution_/2;
      warn_cell.z = 0;
      potential_field_warn_.cells.push_back(warn_cell);
      }
    }
  }
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












