#include <cob_collision_grid_map.h>

//Class FootprintLine
FootPrintLine::FootPrintLine(){  
}

FootPrintLine::~FootPrintLine(){
}


//Class PotentialFieldGridMap
PotentialFieldGridMap::PotentialFieldGridMap()
{ 
  costmap_received_=false;
  // grid_resolution_ = 0.035;
  // cell_size_x_ = map_width_ / resolution_;
  // cell_size_y_ = map_height_ / resolution_; 
  // grid_size_x_ = map_width_ / grid_resolution_;
  // grid_size_y_ = map_height_ / grid_resolution_; 
  grid_map_ = NULL;
  cost_map_ = NULL;
  //influence_radius_ = 1.5;
  // obstacle is detected when potential field value equals to 250
  //max_potential_value_ = 250;
  //step_value_ = max_potential_value_ / ((influence_radius_ / resolution_) - 1);
  //stop_threshold_ = 0.1;
  //in_forbidden_area_ = false;
  //in_warn_area_ = false;
  //if((cell_size_x_%2)==1) cell_size_x_odd_ = true; 
  //else cell_size_x_odd_ = false;
  //if((cell_size_y_%2)==1) cell_size_y_odd_ = true;
  //else cell_size_y_odd_ = false;
  //forbidden_value_ = 250;
  //forbidden_value_ = max_potential_value_ - (stop_threshold_ / resolution_) * step_value_;
  //warn_value_ = max_potential_value_ - ((influence_radius_ - 0.5) / resolution_) * step_value_;
}

// Destructor
PotentialFieldGridMap::~PotentialFieldGridMap(){
  delete[] cost_map_; 
}


void PotentialFieldGridMap::initial(){
  cell_size_x_ = map_width_ / resolution_;
  cell_size_y_ = map_height_ / resolution_;
  step_value_  = max_potential_value_ / ((influence_radius_ / resolution_) - 1);
  forbidden_value_ = max_potential_value_ - (stop_threshold_ / resolution_) * step_value_;
  warn_value_ = max_potential_value_ - ((influence_radius_ - 0.5) / resolution_) * step_value_;

  if((cell_size_x_%2)==1) cell_size_x_odd_ = true;
  else cell_size_x_odd_ = false;
  if((cell_size_y_%2)==1) cell_size_y_odd_ = true;
  else cell_size_y_odd_ = false;
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
              // update the map
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
  for(int i=0;i<cell_size_x_;i++){
    for(int j=0;j<cell_size_y_;j++ ){
      int index = getCellIndex(i,j);
      if(cost_map_[index]>=forbidden_value_){
      forbidden_cell.x = getCellCoordX(i);
      forbidden_cell.y = getCellCoordY(j);
      forbidden_cell.z = 0;
      potential_field_forbidden_.cells.push_back(forbidden_cell);
      //printf("(%2d,%2d,%3d)",i,j,cost_map_[index]);
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
  for(int i=0;i<cell_size_x_;i++){
    for(int j=0;j<cell_size_y_;j++){
      int index = getCellIndex(i,j);
      if((cost_map_[index]>=warn_value_)&&(cost_map_[index]<forbidden_value_)){
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
  if(cell_size_x_odd_) cell_x = (index_x+1)*resolution_ - cell_size_x_*resolution_/2;
  else cell_x = (index_x+1)*resolution_ - (cell_size_x_+1)*resolution_/2;  
  return cell_x;
}

double PotentialFieldGridMap::getCellCoordY(int index_y){
  double cell_y;
  if(cell_size_y_odd_) cell_y = (index_y+1)*resolution_ - cell_size_y_*resolution_/2;
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

bool PotentialFieldGridMap::collisionByRotation(double angular,const std::vector<geometry_msgs::Point>& footprint){
  getFootPrintCells(footprint);

   double angle;
   double radius;
   geometry_msgs::Point pt;
   std::vector<geometry_msgs::Point> new_footprint;
   new_footprint.clear();  
   //TODO check the footprint after a 4.5 and a  9 degree ratation.
   for(unsigned int i=0; i<footprint.size(); i++){
   angle = atan2(footprint[i].y,footprint[i].x);
   radius = sqrt(footprint[i].x*footprint[i].x + footprint[i].y*footprint[i].y);
   //printf("angle:%f,radius:%f\n",angle,radius);
   if(angular > 0)  angle = angle + M_PI/40; 
   else angle = angle - M_PI/40;
   pt.x = radius * cos(angle);
   pt.y = radius * sin(angle);
   pt.z = 0; 
   new_footprint.push_back(pt);
  }
    
   getFootPrintCells(new_footprint);
   if(checkCollision()) return true; 
   else {
   new_footprint.clear();  
   } 

   for(unsigned int i=0; i<footprint.size(); i++){
   angle = atan2(footprint[i].y,footprint[i].x);
   radius = sqrt(footprint[i].x*footprint[i].x + footprint[i].y*footprint[i].y);
   //printf("angle:%f,radius:%f\n",angle,radius);
   if(angular > 0)  angle = angle + M_PI/20; 
   else angle = angle - M_PI/20;
   pt.x = radius * cos(angle);
   pt.y = radius * sin(angle);
   pt.z = 0; 
   new_footprint.push_back(pt);
  }

  getFootPrintCells(new_footprint);
  return (checkCollision());
}


bool PotentialFieldGridMap::collisionPreCalculate(const geometry_msgs::Vector3& cmd_vel, const std::vector<geometry_msgs::Point>& footprint){
  //simulate a new footprint 
  double vel_angle = atan2(cmd_vel.y, cmd_vel.x);
  double x_offset = cos(vel_angle) * stop_threshold_ ; 
  double y_offset = sin(vel_angle) * stop_threshold_ ;
  //ROS_INFO("x_offset:%f,y_offset:%f",x_offset,y_offset);
  std::vector<geometry_msgs::Point> new_footprint;
  new_footprint.clear();
  geometry_msgs::Point pt;
  for(unsigned int i=0; i<footprint.size(); i++){
    pt.x = footprint[i].x + x_offset;
    pt.y = footprint[i].y + y_offset;
    pt.z = 0;
    new_footprint.push_back(pt);
   // ROS_INFO("new footprint %d is: %f,%f,%f\n",i,pt.x,pt.y,pt.z);
  }
  getFootPrintCells(new_footprint);
  return (checkCollision());
}


bool PotentialFieldGridMap::checkCollision(){
  int index_x,index_y;
  bool in_forbidden_area = false; 
  for(unsigned int i=0;i<footprint_line_.size();i++){ 
    if(cost_map_[getCellIndex(footprint_line_[i].index_x_start_, footprint_line_[i].index_y_start_)] == forbidden_value_) in_forbidden_area = true; 
    for(unsigned int j=0;j<footprint_line_[i].index_x_.size();j++){
      index_x = footprint_line_[i].index_x_[j];
      index_y = footprint_line_[i].index_y_[j];
      if(cost_map_[getCellIndex(index_x,index_y)] > forbidden_value_)
      in_forbidden_area = true;
      
    }          
  }
  return in_forbidden_area;
}


bool PotentialFieldGridMap::checkCollisionRotate(){
  int index_x,index_y;
  bool in_forbidden_area = false;
  for(unsigned int i=0;i<footprint_line_.size();i++){
    if(cost_map_[getCellIndex(footprint_line_[i].index_x_start_, footprint_line_[i].index_y_start_)] == forbidden_value_) in_forbidden_area = true;
    for(unsigned int j=0;j<footprint_line_[i].index_x_.size();j++){
      index_x = footprint_line_[i].index_x_[j];
      index_y = footprint_line_[i].index_y_[j];
      if(cost_map_[getCellIndex(index_x,index_y)] == max_potential_value_)
      in_forbidden_area = true;

    }
  }
  return in_forbidden_area;
}



//need to rewrite  for different footprints 
void PotentialFieldGridMap::setRelatedLine(const geometry_msgs::Vector3& cmd_vel){
  /*double start_angle = 0;
  double end_angle = 0;
  for(unsigned int i=0; i<footprint_line_.size(); i++){
    start_angle = atan2(footprint_line_[i].y_start_,footprint_line_[i].x_start_);
    end_angle = atan2(footprint_line_[i].y_end_,footprint_line_[i].x_end_); 
    if(0<start_angle<M_PI && 0<end_angle<M_PI && (footprint_line_[i].y_start_==footprint_line_[i].y_end_)) footprint_line_[i].related_direction[0] = true;
    else if(-M_PI<start_angle<0 && -M_PI<end_angle<-0 && (footprint_line_[i].y_start_==footprint_line_[i].y_end_)) footprint_line_[i].related_direction[4] = true;
    else if((M_PI/2<start_angle<M_PI||-M_PI<start_angle<-M_PI/2) && (M_PI/2<end_angle<M_PI||-M_PI<end_angle<M-M_PI/2) && (footprint_line_[i].x_start_==footprint_line_[i].x_end_)) footprint_line_[i].related_direction[6] = true;
    else if(-M_PI/2<start_angle<M_PI/2 && -M_PI/2<end_angle<M_PI/2 && (footprint_line_[i].x_start_==footprint_line_[i].x_end_)) footprint_line_[i].related_direction[2] = true;
    else{
      if(0<=start_angle<=M_PI/2 && 0<=start_angle<=M_PI/2)
    }
  }*/

  if(cmd_vel.x==0&&cmd_vel.y>0) footprint_line_[3].related_ = true;
  else if(cmd_vel.x==0&&cmd_vel.y<0) footprint_line_[1].related_ = true;
  else if(cmd_vel.x>0&&cmd_vel.y==0) footprint_line_[0].related_ = true;
  else if(cmd_vel.x<0&&cmd_vel.y==0) footprint_line_[2].related_ = true;
  else if(cmd_vel.x<0&&cmd_vel.y<0) { footprint_line_[1].related_ = true; footprint_line_[2].related_ = true;}
  else if(cmd_vel.x<0&&cmd_vel.y>0) { footprint_line_[2].related_ = true; footprint_line_[3].related_ = true;}
  else if(cmd_vel.x>0&&cmd_vel.y<0) { footprint_line_[0].related_ = true; footprint_line_[1].related_ = true;}
  else if(cmd_vel.x>0&&cmd_vel.y>0) { footprint_line_[3].related_ = true; footprint_line_[0].related_ = true;}
}


int PotentialFieldGridMap::findWarnValue(const geometry_msgs::Vector3& cmd_vel){
  for(unsigned int i=0;i<footprint_line_.size();i++){
    footprint_line_[i].related_ = false;
    //for(unsigned int j=0; j<8; j++){
    //  footprint_line_.related_direction_[j] = false;
    //}
  }
  setRelatedLine(cmd_vel);
  int max_warn_value = 0;
  int warn_value;
  unsigned int index_x,index_y;
  for(unsigned int i=0;i<footprint_line_.size();i++){
    if(footprint_line_[i].related_ == true){
      for(unsigned int j=0;j<footprint_line_[i].index_x_.size(); j++){
        index_x = footprint_line_[i].index_x_[j];
   	index_y = footprint_line_[i].index_y_[j];
        warn_value = cost_map_[getCellIndex(index_x,index_y)];
	if(warn_value > max_warn_value) max_warn_value = warn_value;
      }
    warn_value = cost_map_[getCellIndex(footprint_line_[i].index_x_start_,footprint_line_[i].index_y_start_)];
    if(warn_value > max_warn_value) max_warn_value = warn_value;
    warn_value = cost_map_[getCellIndex(footprint_line_[i].index_x_end_,footprint_line_[i].index_y_end_)];
    if(warn_value > max_warn_value) max_warn_value = warn_value;
    }
  //ROS_INFO("max warn_value:%d",max_warn_value);
  }
  return max_warn_value;
}

void PotentialFieldGridMap::findClosestLine(){
  //set the first line as the defalult cloesest line
  int cell_potential_field = 0;
  int cell_potential_field_max = 0;
  int index_x;
  int index_y;
  unsigned int index_x_max =0;
  unsigned int index_y_max =0;
 
  for(unsigned int i=0;i<footprint_line_.size();i++){
    for(unsigned int j=0;j<footprint_line_[i].index_x_.size();j++){
      index_x = footprint_line_[i].index_x_[j];
      index_y = footprint_line_[i].index_y_[j];
      cell_potential_field = cost_map_[getCellIndex(index_x,index_y)];
      if(cell_potential_field>cell_potential_field_max){
        cell_potential_field_max = cell_potential_field;
        closest_line_num_ = i;
        index_x_max = index_x;
        index_y_max = index_y;
      }
    }       
  }
   
  double closest_line_start_angle = atan2(footprint_line_[closest_line_num_].y_start_,footprint_line_[closest_line_num_].x_start_);
  double closest_line_end_angle = atan2(footprint_line_[closest_line_num_].y_end_,footprint_line_[closest_line_num_].x_end_);
  if(closest_line_end_angle<closest_line_start_angle) std::swap(closest_line_end_angle,closest_line_start_angle);
  if(footprint_line_[closest_line_num_].has_slope_){
    closest_line_orth_angle_ = atan(footprint_line_[closest_line_num_].slope_) + M_PI / 2.0f;
    if(closest_line_orth_angle_<closest_line_start_angle||closest_line_orth_angle_>closest_line_end_angle) 
    closest_line_orth_angle_ = - closest_line_orth_angle_;
    if(closest_line_orth_angle_<closest_line_start_angle||closest_line_orth_angle_>closest_line_end_angle) 
    ROS_WARN("closest line orth angle is out of rage");
  } 
 // ROS_INFO("the closest line number is %d",closest_line_num_);
 // ROS_INFO("max potential field value is %d on (%d,%d)",cell_potential_field_max,index_x_max,index_y_max);
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
     // ROS_INFO("ponit num %d (%f,%f)",i,fp_line.x_start_,fp_line.y_start_);
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
    footprint_line_[i].index_x_start_ = index_x_start;
    footprint_line_[i].index_y_start_ = index_y_start;
    index_x_end = (footprint_line_[i].x_end_ + map_w/2) / resolution_;
    index_y_end = (footprint_line_[i].y_end_ + map_h/2) / resolution_;
    footprint_line_[i].index_x_end_ = index_x_end;
    footprint_line_[i].index_y_end_ = index_y_end;
    // ROS_INFO("line %d (%d,%d)",i,index_x_start,index_y_start); 
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
   /*test 
   for(unsigned int j=0;j<footprint_line_[i].index_x_.size();j++){   
       printf("( %d,%d )",footprint_line_[i].index_x_[j],footprint_line_[i].index_y_[j]);
   }*/     
  }
}





