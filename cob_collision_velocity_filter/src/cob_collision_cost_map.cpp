#include <cob_collision_cost_map.h>

//Class FootprintLine
FootPrintLine::FootPrintLine(){  
}

FootPrintLine::~FootPrintLine(){
}


//Class PotentialFieldCostMap
PotentialFieldCostMap::PotentialFieldCostMap()
{ 
  costmap_received_=false;
  cost_map_ = NULL;
}

// Destructor
PotentialFieldCostMap::~PotentialFieldCostMap(){
  delete[] cost_map_; 
}


/****Core Functions****/

//generates potential field for the costmap cell linearly, the way to assign the value
//can be changed. e.g. replace  getRectangleCellValue() by getCircleValue()  
void PotentialFieldCostMap::cellPFLinearGeneration(){
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
              // change the way to assign the value 
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


//check if collision will happen when rotates, right now 4.5 and 9 degree rotation is used. larger degree can be added
//to avoid faseter rotation
bool PotentialFieldCostMap::collisionByRotation(double& angular,const std::vector<geometry_msgs::Point>& footprint){
  getFootPrintCells(footprint);

   double angle;
   double radius;
   geometry_msgs::Point pt;
   std::vector<geometry_msgs::Point> new_footprint;
   new_footprint.clear();  
   //check the footprint after a 4.5 and a  9 degree rotation. If collision exists in 4.5 degreee, the robot stops
   //if collision only exists in 9 degree, then the robot can only rotate in a very slow speed.


   for(unsigned int i=0; i<footprint.size(); i++){
     angle = atan2(footprint[i].y,footprint[i].x);
     radius = sqrt(footprint[i].x*footprint[i].x + footprint[i].y*footprint[i].y);
     if(angular > 0)  angle = angle + M_PI/20; 
     else angle = angle - M_PI/20;
     pt.x = radius * cos(angle);
     pt.y = radius * sin(angle);
     pt.z = 0;
     new_footprint.push_back(pt);
   }
   getFootPrintCells(new_footprint);
   if(checkCollision()) angular = 0.05; 

   for(unsigned int i=0; i<footprint.size(); i++){
     angle = atan2(footprint[i].y,footprint[i].x);
     radius = sqrt(footprint[i].x*footprint[i].x + footprint[i].y*footprint[i].y);
     if(angular > 0)  angle = angle + M_PI/40; 
     else angle = angle - M_PI/40;
     pt.x = radius * cos(angle);
     pt.y = radius * sin(angle);
     pt.z = 0; 
     new_footprint.push_back(pt);
   } 
   getFootPrintCells(new_footprint);
   //return true if collision exists
   if(checkCollision()) return true; 
   else return false;
}


//check if collision will happen when translates, the behavior is influenced by stop_threshold
bool PotentialFieldCostMap::collisionByTranslation(const geometry_msgs::Vector3& cmd_vel, const std::vector<geometry_msgs::Point>& footprint,double predicted_distance){
  //simulate a new footprint 
  double vel_angle = atan2(cmd_vel.y, cmd_vel.x);
  double x_offset = cos(vel_angle) * predicted_distance; 
  double y_offset = sin(vel_angle) * predicted_distance;
  std::vector<geometry_msgs::Point> new_footprint;
  new_footprint.clear();
  geometry_msgs::Point pt;
  for(unsigned int i=0; i<footprint.size(); i++){
    pt.x = footprint[i].x + x_offset;
    pt.y = footprint[i].y + y_offset;
    pt.z = 0;
    new_footprint.push_back(pt);
  }
  getFootPrintCells(new_footprint);
  return (checkCollision());
}


//reads the robot footprint and computes the cells occupied by the footprint in the potential field costmap
void PotentialFieldCostMap::getFootPrintCells(const std::vector<geometry_msgs::Point>& footprint){
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
  }
}

//find the biggest potential value in related footpirnt line 
int PotentialFieldCostMap::findWarnValue(const geometry_msgs::Vector3& cmd_vel){
  for(unsigned int i=0;i<footprint_line_.size();i++){
    footprint_line_[i].related_ = false;
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



/***help functions***/

//return true if collision exists  
bool PotentialFieldCostMap::checkCollision(){
  int index_x,index_y;
  for(unsigned int i=0;i<footprint_line_.size();i++){ 
    if(cost_map_[getCellIndex(footprint_line_[i].index_x_start_, footprint_line_[i].index_y_start_)] >= forbidden_value_) return true; 
    for(unsigned int j=0;j<footprint_line_[i].index_x_.size();j++){
      index_x = footprint_line_[i].index_x_[j];
      index_y = footprint_line_[i].index_y_[j];
      if(cost_map_[getCellIndex(index_x,index_y)] >= forbidden_value_) return true;
      
    }          
  }
  return false;
}


// initialize the parameters of the potential field costmap
void PotentialFieldCostMap::initial(){
  cell_size_x_ = map_width_ / resolution_;
  cell_size_y_ = map_height_ / resolution_;
  step_value_  = max_potential_value_ / ((influence_radius_ / resolution_) - 1);
  forbidden_value_ = max_potential_value_ - int(stop_threshold_ / resolution_) * step_value_;
  warn_value_ = max_potential_value_ - ((influence_radius_ *2/3) / resolution_) * step_value_;

  ROS_INFO("cell_size_x_= %d cell_size_y_= %d",cell_size_x_, cell_size_y_);
  ROS_INFO("max_potential_value is %d",max_potential_value_);
  ROS_INFO("step value is %d",step_value_);
  ROS_INFO("stop_threshold is %f",stop_threshold_);
  ROS_INFO("forbidden value is %d",forbidden_value_);
  ROS_INFO("warn value is %d",warn_value_);

  if((cell_size_x_%2)==1) cell_size_x_odd_ = true;
  else cell_size_x_odd_ = false;
  if((cell_size_y_%2)==1) cell_size_y_odd_ = true;
  else cell_size_y_odd_ = false;
}


// translate the coordinate of obstacle into the index of costmap array
void PotentialFieldCostMap::getCostMap(const nav_msgs::GridCells& last_costmap_received){
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
      // if the obstacle is inside the potential field costmap
      if (index!=-1)
      cost_map_[index] = max_potential_value_; 
    } 
   } 
}


//generate the msg for potential_field_forbidden_
void PotentialFieldCostMap::getPotentialForbidden(){
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
      }              
    }
  }
}


//generate the msg for potential_field_warn_
void PotentialFieldCostMap::getPotentialWarn(){
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


double PotentialFieldCostMap::getCellCoordX(int index_x){
  double cell_x;
  if(cell_size_x_odd_) cell_x = (index_x+1)*resolution_ - (cell_size_x_)*resolution_/2;  
  else cell_x = (index_x+1)*resolution_ - (cell_size_x_+1)*resolution_/2;
  return cell_x;
}

double PotentialFieldCostMap::getCellCoordY(int index_y){
  double cell_y;
  if(cell_size_y_odd_) cell_y = (index_y+1)*resolution_ - (cell_size_y_)*resolution_/2;  
  else cell_y = (index_y+1)*resolution_ - (cell_size_y_+1)*resolution_/2;
  return cell_y;
}


int PotentialFieldCostMap::getRectangleCellValue(int x, int y){
  int cell_value;
  if(fabs(x)>fabs(y)) cell_value = max_potential_value_ - fabs(x)*step_value_;
  else cell_value =  max_potential_value_ - fabs(y)*step_value_;
  return cell_value;
}


int PotentialFieldCostMap::getCircleCellValue(int x, int y){
  int cell_value;
  cell_value = max_potential_value_ - sqrt(x*x + y*y)*step_value_;
  return cell_value;
}


void PotentialFieldCostMap::initCostMap(){ 
  cost_map_ = new int[cell_size_x_*cell_size_y_];
  memset(cost_map_,0,cell_size_x_*cell_size_y_*sizeof(int));  
}


void PotentialFieldCostMap::deleteCostMap(){
  delete[] cost_map_; 
}



int PotentialFieldCostMap::getCellIndex(int cell_x,int cell_y){
  if((cell_x>=0)&&(cell_x<cell_size_x_)&&(cell_y>=0)&&(cell_y<cell_size_y_))
  return cell_y * cell_size_x_ + cell_x; 
  else{
    return -1;
  } 
}



void PotentialFieldCostMap::setRelatedLine(const geometry_msgs::Vector3& cmd_vel){
 
  double start_angle = 0;
  double end_angle = 0;
  for(unsigned int i=0; i<footprint_line_.size(); i++){
    start_angle = atan2(footprint_line_[i].y_start_,footprint_line_[i].x_start_);
    end_angle = atan2(footprint_line_[i].y_end_,footprint_line_[i].x_end_); 
 
    if(0<start_angle&&start_angle<M_PI/2&&M_PI/2<end_angle&&end_angle<M_PI){
      if(footprint_line_[i].y_start_ == footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 0;
      else if (footprint_line_[i].y_start_ < footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 8;
      else footprint_line_[i].related_type_ = 9;
    }

    else if(0<end_angle&&end_angle<M_PI/2&&M_PI/2<start_angle&&start_angle<M_PI){
      if(footprint_line_[i].y_start_ == footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 0;
      else if (footprint_line_[i].y_start_ < footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 9;
      else footprint_line_[i].related_type_ = 8;
    }

    else if(0<start_angle&&start_angle<M_PI/2&&(-M_PI/2)<end_angle&&end_angle<0){
      if(footprint_line_[i].x_start_ == footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 1;
      else if (footprint_line_[i].x_start_ < footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 11;
      else footprint_line_[i].related_type_ = 10;
    }

    else if(0<end_angle&&end_angle<M_PI/2&&(-M_PI/2)<start_angle&&start_angle<0){
      if(footprint_line_[i].x_start_ == footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 1;
      else if (footprint_line_[i].x_start_ < footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 10;
      else footprint_line_[i].related_type_ = 11;
    }

    else if((-M_PI)<start_angle&&start_angle<(-M_PI/2)&&(-M_PI/2)<end_angle&&end_angle<0){
      if(footprint_line_[i].y_start_ == footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 2;
      else if (footprint_line_[i].y_start_ < footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 13;
      else footprint_line_[i].related_type_ = 12;
    }
  
    else if((-M_PI)<end_angle&&end_angle<(-M_PI/2)&&(-M_PI/2)<start_angle&&start_angle<0){
      if(footprint_line_[i].y_start_ == footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 2;
      else if (footprint_line_[i].y_start_ < footprint_line_[i].y_end_) footprint_line_[i].related_type_ = 12;
      else footprint_line_[i].related_type_ = 13;
    }

    else if((M_PI/2)<start_angle&&start_angle<(M_PI)&&(-M_PI)<end_angle&&end_angle<(-M_PI/2)){
      if(footprint_line_[i].x_start_ == footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 3;
      else if (footprint_line_[i].x_start_ < footprint_line_[i].x_end_) footprint_line_[i].related_type_ =15 ;
      else footprint_line_[i].related_type_ = 14;
    }

    else if((M_PI/2)<end_angle&&end_angle<(M_PI)&&(-M_PI)<start_angle&&start_angle<(-M_PI/2)){
      if(footprint_line_[i].x_start_ == footprint_line_[i].x_end_) footprint_line_[i].related_type_ = 3;
      else if (footprint_line_[i].x_start_ < footprint_line_[i].x_end_) footprint_line_[i].related_type_ =14 ;
      else footprint_line_[i].related_type_ = 15;
    }

    else if(0<=end_angle&&end_angle<=M_PI/2&&0<=start_angle&&start_angle<=M_PI/2)             footprint_line_[i].related_type_ = 4;
    else if(M_PI/2<=end_angle&&end_angle<=M_PI&&M_PI/2<=start_angle&&start_angle<=M_PI)       footprint_line_[i].related_type_ = 7;
    else if((-M_PI/2)<=end_angle&&end_angle<=0&&(-M_PI/2)<=start_angle&&start_angle<=0)       footprint_line_[i].related_type_ = 5;
    else if((-M_PI)<=end_angle&&end_angle<=(-M_PI/2)&&(-M_PI)<=start_angle&&start_angle<=(-M_PI/2)) footprint_line_[i].related_type_ = 6;
    
   else {
     footprint_line_[i].related_type_ = -1;
     ROS_WARN("unknown type of footprint line");
   }

  }

  /*velocity type
  0 -- north 
  1 -- east
  2 -- south
  3 -- west
  4 -- north east
  5 -- south east
  6 -- south west
  7 -- north west*/
    
  int vel_type=-1;
  if(cmd_vel.x==0&&cmd_vel.y>0)      vel_type = 0;
  else if(cmd_vel.x==0&&cmd_vel.y<0) vel_type = 2;
  else if(cmd_vel.x>0&&cmd_vel.y==0) vel_type = 1;
  else if(cmd_vel.x<0&&cmd_vel.y==0) vel_type = 3;
  else if(cmd_vel.x<0&&cmd_vel.y<0)  vel_type = 6;
  else if(cmd_vel.x<0&&cmd_vel.y>0)  vel_type = 7;
  else if(cmd_vel.x>0&&cmd_vel.y<0)  vel_type = 5;
  else if(cmd_vel.x>0&&cmd_vel.y>0)  vel_type = 4;
  

  for(unsigned int i=0;i<footprint_line_.size();i++){
    switch(vel_type) {
      case 0:{ 
        if(footprint_line_[i].related_type_==0||footprint_line_[i].related_type_==4||footprint_line_[i].related_type_==7||footprint_line_[i].related_type_==8||footprint_line_[i].related_type_==9||footprint_line_[i].related_type_==11||footprint_line_[i].related_type_==15) footprint_line_[i].related_ = true;
        break;
      }
      case 1:{ 
        if(footprint_line_[i].related_type_==1||footprint_line_[i].related_type_==4||footprint_line_[i].related_type_==5||footprint_line_[i].related_type_==10||footprint_line_[i].related_type_==11||footprint_line_[i].related_type_==8||footprint_line_[i].related_type_==13) footprint_line_[i].related_ = true;
        break;
      }
      case 2:{ 
        if(footprint_line_[i].related_type_==2||footprint_line_[i].related_type_==5||footprint_line_[i].related_type_==6||footprint_line_[i].related_type_==12||footprint_line_[i].related_type_==13||footprint_line_[i].related_type_==10||footprint_line_[i].related_type_==14) footprint_line_[i].related_ = true;
        break;
      }
      case 3:{ 
        if(footprint_line_[i].related_type_==3||footprint_line_[i].related_type_==6||footprint_line_[i].related_type_==7||footprint_line_[i].related_type_==14||footprint_line_[i].related_type_==15||footprint_line_[i].related_type_==12||footprint_line_[i].related_type_==9) footprint_line_[i].related_ = true;
        break;
      }
      case 4:{ 
        if(footprint_line_[i].related_type_==0||footprint_line_[i].related_type_==1||footprint_line_[i].related_type_==4||footprint_line_[i].related_type_==8||footprint_line_[i].related_type_==9||footprint_line_[i].related_type_==10||footprint_line_[i].related_type_==11) footprint_line_[i].related_ = true;
        break;
      }
      case 5:{ 
        if(footprint_line_[i].related_type_==1||footprint_line_[i].related_type_==2||footprint_line_[i].related_type_==5||footprint_line_[i].related_type_==10||footprint_line_[i].related_type_==11||footprint_line_[i].related_type_==12||footprint_line_[i].related_type_==13) footprint_line_[i].related_ = true;
        break;
      }
      case 6:{ 
        if(footprint_line_[i].related_type_==2||footprint_line_[i].related_type_==3||footprint_line_[i].related_type_==6||footprint_line_[i].related_type_==12||footprint_line_[i].related_type_==13||footprint_line_[i].related_type_==14||footprint_line_[i].related_type_==15) footprint_line_[i].related_ = true;
        break;
      }
      case 7:{ 
        if(footprint_line_[i].related_type_==0||footprint_line_[i].related_type_==3||footprint_line_[i].related_type_==7||footprint_line_[i].related_type_==8||footprint_line_[i].related_type_==9||footprint_line_[i].related_type_==14||footprint_line_[i].related_type_==15) footprint_line_[i].related_ = true;
        break;
      }
      default:{
        ROS_INFO("footprint line do not has a related type");
        break;
      }
    }
  }
}

void PotentialFieldCostMap::findClosestLine(){
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
}







