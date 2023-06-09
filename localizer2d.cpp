#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}
      

//using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;
//using TreeNodeType = TreeNode_<ContainerType::iterator>;


/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO Is this not done already in the callback?
  if(!_map->initialized()){
  return;
  }
  

  // Create KD-Tree
  // TODO
  for(int col=0; col<_map->cols(); col++){
  
  	for(int row=0; row<_map->rows(); row++){
  	
  		if ((*_map)(row,col)==100){
  			//create Eigen::Vector2f with x and y to pass
  			//vector 2f to run it
  			auto point=cv::Point2i(row,col);
  			_obst_vect.push_back(_map->grid2world(point)) ;
  		}
  	}
  }
  TreeType obst_kd_tree(_obst_vect.begin(), _obst_vect.end(), 10);
  
  //_obst_tree_ptr=std::make_shared<TreeType> (obst_kd_tree);
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end(), 10);
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
  _laser_in_world = initial_pose_;
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO
  

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO
  
  ContainerType prediction;
  getPrediction(prediction); //Call without equal?
  
  ICP solver(prediction, scan_, 4);
  solver.run(100);

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO
  
  //_laser_in_world = _laser_in_world * solver.X();
  _laser_in_world = solver.X();
  
  
  //ROS_INFO("Scan [points:%ld,chi:%f,pos=(%f, %f)]", scan.size(), solver.chi(),
  //         _laser_in_world.translation().x(), _laser_in_world.translation().y());
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO
  TreeType::AnswerType neighbors;
  //prediction_=_obst_tree_ptr->fullSearch(neighbors,&_laser_in_world, 10); //_range_max?10 picked randomly
  
  //Why X() that is the identity?(so is defined in eigen_icp_2d.h)
  
  std::cerr << "setted initial pose2"<< std::endl;
  _obst_tree_ptr->fullSearch(neighbors,X().translation(), _range_max);
  std::cerr << "setted initial pose3"<< std::endl;
  
  for(auto point:neighbors){
  prediction_.push_back(*point);
  }
  //TODO CONTROL IF PREDICTION HAS POINTS
  std::cerr << "point"<< std::endl;
  
}
