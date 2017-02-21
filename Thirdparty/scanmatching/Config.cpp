/*
 * config.cpp
 *
 *  Created on: 21/lug/2012
 *      Author: Mladen Mazuran
 */

#include <string>
#include "Config.h"

namespace config {

/* Configuration variables with default values */

std::string node_name           = "laser_odometry_node";
std::string laser_topic         = "scan";
std::string odometry_topic      = "laser_odometry";

std::string tf_parent_frame     = "laser_odometry";
std::string tf_child_frame      = "/base_link";
bool send_transform             = false;

int laser_queue_len             = 10;
int odometry_queue_len          = 1000;
double max_rate                 = 1000;
double sigma2                   = 0.02 * 0.02;

double icp_metric_l             = 3;
double icp_trim_ratio           = 0.9;
double icp_conv_error           = 1e-4;
int icp_max_iter                = 200;

NNEngineType nn_engine          = ANN;
ICPAlgorithmType icp_algorithm  = PointToLine;

} /* namespace config */
