/*
 * config.h
 *
 *  Created on: 21/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>

namespace config {

/* Nearest neighbour engine type: O(N^2) or k-d trees ANN implementation */
enum NNEngineType {
	ON2, ANN
};

/* ICP algorithm type */
enum ICPAlgorithmType {
	Classic, Metric, PointToLine
};

extern std::string node_name;           // Fixed gobal node name
extern std::string laser_topic;         // Laser input topic
extern std::string odometry_topic;      // Odometry output topic

extern std::string tf_parent_frame;     // Parent frame (tf)
extern std::string tf_child_frame;      // Child frame (tf)
extern bool send_transform;             // Whether to send tf transform or not

extern int laser_queue_len;             // Input topic message queue length
extern int odometry_queue_len;          // Output topic message queue length
extern double max_rate;                 // Maximum rate of scan matching
extern double sigma2;                   // Laser range variance

extern double icp_metric_l;             // L value for metric ICP
extern double icp_trim_ratio;           // Trim ratio for ICP
extern double icp_conv_error;           // Epsilon convergence error for ICP
extern int icp_max_iter;                // Maximum number of ICP iterations

extern NNEngineType nn_engine;          // Nearest neighbour engine choice
extern ICPAlgorithmType icp_algorithm;  // ICP algorithm choice

} /* namespace config */

#endif /* CONFIG_H_ */
