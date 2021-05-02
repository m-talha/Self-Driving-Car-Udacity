#include "classifier.h"
#include <math.h>
#include <string>
#include <vector>

using Eigen::ArrayXd;
using std::string;
using std::vector;

// Initializes GNB
GNB::GNB() {
  /**
   * TODO: Initialize GNB, if necessary. May depend on your implementation.
   */
  left_means = ArrayXd(4);
  left_means << 0,0,0,0;
  
  left_sds = ArrayXd(4);
  left_sds << 0,0,0,0;
    
  left_prior = 0;
    
  keep_means = ArrayXd(4);
  keep_means << 0,0,0,0;
  
  keep_sds = ArrayXd(4);
  keep_sds << 0,0,0,0;
  
  keep_prior = 0;
  
  right_means = ArrayXd(4);
  right_means << 0,0,0,0;
  
  right_sds = ArrayXd(4);
  right_sds << 0,0,0,0;
  
  right_prior = 0;
}

GNB::~GNB() {}

void GNB::train(const vector<vector<double>> &data, 
                const vector<string> &labels) {
  /**
   * Trains the classifier with N data points and labels.
   * @param data - array of N observations
   *   - Each observation is a tuple with 4 values: s, d, s_dot and d_dot.
   *   - Example : [[3.5, 0.1, 5.9, -0.02],
   *                [8.0, -0.3, 3.0, 2.2],
   *                 ...
   *                ]
   * @param labels - array of N labels
   *   - Each label is one of "left", "keep", or "right".
   *
   * TODO: Implement the training function for your classifier.
   */
  
  // For each label, compute ArrayXd of means, one for each data class 
  //   (s, d, s_dot, d_dot).
  // These will be used later to provide distributions for conditional 
  //   probabilites.
  // Means are stored in an ArrayXd of size 4.
  
  float left_size = 0;
  float keep_size = 0;
  float right_size = 0;
  
  // For each label, compute the numerators of the means for each class
  //   and the total number of data points given with that label.
  for (int i=0; i<labels.size(); ++i) {
    if (labels[i] == "left") {
      // conversion of data[i] to ArrayXd
      left_means += ArrayXd::Map(data[i].data(), data[i].size());
      left_size += 1;
    } else if (labels[i] == "keep") {
      keep_means += ArrayXd::Map(data[i].data(), data[i].size());
      keep_size += 1;
    } else if (labels[i] == "right") {
      right_means += ArrayXd::Map(data[i].data(), data[i].size());
      right_size += 1;
    }
  }

  // Compute the means. Each result is a ArrayXd of means 
  //   (4 means, one for each class)
  left_means = left_means/left_size;
  keep_means = keep_means/keep_size;
  right_means = right_means/right_size;

  // Compute std. dev. numerators for each class
  ArrayXd data_point = ArrayXd(4);
  for (auto i=0; i<labels.size(); ++i) {
    data_point = ArrayXd::Map(data[i].data(), data[i].size());
    if (labels[i] == "left") {
      left_sds += (data_point - left_means) * (data_point - left_means);
    } else if (labels[i] == "keep") {
      keep_sds += (data_point - keep_means) * (data_point - keep_means);
    } else if (labels[i] == "right") {
      right_sds += (data_point - right_means) * (data_point - right_means);
    }
  }
  
  // Compute std.dev
  left_sds = (left_sds / left_size).sqrt();
  keep_sds = (keep_sds / keep_size).sqrt();
  right_sds = (right_sds / right_size).sqrt();
  
  // Compute probability of each label
  left_prior = left_size / labels.size();
  keep_prior = keep_size / labels.size();
  right_prior = right_size / labels.size();
}

string GNB::predict(const vector<double> &sample) {
  /**
   * Once trained, this method is called and expected to return 
   *   a predicted behavior for the given observation.
   * @param sample - a 4 tuple with s, d, s_dot, d_dot.
   *   - Example: [3.5, 0.1, 8.5, -0.2]
   * @output A label representing the best guess of the classifier. Can
   *   be one of "left", "keep" or "right".
   *
   * TODO: Complete this function to return your classifier's prediction
   */
   
  // Compute the conditional probabilities and posterior numerator for each feature/label
  // combination.
  // P(s | left) * P(d | left) * P(s_dot | left) * P(d_dot | left)- same for"keep" & "right"
  double left_cp = 1.0;
  double keep_cp = 1.0;
  double right_cp = 1.0;
  
  // Computing conditional probabilities
  for (auto i=0; i<sample.size(); ++i) {
    left_cp *= (1.0 / sqrt(2 * M_PI * left_sds[i] * left_sds[i])) * \
        exp(-0.5 * pow(sample[i] - left_means[i], 2) / (left_sds[i] * left_sds[i]));
    keep_cp *= (1.0 / sqrt(2 * M_PI * keep_sds[i] * keep_sds[i])) * \
        exp(-0.5 * pow(sample[i] - keep_means[i], 2) / (keep_sds[i] * keep_sds[i]));
    right_cp *= (1.0 / sqrt(2 * M_PI * right_sds[i] * right_sds[i])) * \
        exp(-0.5 * pow(sample[i] - right_means[i], 2) / (right_sds[i] * right_sds[i]));
  }
  
  // multiply the conditional prob by prior to get posterior numerator
  left_cp *= left_prior;
  keep_cp *= keep_prior;
  right_cp *= right_prior;
  
  // Choose the largest posterior numerator as the best prediction
  ArrayXd posteriors(3);
  posteriors << left_cp, keep_cp, right_cp;
  double max_p = left_cp;
  int max_i = 0;
  for (auto i=0; i<posteriors.size(); ++i) {
    if (posteriors[i] > max_p) {
      max_i = i;
      max_p = posteriors[i];
    }
  }
  
  return this -> possible_labels[max_i];
}
