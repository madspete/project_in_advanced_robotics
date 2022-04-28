#pragma once

// Internal
#include "trajectory_learning/demonstrated_trajectories.hpp"

// Linear algebra library, used for gmm
#include <armadillo>

// GMR library
#include <Gaussians.h>

// Other 
#include <vector>
#include <string>
#include <memory>

namespace trajectory_learning
{

class GMMAndGMR 
{
public:
  GMMAndGMR();

  GMMAndGMR(const char *trajectory_path);

  void load_trajectories(const char *trajectory_path);

  void load_model(std::string model_path);

  void save_model(std::string model_path);

  void gmm_learn(int k=5, const arma::gmm_dist_mode& dist =arma::maha_dist, const arma::gmm_seed_mode& seed = arma::random_spread, int km_iter=100, int em_iter=100, double err=1e-15, bool print_mode=true);

  // Currently only one dimensional input, since we reley on the time, this could be improved
  void gmr_calculation(arma::mat& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out);

  void gmr_calculation_fast_gmm(std::vector<Vector>& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out);

  void print_data();

  void write_data_to_file(std::string filepath_means, std::string filepath_cov, std::string filepath_priors);

private:
  void get_trajectory_filenames();
  
  std::vector<std::vector<double>> get_trajectory(std::string file);
  void align_trajectories();

  void prepare_gmr_data(std::vector<unsigned int> in, std::vector<unsigned int> out,
                        arma::mat& input_means, arma::mat& output_means, 
                        std::vector<arma::mat>& sigma_out, std::vector<arma::mat>& sigma_in, 
                        std::vector<arma::mat>& sigma_in_out, std::vector<arma::mat>& sigma_out_in);

  DemonstratedTrajectories aligned_demonstrated_trajectories_;
  std::vector<std::string> trajectory_filenames_;
  const char *path_;
  bool trajectories_loaded_;
  bool model_learned_;
  std::string model_path_;
  arma::gmm_full gmm_model_;
  std::unique_ptr<Gaussians> gmr_model_;

};
}

