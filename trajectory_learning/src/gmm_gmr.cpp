// local files
#include "trajectory_learning/gmm_gmr.hpp"
#include "trajectory_learning/demonstrated_trajectories.hpp"


// Ros dependencies
#include <ros/ros.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>

// Other
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <cmath>

namespace trajectory_learning
{

GMMAndGMR::GMMAndGMR()
{
  trajectories_loaded_ = false;
  model_learned_ = false;
}


GMMAndGMR::GMMAndGMR(const char *trajectory_path)
{
  trajectories_loaded_ = false;
  model_learned_ = false;
  load_trajectories(trajectory_path);
  
}

void GMMAndGMR::load_trajectories(const char *trajectory_path)
{
  path_ = trajectory_path;
  get_trajectory_filenames();
  align_trajectories();
  trajectories_loaded_ = true;
}

void GMMAndGMR::load_model(std::string model_path)
{
  if (!gmm_model_.load(model_path))
  {
    std::cout << "failed to load model, correct path?" << std::endl;
    return;
  }
  model_learned_ = true;
}

void GMMAndGMR::save_model(std::string model_path)
{
  if (!gmm_model_.save(model_path))
  {
    std::cout << "failed to save model, correct path?" << std::endl;
    return;
  }
}

void GMMAndGMR::gmm_learn(int k, const arma::gmm_dist_mode& dist, const arma::gmm_seed_mode& seed, int km_iter, int em_iter, double err, bool print_mode)
{
  if (trajectories_loaded_ == false)
  {
    std::cout << "please go ahead and load trajectories first " << std::endl;
    return;
  }
  arma::uword d = 8;       // dimensionality
  std::vector<std::vector<double> > all_trajectories_as_one = aligned_demonstrated_trajectories_.get_all_as_one();
  arma::uword N = aligned_demonstrated_trajectories_.total_size();
  arma::mat data(d, N, arma::fill::zeros);

  unsigned int i = 0; 
  while (i < N)
  { 
    arma::Col<double> col_vector(all_trajectories_as_one[i]);
    data.col(i) = col_vector;
    ++i;
  }

  bool status = gmm_model_.learn(data, k, dist, seed, km_iter, em_iter, err, print_mode);
  if (status == false)
  {
    std::cout << "Failed to learn, maybe have a look at the encoded data." << std::endl;
    return;
  }

  model_learned_ = true;
}

void GMMAndGMR::gmr_calculation(arma::mat& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out)
{
  // Create sub matrices 
  arma::mat input_means(in.size(), gmm_model_.n_gaus(), arma::fill::zeros);
  arma::mat output_means(out.size(), gmm_model_.n_gaus(), arma::fill::zeros);

  std::vector<arma::mat> sigma_out;
  std::vector<arma::mat> sigma_in;
  std::vector<arma::mat> sigma_in_out;
  std::vector<arma::mat> sigma_out_in;
  
  prepare_gmr_data(in, out, input_means, output_means, sigma_out, sigma_in, sigma_in_out, sigma_out_in);
  
  // Calculate gmm influence from each component
  std::vector<std::vector<double> > gmm_component_influence;
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    std::vector<double> temp_gmm_dist;
    for (unsigned int j = 0; j < x.size(); ++j)
    {
      double mean = input_means.at(i);
      // Only one element allocated, when the input data is one dimensional
      double sigma = sigma_in[i].at(0);
      // This seems to be the issue, not sure what is going wrong with this implementation
      temp_gmm_dist.push_back(gmm_model_.hefts[i] * arma::normpdf(x[j], mean, sigma));
    }
    gmm_component_influence.push_back(temp_gmm_dist);
  }

  std::vector<std::vector<double> > final_gmm;
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i )
  {
    std::vector<double> combined_gmm_influence(x.size(), 0.0);
    for (unsigned int j = 0; j < gmm_model_.n_gaus(); ++j )
    {
      for (unsigned int k = 0; k < x.size(); ++k)
      {
        combined_gmm_influence[k] += gmm_component_influence[j][k];
      }
    }
    std::vector<double> temp_final_gmm;
    for (unsigned int k = 0; k < x.size(); ++k)
    {
      temp_final_gmm.push_back(gmm_component_influence[i][k] / combined_gmm_influence[k]);
      //std::cout << temp_final_gmm[k] << std::endl;
    }
    final_gmm.push_back(temp_final_gmm);
  }

  arma::mat expected_means(out.size(), x.size(), arma::fill::zeros); 
  std::vector<double> temp_expected_means(x.size(), 0.0);
  std::vector<arma::mat> expected_sigmas;
  for (unsigned int i = 0; i < x.size(); ++i)
  {
    for (unsigned int j = 0; j < gmm_model_.n_gaus(); ++j)
    {
      arma::vec temp_mean = output_means.col(j) + sigma_out_in[j] * arma::inv(sigma_in[j])*(x[i]-input_means[j]);
      expected_means.col(i) = expected_means.col(i) + final_gmm[j][i]*temp_mean;
    }
  }
  target = expected_means;
}

void GMMAndGMR::gmr_calculation_fast_gmm(std::vector<Vector>& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out)
{
  // Prepare data
  std::vector<double> priors;
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    priors.push_back(gmm_model_.hefts[i]);
  }
  std::vector<double> means;
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    for (unsigned int j = 0; j < gmm_model_.n_dims(); ++j)
    {
      means.push_back(gmm_model_.means(j,i));
    }
  }
  std::vector<double> sigma;
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    arma::mat temp_sigma = gmm_model_.fcovs.slice(i);
    for(unsigned int j = 0; j < gmm_model_.n_dims(); ++j)
    {
      for (unsigned int k = 0; k < gmm_model_.n_dims(); ++k)
      {
        sigma.push_back(temp_sigma(j,k));
      }
    }
  }
  gmr_model_.reset(new Gaussians(gmm_model_.n_gaus(), gmm_model_.n_dims(), priors, means, sigma));
  
  gmr_model_->InitFastGMR(in[0], in[in.size()-1], out[0], out[out.size()-1]);

  for (unsigned int i = 0; i < x.size(); ++i)
  {
    Vector input(in.size());
    input[0] = x[i];
    Vector output(out.size());
    gmr_model_->Regression(input, output);
    target.push_back(output);
  }
}

void GMMAndGMR::prepare_gmr_data(std::vector<unsigned int> in, std::vector<unsigned int> out,
                                 arma::mat& input_means, arma::mat& output_means,
                                 std::vector<arma::mat>& sigma_out, std::vector<arma::mat>& sigma_in, 
                                 std::vector<arma::mat>& sigma_in_out, std::vector<arma::mat>& sigma_out_in)
{
  arma::cube covariance_matrices = gmm_model_.fcovs;
  arma::mat means = gmm_model_.means;

  for (unsigned int i = 0; i < in.size(); ++i)
  {
    input_means.row(i) = means.row(in[i]);

  }

  for (unsigned int i = 0; i < out.size(); ++i)
  {
    output_means.row(i) = means.row(out[i]);
  }

  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    arma::mat cov = covariance_matrices.slice(i);
    
    // Input matrix
    arma::mat temp_sigma_in(in.size(), in.size()); 
    for (unsigned int j = 0; j < in.size(); ++j)
    {
      for (unsigned int k = 0; k < in.size(); ++k)
      {
        temp_sigma_in(j, k) = cov(in[j], in[k]);
      }
    }
    sigma_in.push_back(temp_sigma_in);

    // output matrix
    arma::mat temp_sigma_out(out.size(), out.size()); 
    for (unsigned int j = 0; j < out.size(); ++j)
    {
      for (unsigned int k = 0; k < out.size(); ++k)
      {
        temp_sigma_out(j, k) = cov(out[j], out[k]);
      }
    }
    sigma_out.push_back(temp_sigma_out);

    // input output matrix
    arma::mat temp_sigma_in_out(in.size(), out.size()); 
    for (unsigned int j = 0; j < in.size(); ++j)
    {
      for (unsigned int k = 0; k < out.size(); ++k)
      {
        temp_sigma_in_out(j, k) = cov(in[j], out[k]);
      }
    }

    sigma_in_out.push_back(temp_sigma_in_out);

    // output input matrix
    arma::mat temp_sigma_out_in(out.size(), in.size()); 
    for (unsigned int j = 0; j < out.size(); ++j)
    {
      for (unsigned int k = 0; k < in.size(); ++k)
      {
        temp_sigma_out_in(j, k) = cov(out[j], in[k]);
      }
    }
    sigma_out_in.push_back(temp_sigma_out_in);
  }
}

void GMMAndGMR::print_data()
{
  std::cout << "means " << gmm_model_.means << std::endl;
  std::cout << "prior " << gmm_model_.hefts << std::endl;
  std::cout << "cov " << gmm_model_.fcovs << std::endl;
}

void GMMAndGMR::write_data_to_file(std::string filepath_means, std::string filepath_cov, std::string filepath_priors)
{
  std::ofstream myfile;
  myfile.open (filepath_means);
  myfile << gmm_model_.means;
  myfile.close();

  myfile.open (filepath_priors);
  myfile << gmm_model_.hefts;
  myfile.close();

  myfile.open (filepath_cov);
  for (unsigned int i = 0; i < gmm_model_.n_gaus(); ++i)
  {
    myfile << gmm_model_.fcovs.slice(0) << "\n";
  }
  
  myfile.close();
}

void GMMAndGMR::get_trajectory_filenames()
{
  struct dirent *entry;
  DIR *dir = opendir(path_);
  if (dir == NULL)
  {
    throw("the directory hasn't been found");
  }
  while ((entry = readdir(dir)) != NULL)
  {
    std::string file_name = (std::string) entry->d_name;

    if (file_name != "." && file_name != "..")
    {
      std::string directory = (std::string) path_;
      trajectory_filenames_.push_back(directory + '/' + file_name);
    }
  }
  closedir(dir);
}

std::vector<std::vector<double>> GMMAndGMR::get_trajectory(std::string file)
{
  std::vector<std::vector<double>> trajectory;
  std::vector<std::vector<double>> move_trajectory;
  std::ifstream myfile;
  myfile.open(file);
  if (myfile.is_open())
  {
    std::string line;
    while (std::getline(myfile, line, '\n'))
    {
      std::stringstream ss(line);
      unsigned int idx = 0;
      std::string data;
      std::vector<double> elems;
      while (getline(ss, data, ','))
      {
        if (idx <= 1)
        {
          idx += 1;
          continue;
        }
        idx += 1;
        elems.push_back(atof(data.c_str()));
      }
      trajectory.push_back(elems);
    }
    // This is used to detect when the robot starts moving, might be needed or might not be needed.
    double start_x = trajectory[0][0];
    double start_y = trajectory[0][1];
    double end_x = trajectory[trajectory.size()- 1][0];
    double end_y = trajectory[trajectory.size()- 1][1];
    unsigned int end_idx = 0;
    for (unsigned int i = trajectory.size() - 1; i >= 0; --i)
    {
      double dist = sqrt(pow(end_x - trajectory[i][0], 2)
                + pow(end_y - trajectory[i][1], 2) * 1.0);
      if (dist > 0.0001)
      {
        end_idx = i;
        break;
      }
    }
    bool trajectory_started = true;
    for (unsigned int i = 1; i < trajectory.size(); ++i)
    {
      double dist = sqrt(pow(start_x - trajectory[i][0], 2)
                + pow(start_y - trajectory[i][1], 2) * 1.0);
      if (dist > 0.0001)
      {
        trajectory_started = true;
      }
      if (trajectory_started == true && i <= end_idx)
      {
        move_trajectory.push_back(trajectory[i]);
      }
      if (i > end_idx)
      {
        break;
      }
    }
  }
  else
  {
    throw("Failed to open file, have a look at the path");
  }
  return trajectory;
}

void GMMAndGMR::align_trajectories()
{
  DemonstratedTrajectories demonstrated_trajectories;
  if (trajectory_filenames_.empty())
  {
    throw("No trajectories has been found, have you teached the robot");
  }
  else if (trajectory_filenames_.size() == 1)
  {
    std::vector<std::vector<double>> trajectory = get_trajectory(trajectory_filenames_[0]);
    aligned_demonstrated_trajectories_.add_trajectory(trajectory);
  }
  else // More than one trajectory, align them
  {
    for (unsigned int i = 0; i < trajectory_filenames_.size(); ++i)
    {
      std::vector<std::vector<double>> trajectory = get_trajectory(trajectory_filenames_[i]);
      demonstrated_trajectories.add_trajectory(trajectory);
    }
    double longest_traj_length = -1000;
    double longest_traj_idx = -1;
    for (unsigned int i = 0; i < demonstrated_trajectories.size(); ++i)
    {
      if (demonstrated_trajectories.get_trajectory(i).size() > longest_traj_length)
      {
        longest_traj_length = demonstrated_trajectories.get_trajectory(i).size();
        longest_traj_idx = i;
      }
    }
    std::vector<ros_controllers_cartesian::CartesianTrajectory> traj;

    std::vector<std::vector<double>> longest_traj = demonstrated_trajectories.get_trajectory(longest_traj_idx);
    double step = 10.0 / longest_traj.size();

    for (unsigned int i = 0; i < demonstrated_trajectories.size(); ++i)
    {
      if (i == longest_traj_idx)
      {
        continue;
      }
      double temp_step = 10.0 / demonstrated_trajectories.get_trajectory(i).size();
      double cur_time = 0.0;
      cartesian_control_msgs::CartesianTrajectory traj_msg;
      // maybe as input
      traj_msg.header.frame_id = "tool0";
      std::vector<std::vector<double>> cur_traj = demonstrated_trajectories.get_trajectory(i);
      for (unsigned int j = 0; j < cur_traj.size(); ++j)
      {
        cartesian_control_msgs::CartesianTrajectoryPoint point;
        ros::Duration dur(cur_time);
        point.time_from_start = dur;
        point.pose.position.x = cur_traj[j][0];
        point.pose.position.y = cur_traj[j][1];
        point.pose.position.z = cur_traj[j][2];
        point.pose.orientation.w = cur_traj[j][3];
        point.pose.orientation.x = cur_traj[j][4];
        point.pose.orientation.y = cur_traj[j][5];
        point.pose.orientation.z = cur_traj[j][6];
        traj_msg.points.push_back(point);
        cur_time += temp_step;
      }
      traj.push_back(traj_msg);
    }
    double time = 0.0;
    std::vector<std::vector<double>> cur_timed_traj;
    for (unsigned int i = 0; i < longest_traj.size(); ++i)
    {
      std::vector<double> elems;
      elems = longest_traj[i];
      elems.push_back(time);
      cur_timed_traj.push_back(elems);
      time += step;
    }
    aligned_demonstrated_trajectories_.add_trajectory(cur_timed_traj);
    for (unsigned int i = 0; i < traj.size(); ++i)
    {
      std::vector<std::vector<double>> cur_timed_traj;
      for (double j = 0.0; j < 10; j = j + step)
      {
        ros_controllers_cartesian::CartesianState desired;
        traj[i].sample(j, desired);
        std::vector<double> elems;
        elems.push_back(desired.p.x());
        elems.push_back(desired.p.y());
        elems.push_back(desired.p.z());
        elems.push_back(desired.q.w());
        elems.push_back(desired.q.x());
        elems.push_back(desired.q.y());
        elems.push_back(desired.q.z());
        elems.push_back(j);
        std::cout << j << std::endl;
        cur_timed_traj.push_back(elems);
      }
      aligned_demonstrated_trajectories_.add_trajectory(cur_timed_traj);
    }
  }
}

} // namespace
