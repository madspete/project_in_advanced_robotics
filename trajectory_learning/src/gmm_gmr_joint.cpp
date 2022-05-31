// local files
#include "trajectory_learning/gmm_gmr_joint.hpp"
#include "trajectory_learning/demonstrated_trajectories.hpp"


// Ros dependencies
#include <ros/ros.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

// Other
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <cmath>

namespace trajectory_learning
{

GMMAndGMRJoint::GMMAndGMRJoint()
{
  trajectories_loaded_ = false;
  model_learned_ = false;
}


GMMAndGMRJoint::GMMAndGMRJoint(const char *trajectory_path)
{
  trajectories_loaded_ = false;
  model_learned_ = false;
  load_trajectories(trajectory_path);
  
}

void GMMAndGMRJoint::load_trajectories(const char *trajectory_path)
{
  path_ = trajectory_path;
  get_trajectory_filenames();
  align_trajectories();
  trajectories_loaded_ = true;
}

void GMMAndGMRJoint::load_model(std::string model_path)
{
  if (!gmm_model_.load(model_path))
  {
    std::cout << "failed to load model, correct path?" << std::endl;
    return;
  }
  model_learned_ = true;
}

void GMMAndGMRJoint::save_model(std::string model_path)
{
  if (!gmm_model_.save(model_path))
  {
    std::cout << "failed to save model, correct path?" << std::endl;
    return;
  }
}

void GMMAndGMRJoint::gmm_learn(int k, const arma::gmm_dist_mode& dist, const arma::gmm_seed_mode& seed, int km_iter, int em_iter, double err, bool print_mode)
{
  if (trajectories_loaded_ == false)
  {
    std::cout << "please go ahead and load trajectories first " << std::endl;
    return;
  }
  arma::uword d = 7;       // dimensionality
  std::vector<std::vector<double> > all_trajectories_as_one = aligned_demonstrated_trajectories_.get_all_as_one();
  arma::uword N = aligned_demonstrated_trajectories_.total_size();
  arma::mat data(d, N, arma::fill::zeros);

  // Write to a file here and plot with python
  std::ofstream myfile;
  myfile.open ("/home/mads/git/project_in_advanced_robotics/trajectory_learning/BIC/combined.csv");
  for (unsigned int i = 0; i < N; ++i)
  {
    myfile << all_trajectories_as_one[i][0] << " " << all_trajectories_as_one[i][1] << " " << all_trajectories_as_one[i][2] << " "
           << all_trajectories_as_one[i][3] << " " << all_trajectories_as_one[i][4] << " "  << all_trajectories_as_one[i][5] << " "
           << all_trajectories_as_one[i][6] << "\n";
  }

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

void GMMAndGMRJoint::gmr_calculation(arma::mat& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out)
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

void GMMAndGMRJoint::gmr_calculation_fast_gmm(std::vector<Vector>& target, std::vector<double> x, std::vector<unsigned int> in, std::vector<unsigned int> out)
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

double GMMAndGMRJoint::get_end_time()
{
  return end_time_;
}

void GMMAndGMRJoint::prepare_gmr_data(std::vector<unsigned int> in, std::vector<unsigned int> out,
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

void GMMAndGMRJoint::print_data()
{
  std::cout << "means " << gmm_model_.means << std::endl;
  std::cout << "prior " << gmm_model_.hefts << std::endl;
  std::cout << "cov " << gmm_model_.fcovs << std::endl;
}

void GMMAndGMRJoint::write_data_to_file(std::string filepath_means, std::string filepath_cov, std::string filepath_priors)
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

void GMMAndGMRJoint::get_trajectory_filenames()
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

std::vector<std::vector<double>> GMMAndGMRJoint::get_trajectory(std::string file)
{
  std::vector<std::vector<double>> trajectory;
  std::ifstream myfile;
  myfile.open(file);
  if (myfile.is_open())
  {
    std::string line;
    while (std::getline(myfile, line, '\n'))
    {
      std::stringstream ss(line);
      std::string data;
      std::vector<double> elems;
      while (getline(ss, data, ','))
      {
        elems.push_back(atof(data.c_str()));
      }
      trajectory.push_back(elems);
    }
  }
  else
  {
    throw("Failed to open file, have a look at the path");
  }
  return trajectory;
}

void GMMAndGMRJoint::align_trajectories()
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
    std::vector<std::vector<double>> longest_traj = demonstrated_trajectories.get_trajectory(longest_traj_idx);
    double step = 10.0 / longest_traj.size();
    // This is used to store the correct timing as the data points are collected with this frequenzy
    double ur_step = 0.002;

    std::vector<std::vector<JointSegment> > traj;
    for (unsigned int i = 0; i < demonstrated_trajectories.size(); ++i)
    {
      if (i == longest_traj_idx)
      {
        continue;
      }
      double temp_step = 10.0 / demonstrated_trajectories.get_trajectory(i).size();
      double cur_time = 0.0;
      std::vector<std::vector<double>> cur_traj = demonstrated_trajectories.get_trajectory(i);
      std::vector<JointSegment> jointSegments;
      for (unsigned int j = 0; j < cur_traj.size()-1; ++j)
      {
        std::vector<double> point;
        std::vector<double> next_point;
        point.push_back(cur_traj[j][0]);
        point.push_back(cur_traj[j][1]);
        point.push_back(cur_traj[j][2]);
        point.push_back(cur_traj[j][3]);
        point.push_back(cur_traj[j][4]);
        point.push_back(cur_traj[j][5]);
        double start_time = cur_time;

        next_point.push_back(cur_traj[j+1][0]);
        next_point.push_back(cur_traj[j+1][1]);
        next_point.push_back(cur_traj[j+1][2]);
        next_point.push_back(cur_traj[j+1][3]);
        next_point.push_back(cur_traj[j+1][4]);
        next_point.push_back(cur_traj[j+1][5]);
        double end_time = cur_time + temp_step;
        cur_time = end_time;
        JointSegment segment(point, next_point, start_time, end_time);
        jointSegments.push_back(segment);
      }
      traj.push_back(jointSegments);
    }
    double time = 0.0;
    std::vector<std::vector<double>> cur_timed_traj;
    for (unsigned int i = 0; i < longest_traj.size(); ++i)
    {
      std::vector<double> elems;
      elems = longest_traj[i];
      elems.push_back(time);
      cur_timed_traj.push_back(elems);
      time += ur_step;
    }
    end_time_ = time;
    aligned_demonstrated_trajectories_.add_trajectory(cur_timed_traj);

    std::vector<trajectory_msgs::JointTrajectory> trajectory_msgs;
    
    for (unsigned int i = 0; i < traj.size(); ++i)
    {
      time = 0.0;
      std::vector<std::vector<double>> interpolated_traj;
      for (double j = 0; j < 10-step; j = j + step)
      {
        unsigned int index = 0;
        for (unsigned int k = 0; k < traj[i].size(); ++k)
        {
          if (traj[i][k].start >= j && j <= traj[i][k].end)
          {
            index = k;
            break;
          }
        }
        std::vector<double> elems = traj[i][index].interpolate(j);
        elems.push_back(time);
        time += ur_step;
        interpolated_traj.push_back(elems);
      }
      aligned_demonstrated_trajectories_.add_trajectory(interpolated_traj);
    }
  }
  /*for (unsigned int i = 0; i < aligned_demonstrated_trajectories_.size(); ++i)
  {

    std::ofstream myfile;
    std::string filename = "/home/mads/git/project_in_advanced_robotics/traj" + std::to_string(i) + ".csv"; 
    myfile.open (filename.c_str());
    std::vector<std::vector<double>> traj_temp =  aligned_demonstrated_trajectories_.get_trajectory(i);
    for (int i = 0; i < traj_temp.size(); i++)
    {
      myfile << traj_temp[i][0] << "," << traj_temp[i][1] << "," << traj_temp[i][2] << "," << traj_temp[i][3] << "," << traj_temp[i][4] << "," << traj_temp[i][5] << "," << traj_temp[i][6] << "\n";
    }
    myfile.close();
  }*/
}

} // namespace
