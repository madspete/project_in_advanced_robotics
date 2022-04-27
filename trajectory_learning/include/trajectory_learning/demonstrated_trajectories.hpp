#pragma once

#include <vector>

namespace trajectory_learning
{
struct DemonstratedTrajectories
{
public:
  using trajectory = std::vector<std::vector<double>>;

  DemonstratedTrajectories()
  {}

  void add_trajectory(trajectory traj)
  {
    deomenstrated_trajectories_.push_back(traj);
  }

  trajectory get_trajectory(unsigned int index)
  {
    if (index > deomenstrated_trajectories_.size() - 1)
    {
      throw("index out of bounds");
    }
    return deomenstrated_trajectories_[index];
  }

  void set_trajectory(unsigned int index, trajectory new_traj)
  {
    if (index > deomenstrated_trajectories_.size() - 1)
    {
      throw("index out of bounds");
    }
    deomenstrated_trajectories_[index] = new_traj;
  }

  unsigned int size()
  {
    return deomenstrated_trajectories_.size();
  }

  unsigned int total_size()
  {
    int total_size = 0;
    for (unsigned int i = 0; i < deomenstrated_trajectories_.size(); ++i)
    {
      total_size += deomenstrated_trajectories_[i].size();
    }
    return total_size;
  }

  trajectory get_all_as_one()
  {
    trajectory all_as_one;
    for (unsigned int i = 0; i < deomenstrated_trajectories_.size(); ++i)
    {
      for (unsigned int j = 0; j < deomenstrated_trajectories_[i].size(); ++j)
      {
        all_as_one.push_back(deomenstrated_trajectories_[i][j]);
      }
    }
    return all_as_one;
  }

private:
  std::vector<trajectory> deomenstrated_trajectories_;

};

}
