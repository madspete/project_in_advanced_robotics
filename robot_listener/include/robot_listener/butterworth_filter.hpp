#pragma once

// This code was inspired by: https://github.com/nxsEdson/Butterworth-Filter
#include <iostream>
#include <fstream>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <complex>
#include <deque>

class ButterworthFilter
{
public:

  ButterworthFilter(){}

  std::vector<double> ComputeDenCoeffs(int FilterOrder, double Lcutoff, double Ucutoff);

  std::vector<double> TrinomialMultiply(int FilterOrder, std::vector<double> b, std::vector<double> c);

  std::vector<double> ComputeNumCoeffs(int FilterOrder, double Lcutoff, double Ucutoff, std::vector<double> DenC);

  std::vector<double> ComputeLP(int FilterOrder);

  std::vector<double> ComputeHP(int FilterOrder);

  std::vector<double> filter(std::vector<double>x, std::vector<double> coeff_b, std::vector<double> coeff_a);
  std::vector<double> filter(std::deque<double>x, std::vector<double> coeff_b, std::vector<double> coeff_a);

private:
};
