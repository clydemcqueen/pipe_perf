#include <cmath>
#include <vector>

double mean(const std::vector<double> &v)
{
  double sum = 0;
  for (const auto &m : v) {
    sum += m;
  }
  return sum / v.size();
}

double stdev(const std::vector<double> &v, double u)
{
  double squared = 0;
  for (const auto &m : v) {
    squared += ((m - u) * (m - u));
  }
  return sqrt(squared / v.size());
}

