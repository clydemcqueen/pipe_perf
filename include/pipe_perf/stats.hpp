#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

constexpr int QUEUE_SIZE = 10;
constexpr int NUM_MEASUREMENTS = 1000;

#undef RUN_PERF
#ifdef RUN_PERF
#define START_PERF()\
auto __start__ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg)\
auto __stop__ = std::chrono::high_resolution_clock::now();\
std::cout << msg << " " << std::chrono::duration_cast<std::chrono::microseconds>(__stop__ - __start__).count()\
  << " microseconds" << std::endl;
#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

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

void intro(const std::string& name)
{
  std::cout << "receiving messages" << std::endl;
  std::cout << name << " in μs over " << NUM_MEASUREMENTS << " measurements" << std::endl;
  std::cout << "mean, stdev, low, high" << std::endl;
}

void report(std::vector<double> &values)
{
  START_PERF()
  std::sort(values.begin(), values.end());
  auto u = mean(values);
  auto s = stdev(values, u);
  std::cout << std::fixed << std::setprecision(0)
            << u << ", " << s << ", " << values[0] << ", " << values[NUM_MEASUREMENTS - 1] << std::endl;
  values.clear();
  STOP_PERF("report")
}