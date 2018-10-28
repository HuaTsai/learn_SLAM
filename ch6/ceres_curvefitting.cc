#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : x_(x), y_(y) {}
  template <typename T>
  bool operator()(const T* const abc, T *residual) const {
    residual[0] = T(y_) - ceres::exp(abc[0] * T(x_) + abc[1] * T(x_) + abc[2]);
    return true;
  }
  const double x_, y_;
};

int main(int argc, char *argv[]) {
  double a = 1.f, b = 2.f, c = 1.f;
  int N = 100;
  double w_sigma = 1.f;
  cv::RNG rng;
  double abc[3] = {0};

  vector<double> x_data, y_data;
  for (int i = 0; i < N; ++i) {
    double x = i / 100.f;
    x_data.push_back(x);
    y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
    cout << x_data[i] << " " << y_data[i] << endl;
  }

  ceres::Problem problem;
  for (int i = 0; i < N; ++i) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,
      abc
    );
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds." << endl;

  cout << summary.BriefReport() << endl;
  cout << "estimated a, b, c = ";
  for (const auto &elem : abc) {
    cout << a << " ";
  }
  cout << endl;
}