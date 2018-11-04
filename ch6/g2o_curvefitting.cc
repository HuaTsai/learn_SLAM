#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
using namespace std;

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //CurveFittingVertex() {}

  virtual void setToOriginImpl() { _estimate << 0, 0, 0; }

  virtual void oplusImpl(const double *update) {
    _estimate += Eigen::Vector3d(update);
  }

  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}
};

class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  void computeError() {
    const CurveFittingVertex *v =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement -
                   std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }
  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}

 private:
  double _x;
};

int main(int argc, char **argv) {
  double a = 1.f, b = 2.f, c = 1.f;
  int N = 100;
  double w_sigma = 1.f;
  cv::RNG rng;
  double abc[3] = {0};

  cout << "generating data: " << endl;
  vector<double> x_data, y_data;
  for (int i = 0; i < N; ++i) {
    double x = i / 100.f;
    x_data.push_back(x);
    y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
    cout << x_data[i] << " " << y_data[i] << endl;
  }

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;

  auto linearSolver =
      g2o::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
  auto solver_ptr = g2o::make_unique<Block>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  // g2o::OptimizationAlgorithmGaussNewton *solver =
  //     new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  // g2o::OptimizationAlgorithmDogleg *solver =
  //     new g2o::OptimizationAlgorithmDogleg(solver_ptr);

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0, 0, 0));
  v->setId(0);
  optimizer.addVertex(v);

  for (int i = 0; i < N; ++i) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y_data[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 /
                         (w_sigma * w_sigma));
    optimizer.addEdge(edge);
  }

  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds." << endl;

  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;
}
