#include "Registration.hpp"

#include <Eigen/Cholesky>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>
#include <iostream>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

constexpr double kHighUncertainty = 1e6;
constexpr double kMinInformationPivot = 1e-12;
constexpr double kRelativeRegularization = 1e-9;

Eigen::Matrix6d HighUncertaintyCovariance() {
    return Eigen::Matrix6d::Identity() * kHighUncertainty;
}

Eigen::Matrix6d CovarianceFromInformation(const Eigen::Matrix6d &information) {
    if (!information.allFinite()) {
        return HighUncertaintyCovariance();
    }

    Eigen::Matrix6d sym_information = 0.5 * (information + information.transpose());
    Eigen::LDLT<Eigen::Matrix6d> ldlt(sym_information);

    if (ldlt.info() != Eigen::Success ||
        (ldlt.vectorD().array() <= kMinInformationPivot).any()) {
        const double max_diagonal = sym_information.diagonal().cwiseAbs().maxCoeff();
        const double regularization =
            std::max(kMinInformationPivot, kRelativeRegularization * max_diagonal);

        sym_information.diagonal().array() += regularization;
        ldlt.compute(sym_information);
    }

    if (ldlt.info() != Eigen::Success ||
        (ldlt.vectorD().array() <= kMinInformationPivot).any()) {
        return HighUncertaintyCovariance();
    }

    Eigen::Matrix6d covariance = ldlt.solve(Eigen::Matrix6d::Identity());
    if (!covariance.allFinite()) {
        return HighUncertaintyCovariance();
    }

    return 0.5 * (covariance + covariance.transpose());
}

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

//Build the linear system for the GenZ-ICP
std::tuple<Eigen::Matrix6d, Eigen::Vector6d> BuildLinearSystem(
    const std::vector<Eigen::Vector3d> &src_planar,
    const std::vector<Eigen::Vector3d> &tgt_planar,
    const std::vector<Eigen::Vector3d> &normals,
    const std::vector<Eigen::Vector3d> &src_non_planar,
    const std::vector<Eigen::Vector3d> &tgt_non_planar,
    double kernel,
    double alpha) {

    struct ResultTuple {
        Eigen::Matrix6d JTJ;
        Eigen::Vector6d JTr;

        ResultTuple() : JTJ(Eigen::Matrix6d::Zero()), JTr(Eigen::Vector6d::Zero()) {}

        ResultTuple operator+(const ResultTuple &other) const {
            ResultTuple result;
            result.JTJ = JTJ + other.JTJ;
            result.JTr = JTr + other.JTr;
            return result;
        }
    };

    // Point-to-Plane Jacobian and Residual
    auto compute_jacobian_and_residual_planar = [&](auto i) {
        double r_planar = (src_planar[i] - tgt_planar[i]).dot(normals[i]); // residual
        Eigen::Matrix<double, 1, 6> J_planar; // Jacobian matrix
        J_planar.block<1, 3>(0, 0) = normals[i].transpose(); 
        J_planar.block<1, 3>(0, 3) = (src_planar[i].cross(normals[i])).transpose();
        return std::make_tuple(J_planar, r_planar);
    };

    // Point-to-Point Jacobian and Residual
    auto compute_jacobian_and_residual_non_planar = [&](auto i) {
        const Eigen::Vector3d r_non_planar = src_non_planar[i] - tgt_non_planar[i];
        Eigen::Matrix3_6d J_non_planar;
        J_non_planar.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_non_planar.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(src_non_planar[i]);
        return std::make_tuple(J_non_planar, r_non_planar);
    };

    double kernel_squared = kernel * kernel;
    auto compute = [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
        auto Weight = [&](double residual_squared) {
            return kernel_squared / square(kernel + residual_squared);
        };
        auto &[JTJ_private, JTr_private] = J;
        for (size_t i = r.begin(); i < r.end(); ++i) {
            if (i < src_planar.size()) { // Point-to-Plane
                const auto &[J_planar, r_planar] = compute_jacobian_and_residual_planar(i);
                double w_planar = Weight(r_planar * r_planar);
                JTJ_private.noalias() += alpha * J_planar.transpose() * w_planar * J_planar;
                JTr_private.noalias() += alpha * J_planar.transpose() * w_planar * r_planar;
            } else { // Point-to-Point
                size_t index = i - src_planar.size();
                if (index < src_non_planar.size()) {
                    const auto &[J_non_planar, r_non_planar] = compute_jacobian_and_residual_non_planar(index);
                    const double w_non_planar = Weight(r_non_planar.squaredNorm());
                    JTJ_private.noalias() += (1 - alpha) * J_non_planar.transpose() * w_non_planar * J_non_planar;
                    JTr_private.noalias() += (1 - alpha) * J_non_planar.transpose() * w_non_planar * r_non_planar;
                }
            }
        }
        return J;
    };


    size_t total_size = src_planar.size() + src_non_planar.size();
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, total_size),
        ResultTuple(),
        compute,
        [](const ResultTuple &a, const ResultTuple &b) {
            return a + b;
        });

    return std::make_tuple(JTJ, JTr);
}

void VisualizeStatus(size_t planar_count, size_t non_planar_count, double alpha) {
    const int bar_width = 52;
    const std::string planar_color = "\033[1;38;2;0;119;187m";
    const std::string non_planar_color = "\033[1;38;2;238;51;119m";
    const std::string alpha_color = "\033[1;32m";

    printf("\033[2J\033[1;1H"); // Clear terminal
    std::cout << "====================== GenZ-ICP ======================\n";
    std::cout << non_planar_color << "# of non-planar points: " << non_planar_count << ", ";
    std::cout << planar_color << "# of planar points: " << planar_count << "\033[0m\n";

    std::cout << "Unstructured  <-----  ";
    std::cout << alpha_color << "alpha: " << std::fixed << std::setprecision(3) << alpha << "\033[0m";
    std::cout << "  ----->  Structured\n";

    const int alpha_location = static_cast<int>(bar_width * alpha); 
    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i == alpha_location) {
            std::cout << "\033[1;32m█\033[0m"; 
        } else {
            std::cout << "-"; 
        }
    }
    std::cout << "]\n";
    std::cout.flush();
}
}  // namespace

namespace genz_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion)
    : max_num_iterations_(max_num_iteration), 
      convergence_criterion_(convergence_criterion) {}

std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, Eigen::Matrix<double, 6, 6>> Registration::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                                                                   const VoxelHashMap &voxel_map,
                                                                                                   const Sophus::SE3d &initial_guess,
                                                                                                   double max_correspondence_distance,
                                                                                                   double kernel) {
    
    // for visualization
    std::vector<Eigen::Vector3d> final_planar_points;
    std::vector<Eigen::Vector3d> final_non_planar_points;
    final_planar_points.clear();
    final_non_planar_points.clear();

    if (voxel_map.Empty()) return std::make_tuple(initial_guess, final_planar_points, final_non_planar_points, HighUncertaintyCovariance());

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    Eigen::Matrix6d final_information = Eigen::Matrix6d::Zero();
    bool have_final_information = false;

    // GenZ-ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        const auto &[src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, planar_count, non_planar_count] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        const size_t correspondence_count = planar_count + non_planar_count;
        if (correspondence_count == 0) {
            break;
        }

        double alpha = static_cast<double>(planar_count) / static_cast<double>(correspondence_count);
        const auto &[JTJ, JTr] = BuildLinearSystem(src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, kernel, alpha);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        if (!dx.allFinite()) {
            break;
        }

        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (dx.norm() < convergence_criterion_ || j == max_num_iterations_ - 1) {
            if (terminal_status_enabled_) {
                VisualizeStatus(planar_count, non_planar_count, alpha);
            }
            final_planar_points = src_planar;
            final_non_planar_points = src_non_planar;
            final_information = JTJ;
            have_final_information = true;
            break;
        }
    }

    const Eigen::Matrix6d final_covariance =
        have_final_information ? CovarianceFromInformation(final_information) : HighUncertaintyCovariance();

    // // Spit the final transformation
    return std::make_tuple(T_icp * initial_guess, final_planar_points, final_non_planar_points, final_covariance);
}

}  // namespace genz_icp
