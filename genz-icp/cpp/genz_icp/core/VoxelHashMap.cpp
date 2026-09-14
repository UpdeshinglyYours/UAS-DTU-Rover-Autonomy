// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "VoxelHashMap.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

// This parameters are not intended to be changed, therefore we do not expose it
namespace {
static const std::array<Eigen::Vector3i, 27> voxel_shifts{
    Eigen::Vector3i(0, 0, 0),   Eigen::Vector3i(1, 0, 0),   Eigen::Vector3i(-1, 0, 0),
    Eigen::Vector3i(0, 1, 0),   Eigen::Vector3i(0, -1, 0),  Eigen::Vector3i(0, 0, 1),
    Eigen::Vector3i(0, 0, -1),  Eigen::Vector3i(1, 1, 0),   Eigen::Vector3i(1, -1, 0),
    Eigen::Vector3i(-1, 1, 0),  Eigen::Vector3i(-1, -1, 0), Eigen::Vector3i(1, 0, 1),
    Eigen::Vector3i(1, 0, -1),  Eigen::Vector3i(-1, 0, 1),  Eigen::Vector3i(-1, 0, -1),
    Eigen::Vector3i(0, 1, 1),   Eigen::Vector3i(0, 1, -1),  Eigen::Vector3i(0, -1, 1),
    Eigen::Vector3i(0, -1, -1), Eigen::Vector3i(1, 1, 1),   Eigen::Vector3i(1, 1, -1),
    Eigen::Vector3i(1, -1, 1),  Eigen::Vector3i(1, -1, -1), Eigen::Vector3i(-1, 1, 1),
    Eigen::Vector3i(-1, 1, -1), Eigen::Vector3i(-1, -1, 1), Eigen::Vector3i(-1, -1, -1)
};

static const size_t min_neighbors_for_normal_estimation = 5; 
}  // namespace

namespace genz_icp {

std::tuple<Eigen::Vector3d, size_t, Eigen::Matrix3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    double closest_squared_distance = std::numeric_limits<double>::max();
    size_t n_neighbors = 0;
    
    auto kx = static_cast<int>(query.x() / voxel_size_);
    auto ky = static_cast<int>(query.y() / voxel_size_);
    auto kz = static_cast<int>(query.z() / voxel_size_);

    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        Voxel voxel(kx+voxel_shift.x(), ky+voxel_shift.y(), kz+voxel_shift.z());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            const auto &points = search->second.points;
            std::for_each(points.cbegin(), points.cend(), [&](const auto &neighbor){
                double squared_distance = (neighbor - query).squaredNorm();
                if (squared_distance < closest_squared_distance){
                    closest_neighbor = neighbor;
                    closest_squared_distance = squared_distance;
                }
                centroid += neighbor;
                covariance += neighbor*neighbor.transpose();
                n_neighbors++;
            }
        );
        }
    });

    if (n_neighbors >= min_neighbors_for_normal_estimation){
        centroid /= static_cast<double>(n_neighbors);
        covariance /= static_cast<double>(n_neighbors);
        covariance -= centroid*centroid.transpose();
    }
    double closest_distance = (n_neighbors > 0) ? std::sqrt(closest_squared_distance) : std::numeric_limits<double>::max();
    return std::make_tuple(closest_neighbor, n_neighbors, covariance, closest_distance);
}


std::pair<bool, Eigen::Vector3d> VoxelHashMap::DeterminePlanarity(
    const Eigen::Matrix3d &covariance) const{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    // Compute the normal as the eigenvector of the smallest eigenvalue
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance, Eigen::ComputeEigenvectors);
    normal = solver.eigenvectors().col(0);
    normal = normal.normalized();

    // Planarity check
    const auto &eigenvalues = solver.eigenvalues();
    double lambda3 = eigenvalues[0];
    double lambda2 = eigenvalues[1];
    double lambda1 = eigenvalues[2];

    // Check if the surface is planar
    bool is_planar = (lambda3 / (lambda1 + lambda2 + lambda3)) < planarity_threshold_;

    return {is_planar, normal};
}

VoxelHashMap::Vector3dVectorTuple7 VoxelHashMap::GetCorrespondences(
    const Vector3dVector &points, double max_correspondance_distance) const {

    const size_t max_correspondences = points.size();
    Vector3dVector source;
    Vector3dVector target;
    Vector3dVector normals;
    Vector3dVector non_planar_source;
    Vector3dVector non_planar_target;

    source.resize(max_correspondences);
    target.resize(max_correspondences);
    normals.resize(max_correspondences);
    non_planar_source.resize(max_correspondences);
    non_planar_target.resize(max_correspondences);

    std::vector<uint8_t> planar_valid(max_correspondences, 0);
    std::vector<uint8_t> non_planar_valid(max_correspondences, 0);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t> &r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const Eigen::Vector3d &point = points[i];

            // Variables for the closest neighbor search & normal estimation
            const auto &[closest_neighbor, n_neighbors, covariance, closest_distance] = GetClosestNeighbor(point);
            if (closest_distance > max_correspondance_distance) continue;
            
            // Check if it is planar or non-planar
            if (n_neighbors >= min_neighbors_for_normal_estimation){
                const auto &[is_planar, normal] = DeterminePlanarity(covariance);

                if(is_planar){
                    source[i] = point;
                    target[i] = closest_neighbor;
                    normals[i] = normal;
                    planar_valid[i] = 1;
                } else {
                    non_planar_source[i] = point;
                    non_planar_target[i] = closest_neighbor;
                    non_planar_valid[i] = 1;
                }
            } 
            else {
                    non_planar_source[i] = point;
                    non_planar_target[i] = closest_neighbor;
                    non_planar_valid[i] = 1;
            }
            
        }
        });

    size_t planar_count = 0;
    size_t non_planar_count = 0;
    for (size_t i = 0; i < max_correspondences; ++i) {
        if (planar_valid[i]) {
            if (planar_count != i) {
                source[planar_count] = source[i];
                target[planar_count] = target[i];
                normals[planar_count] = normals[i];
            }
            ++planar_count;
        }
        if (non_planar_valid[i]) {
            if (non_planar_count != i) {
                non_planar_source[non_planar_count] = non_planar_source[i];
                non_planar_target[non_planar_count] = non_planar_target[i];
            }
            ++non_planar_count;
        }
    }

    source.resize(planar_count);
    target.resize(planar_count);
    normals.resize(planar_count);
    non_planar_source.resize(non_planar_count);
    non_planar_target.resize(non_planar_count);

    return std::make_tuple(
        std::move(source),
        std::move(target),
        std::move(normals),
        std::move(non_planar_source),
        std::move(non_planar_target),
        planar_count,
        non_planar_count);
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.emplace_back(point);
        }
    }
    return points;
}

size_t VoxelHashMap::PointCount() const {
    size_t count = 0;
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        count += voxel_block.points.size();
    }
    return count;
}

bool VoxelHashMap::HasNeighborWithin(const Eigen::Vector3d &query, double radius) const {
    if (map_.empty() || !query.allFinite() || !std::isfinite(radius) || radius < 0.0 ||
        !std::isfinite(voxel_size_) || voxel_size_ <= 0.0) {
        return false;
    }

    const double radius_squared = radius * radius;
    const int search_radius_voxels =
        std::max(1, static_cast<int>(std::ceil(radius / voxel_size_)));
    const auto kx = static_cast<int>(query.x() / voxel_size_);
    const auto ky = static_cast<int>(query.y() / voxel_size_);
    const auto kz = static_cast<int>(query.z() / voxel_size_);

    for (int dx = -search_radius_voxels; dx <= search_radius_voxels; ++dx) {
        for (int dy = -search_radius_voxels; dy <= search_radius_voxels; ++dy) {
            for (int dz = -search_radius_voxels; dz <= search_radius_voxels; ++dz) {
                const Voxel voxel(kx + dx, ky + dy, kz + dz);
                const auto search = map_.find(voxel);
                if (search == map_.end()) continue;

                for (const auto &neighbor : search->second.points) {
                    if ((neighbor - query).squaredNorm() <= radius_squared) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void VoxelHashMap::Update(const Vector3dVector &points, const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::Update(const Vector3dVector &points, const Sophus::SE3d &pose) {
    Vector3dVector points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        auto voxel = Voxel((point / voxel_size_).template cast<int>());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search.value();
            voxel_block.AddPoint(point);
        } else {
            map_.insert({voxel, VoxelBlock(point, max_points_per_voxel_)});
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = map_cleanup_radius_ * map_cleanup_radius_;
    for (auto it = map_.begin(); it != map_.end();) {
        if ((it->second.points.front() - origin).squaredNorm() > (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}
}  // namespace genz_icp
