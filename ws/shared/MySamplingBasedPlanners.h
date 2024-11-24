#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

#include <functional>
#include <unordered_map>
#include <optional>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

std::vector<Eigen::Matrix<double, 2, 1>> uniformSamples2D(size_t n, const amp::Environment2D& env); 

template<int N>
std::vector<std::pair<size_t, size_t>> findConnectionsWithinR(double r, const std::vector<Eigen::Matrix<double, N, 1>>& samples, std::function<bool(const Eigen::Matrix<double, N, 1>&, const Eigen::Matrix<double, N, 1>&)> rejectFun);

std::vector<std::pair<size_t, size_t>> findConnectionsWithinR2D(double r, const std::vector<Eigen::Vector2d>& samples, const amp::Environment2D& env);

struct GraphBasedResult
{
    amp::Path2D path;
    std::shared_ptr<amp::Graph<double>> graph;
    std::function<Eigen::Vector2d(amp::Node)> nodePos;
    double runtime;
};

class MyPRM : public amp::PRM2D {
public:
    MyPRM(size_t n, double r, std::shared_ptr<amp::AStar> graphSearch, bool smoothing = false) : n(n), r(r), graphSearch(graphSearch), smoothing(smoothing) {}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

    GraphBasedResult detailedPlan(const amp::Problem2D& problem);

protected:
    size_t n;
    double r;
    std::shared_ptr<amp::AStar> graphSearch;
    bool smoothing; 
};

template <int N, int M>
struct RRTResult
{
    std::vector<Eigen::Matrix<double, N, 1>> points;
    std::unordered_map<size_t, size_t> connections;
    bool success;
    std::unordered_map<size_t, Eigen::Matrix<double, M, 1>> controls;

    amp::KinoPath getKinoPath(double dt)
    {
        amp::KinoPath result;
        result.valid = success;

        std::list<size_t> node_path;
        node_path.push_front(points.size()-1); 
        while (connections.count(node_path.front()) > 0)
        {
            node_path.push_front(connections.at(node_path.front())); 
        }

        for (auto&& node : node_path)
        {
            result.waypoints.push_back(points.at(node));
            if (controls.count(node) > 0)
            {
                result.controls.push_back(controls.at(node));
                result.durations.push_back(dt);
            }
        }

        return result;
    }
};

RRTResult<2,2> generateRRT2D(
    const amp::Problem2D& problem, double step, double finishRadius, double pGoal, size_t maxIters);

class MyRRT : public amp::GoalBiasRRT2D {
public:
    MyRRT(double step, double finishRadius, double pGoal, std::shared_ptr<amp::AStar> graphSearch, size_t maxIters) 
    : step(step), finishRadius(finishRadius), pGoal(pGoal), graphSearch(graphSearch), maxIters(maxIters) {}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

    GraphBasedResult detailedPlan(const amp::Problem2D& problem);

protected:
    double step;
    double finishRadius; 
    double pGoal;
    std::shared_ptr<amp::AStar> graphSearch;
    size_t maxIters;
};

template <int N>
std::vector<Eigen::Matrix<double, N, 1>> uniformSamples(size_t n, const Eigen::Matrix<double, N, 2> &boundaries, std::function<bool(const Eigen::Matrix<double, N, 1> &)> rejectFun = [](const Eigen::Matrix<double, N, 1> &){return false;})
{
    std::vector<Eigen::Matrix<double, N, 1>> result;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<std::uniform_real_distribution<>> dis;
    for (auto i = 0; i < N; i++)
    {
        dis.emplace_back(boundaries(i,0), boundaries(i,1));
    }

    while(result.size() < n)
    {
        auto& sample = result.emplace_back();
        for (auto i = 0; i < N; i++) sample[i] = dis.at(i)(gen);
        if (rejectFun(sample)) result.pop_back();
    }

    return result;
}

template <int N>
std::vector<std::pair<size_t, size_t>> findConnectionsWithinR(double r, const std::vector<Eigen::Matrix<double, N, 1>> &samples, std::function<bool(const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, N, 1> &)> rejectFun)
{
    std::vector<std::pair<size_t, size_t>> result;
    for (auto i = 0; i < samples.size(); i++)
    {
        const auto& sampleA = samples.at(i);
        for (auto j = i+1; j < samples.size(); j++)
        {
            const auto& sampleB = samples.at(j);
            if ((sampleA - sampleB).squaredNorm() > std::pow(r, 2)) continue;
            if (rejectFun(sampleA, sampleB)) continue;
            result.emplace_back(i, j);
        }
    }
    return result;
}

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

template <std::size_t Dim, typename BGPoint, typename EigenVector>
void set_point_from_eigen(BGPoint& bg_point, const EigenVector& eigen_vec) 
{
    bg::set<Dim>(bg_point, eigen_vec(Dim));
    if constexpr (Dim > 0)
    {
        set_point_from_eigen<Dim-1>(bg_point, eigen_vec);
    }
    
}

template <int N, int M = N>
RRTResult<N,M> generateRRT(
    Eigen::Matrix<double, N, 1> q_init, 
    const std::function<Eigen::Matrix<double, N, 1>()> &sampleGenerator, 
    const std::function<std::optional<Eigen::Matrix<double, N, 1>>(size_t, const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, M, 1> &)> &realizePath, 
    const std::function<Eigen::Matrix<double, M, 1>(const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, N, 1> &)> &newFromNearAndRand, 
    std::optional<Eigen::Matrix<double, N, 1>> distanceScale, 
    const std::function<bool(const Eigen::Matrix<double, N, 1> &)> &successCondition,
    size_t maxIters)
{

    auto scale = [&distanceScale](const Eigen::Matrix<double, N, 1> & val)
    {
        if (distanceScale)
        {
            return Eigen::Matrix<double, N, 1>(distanceScale->cwiseProduct(val));
        }
        return val;
    };

    using BGPointN = bg::model::point<float, N, bg::cs::cartesian>;
    auto toBGPoint = [&scale](const Eigen::Matrix<double, N, 1> & val)
    {
        BGPointN point;
        set_point_from_eigen<N-1>(point, scale(val));
        return point;
    };

    std::unordered_map<size_t, size_t> connections;
    std::unordered_map<size_t, size_t> path_length;
    std::unordered_map<size_t, Eigen::Matrix<double, M, 1>> controls;
    std::vector<Eigen::Matrix<double, N, 1>> points;

    using PointValue = std::pair<BGPointN, size_t>;
    bgi::rtree<PointValue, bgi::quadratic<16>> rtree;

    points.emplace_back(q_init);
    rtree.insert({toBGPoint(q_init), 0});
    path_length.emplace(0, 0);

    size_t iter = 0;

    while (true)
    {
        iter++;

        if (iter > maxIters)
        {
            std::cout << "FAIL " << std::endl;
            return {std::move(points), std::move(connections), false, std::move(controls)};
        }

        auto rand = sampleGenerator();

        BGPointN query_point = toBGPoint(rand);

        std::vector<PointValue> result;
        rtree.query(bgi::nearest(query_point, 1), std::back_inserter(result));

        auto closest_ind = result.at(0).second;
        const auto& near = points.at(closest_ind);
        
        Eigen::Matrix<double, M, 1> newPointControl = newFromNearAndRand(near, rand);

        std::optional<Eigen::Matrix<double, N, 1>> newPoint = realizePath(path_length.at(closest_ind), near, newPointControl);
        
        if (!newPoint) continue;
        // std::cout << "rand=" << rand.transpose() << "  near="<<near.transpose()<< "  new="<<newPoint->transpose()<< "  newPointControl="<<newPointControl.transpose()<<std::endl; 

        // std::cout << "    Not rejected!" << std::endl;
        auto newPoint_ind = points.size();
        points.push_back(std::move(*newPoint));

        connections.emplace(newPoint_ind, closest_ind);
        path_length.emplace(newPoint_ind, path_length.at(closest_ind)+1);
        controls.emplace(newPoint_ind, std::move(newPointControl));

        rtree.insert({toBGPoint(points.back()), newPoint_ind});

        if (successCondition(points.back()))
        {
            std::cout << "SUCCESS " << std::endl;
            return {std::move(points), std::move(connections), true, std::move(controls)};
        }
    // std::cout << "NOT YET "<< iter<< " " << points.back().transpose() << std::endl;

    }    
}

template<int N, typename T>
auto graphFromPointsAndConnections(
    const std::vector<Eigen::Matrix<double, N, 1>>& samples, 
    const T& connections
)
{
    auto graph = std::make_shared<amp::Graph<double>>();
    for (auto&&[a, b] : connections)
    {
        double distance = (samples.at(a) - samples.at(b)).norm();
        graph->connect(a, b, distance);
        graph->connect(b, a, distance);
    }
    return graph;
}

template<int N>
auto graphPathToWaypoints(
    const std::vector<Eigen::Matrix<double, N, 1>>& samples, 
    const std::list<amp::Node>& nodePath
)
{
    std::vector<Eigen::Matrix<double, N, 1>> waypoints;
    for (auto&& node : nodePath)
    {
        waypoints.push_back(samples.at(node));
    }
    return waypoints;
}