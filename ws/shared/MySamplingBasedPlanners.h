#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

#include <functional>

template<size_t N>
std::vector<Eigen::Matrix<double, N, 1>> uniformSamples(size_t n, const Eigen::Matrix<double, N, 2>& boundaries, std::function<bool(const Eigen::Matrix<double, N, 1>&)> rejectFun); 

std::vector<Eigen::Matrix<double, 2, 1>> uniformSamples2D(size_t n, const amp::Environment2D& env); 

template<size_t N>
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

template<size_t N>
std::pair<std::vector<Eigen::Matrix<double, N, 1>>, std::vector<std::pair<size_t, size_t>>> generateRRT(
    Eigen::Matrix<double, N, 1> q_init,
    const std::function<Eigen::Matrix<double, N, 1>()>& sampleGenerator,
    const std::function<bool(const Eigen::Matrix<double, N, 1>&, const Eigen::Matrix<double, N, 1>&)>& pathReject,
    const std::function<Eigen::Matrix<double, N, 1>(const Eigen::Matrix<double, N, 1>&, const Eigen::Matrix<double, N, 1>&)>& newFromNearAndRand,
    const std::function<double(const Eigen::Matrix<double, N, 1>&, const Eigen::Matrix<double, N, 1>&)>& distanceFunction,
    const std::function<bool(const Eigen::Matrix<double, N, 1>&, size_t)>& finishCondition
);

std::pair<std::vector<Eigen::Vector2d>, std::vector<std::pair<size_t, size_t>>> generateRRT2D(
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

template <size_t N>
std::vector<Eigen::Matrix<double, N, 1>> uniformSamples(size_t n, const Eigen::Matrix<double, N, 2> &boundaries, std::function<bool(const Eigen::Matrix<double, N, 1> &)> rejectFun)
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

template <size_t N>
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

template <size_t N>
std::pair<std::vector<Eigen::Matrix<double, N, 1>>, std::vector<std::pair<size_t, size_t>>> generateRRT(
    Eigen::Matrix<double, N, 1> q_init, 
    const std::function<Eigen::Matrix<double, N, 1>()> &sampleGenerator, 
    const std::function<bool(const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, N, 1> &)> &pathReject, 
    const std::function<Eigen::Matrix<double, N, 1>(const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, N, 1> &)> &newFromNearAndRand, 
    const std::function<double(const Eigen::Matrix<double, N, 1> &, const Eigen::Matrix<double, N, 1> &)> &distanceFunction, 
    const std::function<bool(const Eigen::Matrix<double, N, 1> &, size_t)> &finishCondition)
{
    std::vector<std::pair<size_t, size_t>> connections;
    std::vector<Eigen::Matrix<double, N, 1>> points;
    points.push_back(q_init);

    size_t iter = 0;

    while (true)
    {
        auto rand = sampleGenerator();
        std::vector<double> distances;
        for (auto&& p : points) distances.push_back(distanceFunction(rand, p));
        auto closest_ind = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
        const auto& near = points.at(closest_ind);
        auto newPoint = newFromNearAndRand(near, rand);
        
        // std::cout << "rand=" << rand.transpose() << "  near="<<near.transpose()<< "  new="<<newPoint.transpose()<<std::endl; 
        if (pathReject(near, newPoint)) continue;

        // std::cout << "    Not rejected!" << std::endl;
        auto newPoint_ind = points.size();
        points.push_back(std::move(newPoint));

        connections.emplace_back(closest_ind, newPoint_ind);

        if (finishCondition(points.back(), iter)) break;

        iter++;
    }

    return {std::move(points), std::move(connections)};
}
