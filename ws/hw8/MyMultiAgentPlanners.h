#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

std::tuple<std::vector<Eigen::VectorXd>, std::unordered_map<size_t, size_t>, bool> generateRRT2DMulti(
    const amp::MultiAgentProblem2D& problem, double step, double finishRadius, double pGoal, size_t maxIters);

std::tuple<std::vector<Eigen::Vector2d>, std::unordered_map<size_t, size_t>, bool> generateRRT2DMultiPriority(
    const amp::MultiAgentProblem2D& problem, std::vector<amp::Path2D> previous_paths, double step, double finishRadius, double pGoal, size_t maxIters);

struct MultiResult
{
    amp::MultiAgentPath2D path;
    double runtime;
    size_t treesize;
};

class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
public:
    MyCentralPlanner(double step, double finishRadius, double pGoal, int maxIters) 
    : step(step), finishRadius(finishRadius), pGoal(pGoal), maxIters(maxIters) {}

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override {return detailedPlan(problem).path;}; 
    
    MultiResult detailedPlan(const amp::MultiAgentProblem2D& problem);

private:
    double step;
    double finishRadius; 
    double pGoal;
    int maxIters;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
public:
    MyDecentralPlanner(double step, double finishRadius, double pGoal, int maxIters) 
    : step(step), finishRadius(finishRadius), pGoal(pGoal), maxIters(maxIters) {}

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override {return detailedPlan(problem).path;}; 
    
    MultiResult detailedPlan(const amp::MultiAgentProblem2D& problem);

private:
    double step;
    double finishRadius; 
    double pGoal;
    int maxIters;
};


