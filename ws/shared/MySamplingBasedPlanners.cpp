# include "MySamplingBasedPlanners.h"
#include "GeometryHelper.h"
#include <random>
#include <chrono>

namespace
{
    template<size_t N>
    auto graphFromPointsAndConnections(
        const std::vector<Eigen::Matrix<double, N, 1>>& samples, 
        const std::vector<std::pair<size_t, size_t>>& connections
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

    template<size_t N>
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
}

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) 
{
    return detailedPlan(problem).path;
}

GraphBasedResult MyPRM::detailedPlan(const amp::Problem2D &problem)
{
    auto tic = std::chrono::high_resolution_clock::now();
    // std::cerr << "HERE 0" << std::endl;

    auto samples = uniformSamples2D(n, problem);
    amp::Node init_index = samples.size();
    samples.push_back(problem.q_init);
    amp::Node goal_index = samples.size();
    samples.push_back(problem.q_goal);
    // std::cerr << "HERE 1" << std::endl;

    auto connections = findConnectionsWithinR2D(r, samples, problem);
    // std::cerr << "HERE 2" << std::endl;
    auto graph = graphFromPointsAndConnections<2>(samples, connections);

    // Need to do this to ensure the init node has been initialized in the graph
    graph->connect(init_index, goal_index, 0);
    graph->disconnect(init_index, goal_index);
    // std::cerr << "HERE 3" << std::endl;

    amp::ShortestPathProblem graphProblem{graph, init_index, goal_index};
    auto nodePlan = graphSearch->search(graphProblem, {});
    // std::cerr << "HERE 4" << std::endl;

    amp::Path2D path;
    path.valid = nodePlan.success;
    if (path.valid)
    {
        path.waypoints = graphPathToWaypoints<2>(samples, nodePlan.node_path);
        // std::cerr << "HERE 5" << std::endl;

        // Re-run the algorithm, but this time only include the nodes that form the
        // previously found path and allow connections between points as large as possible
        if (smoothing)
        {
            const auto& samplesSmooth = path.waypoints; 
            auto connectionsSmooth = findConnectionsWithinR2D(std::numeric_limits<double>::infinity(), samplesSmooth, problem);
            auto graphSmooth = graphFromPointsAndConnections<2>(samplesSmooth, connectionsSmooth);

            amp::ShortestPathProblem graphSmoothProblem{graphSmooth, 0, amp::Node(samplesSmooth.size()-1)};
            auto nodePlanSmooth = graphSearch->search(graphSmoothProblem, {});
            path.waypoints = graphPathToWaypoints<2>(samplesSmooth, nodePlanSmooth.node_path);
        }
    }

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runtime = toc - tic;

    return GraphBasedResult{
        path, 
        graph, 
        [samples = std::move(samples)](amp::Node node){return samples.at(node);},
        runtime.count()*1000
    };
}

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    return detailedPlan(problem).path;
}

GraphBasedResult MyRRT::detailedPlan(const amp::Problem2D & problem)
{
    auto tic = std::chrono::high_resolution_clock::now();

    auto&&[samples, connections] = generateRRT2D(problem, step, finishRadius, pGoal, maxIters);

    // Add goal and connect it to the last added sample
    connections.emplace_back(samples.size()-1, samples.size());
    samples.push_back(problem.q_goal);

    // std::cerr << "HERE 2" << std::endl;
    auto graph = graphFromPointsAndConnections<2>(samples, connections);

    // std::cerr << "HERE 3" << std::endl;

    amp::ShortestPathProblem graphProblem{graph, 0, amp::Node(samples.size()-1)};
    auto nodePlan = graphSearch->search(graphProblem, {});
    // std::cerr << "HERE 4" << std::endl;

    amp::Path2D path;
    path.valid = nodePlan.success;
    if (path.valid)
    {
        path.waypoints = graphPathToWaypoints<2>(samples, nodePlan.node_path);
    }

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runtime = toc - tic;

    return GraphBasedResult{
        path, 
        graph, 
        [samples = std::move(samples)](amp::Node node){return samples.at(node);},
        runtime.count()*1000
    };
}

std::vector<Eigen::Matrix<double, 2, 1>> uniformSamples2D(size_t n, const amp::Environment2D &env)
{
    auto obstacles = convertPolygonsToMultiPolygon(env.obstacles);

    auto rejectFun = [&obstacles](const Eigen::Vector2d& point){
        return boost::geometry::within(eigenToBGPoint(point), obstacles);
    };

    return uniformSamples<2>(n, Eigen::Matrix2d{{env.x_min, env.x_max}, {env.y_min, env.y_max}}, rejectFun);
}

std::vector<std::pair<size_t,size_t>> findConnectionsWithinR2D(double r, const std::vector<Eigen::Vector2d>& samples, const amp::Environment2D & env)
{
    auto obstacles = convertPolygonsToMultiPolygon(env.obstacles);

    auto rejectFun = [&obstacles](const Eigen::Vector2d& sampleA, const Eigen::Vector2d& sampleB)
    {
        return boost::geometry::intersects(
            BGSegment{eigenToBGPoint(sampleA), eigenToBGPoint(sampleB)}, 
            obstacles
        );
    };

    return findConnectionsWithinR<2>(r, samples, rejectFun);
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<std::pair<size_t, size_t>>> generateRRT2D(
    const amp::Problem2D &problem, double step, double finishRadius, double pGoal, size_t maxIters)
{
    auto obstacles = convertPolygonsToMultiPolygon(problem.obstacles);

    return generateRRT<2>(
        problem.q_init,
        [pGoal, &problem](){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(gen) < pGoal) return problem.q_goal;

            return uniformSamples2D(1, problem).at(0);
        },
        [&obstacles](const Eigen::Vector2d& sampleA, const Eigen::Vector2d& sampleB)
        {
            return boost::geometry::intersects(
                BGSegment{eigenToBGPoint(sampleA), eigenToBGPoint(sampleB)}, 
                obstacles
            );
        },
        [step](const Eigen::Vector2d& nearPoint, const Eigen::Vector2d& randPoint)
        {
            Eigen::Vector2d direction = (randPoint-nearPoint).normalized();
            // std::cout << "    nearPoint " << nearPoint.transpose() << " randPoint=" << randPoint.transpose() << "  step=" << step << "  res=" <<(nearPoint + step*direction).transpose() << std::endl;
            return Eigen::Vector2d(nearPoint + step*direction);
        },
        [](const Eigen::Vector2d& sampleA, const Eigen::Vector2d& sampleB)
        {
            return (sampleA - sampleB).squaredNorm();
        },
        [finishRadius, maxIters, &q_goal = problem.q_goal](const Eigen::Vector2d& sample, size_t iters)
        {
            return (sample - q_goal).squaredNorm() < std::pow(finishRadius, 2) || iters > maxIters;
        }
    );
}
