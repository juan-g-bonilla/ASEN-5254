#include "MyMultiAgentPlanners.h"

#include "GeometryHelper.h"

namespace{
    template<typename T>
    Eigen::VectorXd concatenate(std::vector<T> vals, std::function<Eigen::Vector2d(const T&)> fn = [](const T& self){return self;})
    {
        Eigen::VectorXd result(vals.size()*2);
        for (size_t i = 0; i < vals.size(); ++i) {
            result.segment<2>(i * 2) = fn(vals.at(i));
        }
        return result;
    }
}

std::tuple<std::vector<Eigen::VectorXd>, std::unordered_map<size_t, size_t>, bool> generateRRT2DMulti(
    const amp::MultiAgentProblem2D& problem, double step, double finishRadius, double pGoal, size_t maxIters)
{
    auto nAgents = problem.numAgents();

    auto total_q_init = concatenate<amp::CircularAgentProperties>(problem.agent_properties, [](auto&& a){return a.q_init;});
    auto total_q_goal = concatenate<amp::CircularAgentProperties>(problem.agent_properties, [](auto&& a){return a.q_goal;});

    auto obstacles = convertPolygonsToMultiPolygon(problem.obstacles);

    return generateRRT<-1>(
        total_q_init,
        [pGoal, &problem, nAgents, total_q_goal, finishRadius](){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(gen) < pGoal) return total_q_goal;

            auto sample = concatenate(uniformSamples2D(nAgents, problem));
            for (size_t i = 0; i < nAgents; ++i)
                if ((sample.segment<2>(i * 2) - total_q_goal.segment<2>(i * 2)).squaredNorm() <= std::pow(finishRadius, 2))
                    sample.segment<2>(i * 2) = total_q_goal.segment<2>(i * 2);

            return sample;
        },
        [&obstacles, &agents = problem.agent_properties](size_t /*t*/, const Eigen::VectorXd& sampleA, const Eigen::VectorXd& sampleB)
        {
            std::vector<BGSegment> segments;
            for (size_t i = 0; i < agents.size(); ++i) {
                auto& seg = segments.emplace_back(
                    eigenToBGPoint(sampleA.segment<2>(i * 2)), 
                    eigenToBGPoint(sampleB.segment<2>(i * 2))
                );
            }

            for (size_t i = 0; i < agents.size(); ++i) 
            {
                if (boost::geometry::distance(segments.at(i), obstacles) < agents.at(i).radius)
                {
                    // std::cerr << "  Path of " << i <<" intersects with obstacles!" << std::endl;
                    return true;
                }
                    

                for (size_t j = i+1; j < agents.size(); ++j) 
                    if (boost::geometry::distance(segments.at(i), segments.at(j)) < (agents.at(j).radius + agents.at(i).radius))
                    {
                        // std::cerr << "  Path of " << i <<" intersects with path of " << j << "!" << std::endl;
                        return true;
                    }
            }

            return false;
        },
        [step](const Eigen::VectorXd& nearPoint, const Eigen::VectorXd& randPoint)
        {
            Eigen::VectorXd newPoint(nearPoint.size());
            for (size_t i = 0; i < nearPoint.size()/2; ++i) {
                Eigen::Vector2d direction = (randPoint.segment<2>(i * 2)-nearPoint.segment<2>(i * 2)).normalized();
                newPoint.segment<2>(i * 2) = nearPoint.segment<2>(i * 2) + step*direction;
            }
            return newPoint;
        },
        std::nullopt,
        [finishRadius, &q_goal = total_q_goal](const Eigen::VectorXd& sample)
        {
            for (size_t i = 0; i < sample.size()/2; ++i)
                if ((sample.segment<2>(i * 2) - q_goal.segment<2>(i * 2)).squaredNorm() > std::pow(finishRadius, 2))
                    return false;

            return true;
        },
        maxIters
    );
}

MultiResult MyCentralPlanner::detailedPlan(const amp::MultiAgentProblem2D &problem)
{
    // return MultiResult{
    //     {problem.numAgents()}, 
    //     0,
    //     0
    // };

    auto tic = std::chrono::high_resolution_clock::now();

    auto&&[samples, connections, success] = generateRRT2DMulti(problem, step, finishRadius, pGoal, maxIters);

    std::list<amp::Node> node_path;
    node_path.push_front(samples.size()-1); 
    while (connections.count(node_path.front()) > 0)
    {
        node_path.push_front(connections.at(node_path.front())); 
    }

    amp::MultiAgentPath2D path(problem.numAgents());
    path.valid = success;
    if (path.valid)
    {
        for (auto&& total_waypoint : graphPathToWaypoints<-1>(samples, node_path))
        {
            for (size_t i = 0; i < problem.numAgents(); ++i)
            {
                path.agent_paths.at(i).waypoints.emplace_back(
                    total_waypoint.segment<2>(i*2)
                );
            }
        }

        for (size_t i = 0; i < problem.numAgents(); ++i)
        {
            path.agent_paths.at(i).waypoints.emplace_back(
                problem.agent_properties.at(i).q_goal
            );
        }
    }

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runtime = toc - tic;

    return MultiResult{
        path, 
        runtime.count()*1000,
        samples.size()
    };
}

std::tuple<std::vector<Eigen::Vector2d>,std::unordered_map<size_t,size_t>,bool> generateRRT2DMultiPriority(const amp::MultiAgentProblem2D & problem, std::vector<amp::Path2D> previouspaths, double step, double finishRadius, double pGoal, size_t maxIters)
{
    auto obstacles = convertPolygonsToMultiPolygon(problem.obstacles);
    size_t m = previouspaths.size();
    auto q_goal  = problem.agent_properties.at(m).q_goal;
    auto q_init  = problem.agent_properties.at(m).q_init;

    return generateRRT<2>(
        q_init,
        [pGoal, &problem, q_goal](){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(gen) < pGoal) return q_goal;

            return uniformSamples2D(1, problem).at(0);
        },
        [&obstacles, m,  &agents = problem.agent_properties, &previouspaths](size_t t, const Eigen::Vector2d& sampleA, const Eigen::Vector2d& sampleB)
        {
            BGSegment segment{eigenToBGPoint(sampleA), eigenToBGPoint(sampleB)};

            for (auto i = 0; i < m; i++) {
                // std::cerr << "Reading at " << i << "/" << m << std::endl;

                auto t_a = std::min(previouspaths.at(i).waypoints.size()-1, t);
                auto t_b = std::min(previouspaths.at(i).waypoints.size()-1, t+1); 

                BGSegment segment_other{
                    eigenToBGPoint(previouspaths.at(i).waypoints.at(t_a)), 
                    eigenToBGPoint(previouspaths.at(i).waypoints.at(t_b))
                };

                if (boost::geometry::distance(segment, segment_other) < (agents.at(m).radius + agents.at(i).radius))
                {
                    // std::cerr << "  Path of " << i <<" intersects with obstacles!" << std::endl;
                    return true;
                }
            }

            return boost::geometry::distance(segment, obstacles) < agents.at(m).radius;
        },
        [step](const Eigen::Vector2d& nearPoint, const Eigen::Vector2d& randPoint)
        {
            Eigen::Vector2d direction = (randPoint-nearPoint).normalized();
            // std::cout << "    nearPoint " << nearPoint.transpose() << " randPoint=" << randPoint.transpose() << "  step=" << step << "  res=" <<(nearPoint + step*direction).transpose() << std::endl;
            return Eigen::Vector2d(nearPoint + step*direction);
        },
        std::nullopt,
        [finishRadius, q_goal](const Eigen::Vector2d& sample)
        {
            return (sample - q_goal).squaredNorm() < std::pow(finishRadius, 2);
        },
        maxIters
    );
}


MultiResult MyDecentralPlanner::detailedPlan(const amp::MultiAgentProblem2D &problem)
{
    auto tic = std::chrono::high_resolution_clock::now();

    amp::MultiAgentPath2D path;
    path.valid = true;
    for (auto i = 0; i < problem.numAgents(); i++)
    {
        // std::cerr << "At m=" << i << std::endl;
        auto&&[samples, connections, success] = generateRRT2DMultiPriority(problem, path.agent_paths, step, finishRadius, pGoal, maxIters);

        if (!success)
        {
            path.valid = false;
            break;
        }
        // std::cerr << "  Found path!" << std::endl;

        std::list<amp::Node> node_path;
        node_path.push_front(samples.size()-1); 
        while (connections.count(node_path.front()) > 0)
        {
            node_path.push_front(connections.at(node_path.front())); 
        }

        auto& waypoints = path.agent_paths.emplace_back().waypoints;
        waypoints = graphPathToWaypoints<2>(samples, node_path);
        waypoints.push_back(problem.agent_properties.at(i).q_goal);
        // for (auto&& w : waypoints) std::cerr << "  " << w.transpose() << std::endl;
    }

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runtime = toc - tic;

    return MultiResult{
        path, 
        runtime.count()*1000,
        0
    };
}
