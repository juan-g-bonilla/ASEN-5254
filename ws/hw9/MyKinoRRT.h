#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"
#include "GeometryHelper.h"
#include "MySamplingBasedPlanners.h" 

struct KinoResult
{
    amp::KinoPath path;
    double runtime;
    std::shared_ptr<amp::Graph<double>> graph = {};
    std::function<Eigen::Vector2d(amp::Node)> nodePos = [](amp::Node){return Eigen::Vector2d{};};
};


template<amp::AgentType agent_type>
struct agent_dim {};

template<>
struct agent_dim<amp::AgentType::SingleIntegrator> { 
    constexpr static size_t state = 2; 
    constexpr static size_t control = 2;
};

template<>
struct agent_dim<amp::AgentType::FirstOrderUnicycle> { 
    constexpr static size_t state = 3; 
    constexpr static size_t control = 2;
};

template<>
struct agent_dim<amp::AgentType::SecondOrderUnicycle> { 
    constexpr static size_t state = 5; 
    constexpr static size_t control = 2;
};

template<>
struct agent_dim<amp::AgentType::SimpleCar> { 
    constexpr static size_t state = 5; 
    constexpr static size_t control = 2;
};

template<amp::AgentType agent_type>
auto generateRRTKino(
    const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, size_t u_samples, size_t maxIters
);

class MyKinoRRT : public amp::KinodynamicRRT {
public:
    MyKinoRRT(int u_samples, int maxIters) : u_samples(u_samples), maxIters(maxIters) {}

    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

    KinoResult detailedPlan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent);

    int u_samples;
    int maxIters;

private:
    template<amp::AgentType agent_type>
    KinoResult detailedPlanImpl(const amp::KinodynamicProblem2D & problem, amp::DynamicAgent & agent)
    {
        auto tic = std::chrono::high_resolution_clock::now();

        auto rrt = generateRRTKino<agent_type>(problem, agent, u_samples, maxIters);
        auto path = rrt.getKinoPath(problem.dt_bounds.second);

        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> runtime = toc - tic;

        return {
            path, 
            runtime.count()*1000,
            graphFromPointsAndConnections(rrt.points, rrt.connections),
            [samples = std::move(rrt.points)](amp::Node node){
                return Eigen::Vector2d{samples.at(node).head(2)}; 
            }
        };
    }
};  


class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};


template<int N>
Eigen::Matrix<double, N, 2> vecOfPairsToMat(std::vector<std::pair<double, double>> vec)
{
    Eigen::Matrix<double, N, 2> mat;
    for (auto i = 0 ; i < N ; i++)
    {
        mat(i,0) = vec.at(i).first;
        mat(i,1) = vec.at(i).second;
    }
    return mat;
}

template<amp::AgentType agent_type>
auto generateRRTKino(
    const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, size_t u_samples, size_t maxIters
)
{
    constexpr static size_t N = agent_dim<agent_type>::state;
    constexpr static size_t M = agent_dim<agent_type>::control;

    constexpr size_t N_POINTS_PATH = 5;
    double pGoal = 0.05;
    double dt = problem.dt_bounds.second;

    Eigen::Matrix<double, N, 1> average_goal;
    for (auto i = 0 ; i < N ; i++)
    {
        average_goal(i) = (problem.q_goal.at(i).first + problem.q_goal.at(i).second) / 2;
    }

    auto obstacles = bufferMultiPolygon(
        convertPolygonsToMultiPolygon(problem.obstacles), 
        problem.isPointAgent ? 0.35 : 0.1
    );

    auto state_bounds = vecOfPairsToMat<N>(problem.q_bounds);
    auto control_bounds = vecOfPairsToMat<M>(problem.u_bounds);

    Eigen::Matrix<double, N, 1> scale;
    double cartesianScale = 1 / std::max( problem.x_max-problem.x_min, problem.y_max-problem.y_min );
    double angularScale = 1 / (2*3.1415);
    for (auto i = 0 ; i < N ; i++)
    {
        scale(i) = problem.isDimCartesian.at(i) ? cartesianScale : angularScale;
    }

    std::function<bool(const Eigen::Matrix<double, -1, N>&)> trajectoryIntersects;
    if (problem.isPointAgent)
    {
        trajectoryIntersects = [&obstacles](const Eigen::Matrix<double, -1, N>& trajectory)
        {
            BGLineString path;
            for (int i = 0; i < trajectory.rows(); ++i) {
                path.push_back(BGPoint{trajectory.row(i)(0), trajectory.row(i)(1)});
            }

            return boost::geometry::intersects(
                path, 
                obstacles
            );
        };
    }
    else
    {
        const double xDir[] = {0, problem.agent_dim.length, problem.agent_dim.length, 0};
        const double yDir[] = {problem.agent_dim.width/2, problem.agent_dim.width/2, -problem.agent_dim.width/2, -problem.agent_dim.width/2};

        BGPolygon untransformed_agent_poly;
        for (auto i = 0; i < 4; i++)
            bg::append(untransformed_agent_poly, BGPoint{xDir[i], yDir[i]});

        trajectoryIntersects = [&obstacles, untransformed_agent_poly = std::move(untransformed_agent_poly), &bounds = problem.q_bounds](const Eigen::Matrix<double, -1, N>& trajectory)
        {
            for (int i = 0; i < trajectory.rows(); ++i) {
                double x = trajectory.row(i)(0);
                double y = trajectory.row(i)(1);
                double theta = trajectory.row(i)(2);

                bg::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(-theta);
                bg::strategy::transform::translate_transformer<double, 2, 2> trans(x, y);

                BGPolygon rotated, translated;
                bg::transform(untransformed_agent_poly, rotated, rotate);
                bg::transform(rotated, translated, trans);

                // std::cout << "x=" << x << " y=" << y << " theta=" << theta*180/3.1415 <<std::endl; 
                for (auto&& point : translated.outer())
                {
                    // std::cout << "    x=" << point.x() << ", y=" << point.y() << std::endl;
                    if (point.x() < bounds.at(0).first || point.x() > bounds.at(0).second) return true;
                    if (point.y() < bounds.at(1).first || point.y() > bounds.at(1).second) return true;
                }

                if (boost::geometry::intersects(translated, obstacles))
                    return true;
            }

            return false;
        };
    }

    return generateRRT<N, M>(
        problem.q_init,
        [pGoal, &state_bounds, average_goal](){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(gen) < pGoal) return average_goal;

            return uniformSamples<N>(1, state_bounds).at(0);
        },
        [&trajectoryIntersects, &agent, &q_bounds = problem.q_bounds, dt, N_POINTS_PATH](size_t /*t*/, const Eigen::Matrix<double, N, 1>& start, const Eigen::Matrix<double, M, 1>& control) -> std::optional<Eigen::Matrix<double, N, 1>>
        {
            Eigen::VectorXd state = start;
            Eigen::VectorXd control_for_propagate = control;

            Eigen::Matrix<double, -1, N> trajectory;
            trajectory.resize(N_POINTS_PATH+1, Eigen::NoChange);
            trajectory.row(0) = state;

            for (auto i = 0; i < N_POINTS_PATH; i++)
            {
                agent.propagate(state, control_for_propagate, dt/N_POINTS_PATH);

                for (auto j = 0; j < N; j++)
                {
                    if (state(j) < q_bounds.at(j).first || state(j) > q_bounds.at(j).second)
                    {
                        // std::cerr << "ERR " << j << state(j) << " " << q_bounds.at(j).first << " " << q_bounds.at(j).second << std::endl;
                        return std::nullopt;
                    }
                }

                trajectory.row(i+1) = state;
            }

            if (trajectoryIntersects(trajectory))
            {
                return std::nullopt;
            }

            return Eigen::Matrix<double, N, 1>{state};
        },
        [&control_bounds = control_bounds, &agent, &scale, dt, u_samples](const Eigen::Matrix<double, N, 1>& nearPoint, const Eigen::Matrix<double, N, 1>& randPoint)
        {
            // return uniformSamples<M>(u_samples, control_bounds).at(0);
            auto random_controls = uniformSamples<M>(u_samples, control_bounds);

            Eigen::VectorXd best_control = random_controls.at(0);
            best_control.fill(-99);
            double best_distance = 999999999999;

            Eigen::Matrix<double, N, 1> scaledRandPoint = scale.cwiseProduct(randPoint);

            // std::cout << "TRYING TO GET TO " << randPoint.transpose() << std::endl;
            for (auto i = 0; i < u_samples ; i++)
            {
                Eigen::VectorXd state = nearPoint;
                Eigen::VectorXd control_for_propagate = random_controls.at(i);
                agent.propagate(state, control_for_propagate, dt);

                Eigen::Matrix<double, N, 1> scaledState = scale.cwiseProduct(state);
                double stateDistance = (scaledRandPoint - scaledState).squaredNorm();
                if (stateDistance < best_distance)
                {
                    best_distance = stateDistance;
                    best_control = random_controls.at(i);
                }
            }
            // std::cout << "BEST " << best_control.transpose() << std::endl;

            
            return Eigen::Matrix<double, M, 1>{best_control};
        },
        scale,
        [&goal_bounds = problem.q_goal](const Eigen::Matrix<double, N, 1>& state)
        {
            for (auto i = 0; i < N; i++)
            {
                if (state(i) < goal_bounds.at(i).first || state(i) > goal_bounds.at(i).second)
                {
                    return false;
                } 
            }
            return true;
        },
        maxIters
    );
}