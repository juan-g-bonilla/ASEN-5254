#include "MyKinoRRT.h"


namespace {

void euler(Eigen::VectorXd& x, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& xdot, double dt)
{
    x += xdot(x) * dt;
}

void rk4(Eigen::VectorXd& x, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& xdot, double dt)
{
    Eigen::VectorXd k1 = xdot(x);
    Eigen::VectorXd k2 = xdot(x + 0.5 * dt * k1);
    Eigen::VectorXd k3 = xdot(x + 0.5 * dt * k2);
    Eigen::VectorXd k4 = xdot(x + dt * k3);

    x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // std::cerr << state.transpose() << " " << control.transpose();
    auto xdot = [&control](const Eigen::VectorXd&) {
        Eigen::VectorXd xdot(2);
        xdot << control[0], control[1];
        return xdot;
    };

    euler(state, xdot, dt);
    // std::cerr << " " << state.transpose() << std::endl;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    double r = 0.25;
    auto xdot = [&control, &state, r](const Eigen::VectorXd&) {
        Eigen::VectorXd xdot(3);
        xdot << 
            r * control[0] * std::cos(state[2]),
            r * control[0] * std::sin(state[2]),
            control[1];
        return xdot;
    };

    euler(state, xdot, dt);
}

void MySecondOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    double r = 0.25;
    auto xdot = [&control, &state, r](const Eigen::VectorXd&) {
        Eigen::VectorXd xdot;
        xdot.resize(5);
        xdot <<
            r*state[3]*std::cos(state[2]),
            r*state[3]*std::sin(state[2]),
            state[4],
            control[0],
            control[1]
        ;
        return xdot;
    };

    euler(state, xdot, dt);
}

void MySimpleCar::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    double L = 5;
    auto xdot = [&control, &state, L](const Eigen::VectorXd&) {
        Eigen::VectorXd xdot;
        xdot.resize(5);
        xdot <<
            state[3]*std::cos(state[2]),
            state[3]*std::sin(state[2]),
            state[3]/L*std::tan(state[4]),
            control[0],
            control[1]
        ;
        return xdot;
    };
    euler(state, xdot, dt);
}



amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    return detailedPlan(problem, agent).path;
}

KinoResult MyKinoRRT::detailedPlan(const amp::KinodynamicProblem2D & problem, amp::DynamicAgent & agent)
{
    if (problem.agent_type == amp::AgentType::SingleIntegrator)
    {
        return detailedPlanImpl<amp::AgentType::SingleIntegrator>(problem, agent);
    }
    else if (problem.agent_type == amp::AgentType::FirstOrderUnicycle)
    {
        return detailedPlanImpl<amp::AgentType::FirstOrderUnicycle>(problem, agent);
    }
    else if (problem.agent_type == amp::AgentType::SecondOrderUnicycle)
    {
        return detailedPlanImpl<amp::AgentType::SecondOrderUnicycle>(problem, agent);
    }
    else if (problem.agent_type == amp::AgentType::SimpleCar)
    {
        return detailedPlanImpl<amp::AgentType::SimpleCar>(problem, agent);
    }
    else
    {
        std::stringstream ss;
        ss << int(problem.agent_type);
        throw std::runtime_error(ss.str());
    }
}
