// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

int main(int argc, char** argv) 
{
    auto obs1 = std::vector<Eigen::Vector2d>{{
        Eigen::Vector2d{3,0}, Eigen::Vector2d{5,0}, Eigen::Vector2d{5, 2}, Eigen::Vector2d{3,2}
    }};
    auto obs2 = std::vector<Eigen::Vector2d>{{
        Eigen::Vector2d{7,3}, Eigen::Vector2d{9,3}, Eigen::Vector2d{9, 5}, Eigen::Vector2d{7,5}
    }};
    auto obs3 = std::vector<Eigen::Vector2d>{{
        Eigen::Vector2d{1,4}, Eigen::Vector2d{4,4}, Eigen::Vector2d{4,6}, Eigen::Vector2d{1,6}
    }};
    auto obs4 = std::vector<Eigen::Vector2d>{{
        Eigen::Vector2d{5,7}, Eigen::Vector2d{6,7}, Eigen::Vector2d{6,10}, Eigen::Vector2d{5,10}
    }};
    auto obss = std::vector{{amp::Polygon{obs1}, amp::Polygon{obs2}, amp::Polygon{obs3}, amp::Polygon{obs4}}};

    KinodynamicProblem2D problem = HW9::getCarProblemWS1();
    for (auto i = 0; i< 5; i++) problem.q_init[i] = 0;

    problem.q_goal.clear();
    for (auto pair : {std::pair<double, double>{8,9}, std::pair<double, double>{8,9}, std::pair<double, double>{4*3.1415/9., 5*3.1415/9.}, std::pair<double, double>{-1/20., 1/20.}, std::pair<double, double>{-99999, 9999999}})
        problem.q_goal.push_back(pair);

    // NOTE: We limit the speed state bounds to -1/20 to 1/20 to
    // help comply with the goal velocity limits (tighter than problem requires)
    problem.q_bounds.clear();
    for (auto pair : {std::pair<double, double>{-1, 11}, std::pair<double, double>{-1, 11}, std::pair<double, double>{-3.1415, 3.1415}, std::pair<double, double>{-1/20., 1/20.}, std::pair<double, double>{-3.1415/6, 3.1415/6}})
        problem.q_bounds.push_back(pair);

    // We make the control bounds for the acceleration tighter
    // to drive "slower" and thus control more easily
    problem.u_bounds.clear();
    for (auto pair : {std::pair<double, double>{-1./60, 1./60}, std::pair<double, double>{-3.1415/6, 3.1415/6}})
        problem.u_bounds.push_back(pair);

    problem.obstacles = obss;
    problem.dt_bounds = {0, 1};

    auto agent = std::make_shared<MySimpleCar>();
    // Further simplify by making the geometry of the agent a rectangle
    // this basically assumes that our uncertainty circle is actually
    // an uncertainty sequare of side length 0.5
    agent->agent_dim.length = 2 + 2*0.25;
    agent->agent_dim.width = 1 + 2*0.25;


    // Select problem, plan, check, and visualize
    for (auto i = 0; i < 3; i++)
    {
        MyKinoRRT kino_planner{20, 5000000};
        auto result = kino_planner.detailedPlan(problem, *agent); 
        Visualizer::makeFigure(problem, result.path, false); // Set to 'true' to render animation        
        // Visualizer::createAxes(*result.graph, result.nodePos);
    }
    Visualizer::showFigures();
}