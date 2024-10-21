#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "MyCSConstructors.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(1);

    // You will need your 2-link manipulator from HW4
    MyManipulator2D manipulator{{1,1}}; // {1.33172, 0.918613}
    Problem2D point_problem = HW2::getWorkspace2();
    Problem2D manip_problem = HW6::getHW4Problem2();
    // Problem2D manip_problem = getWorkspace2();

    // Construct point-agent and manipulator cspace instances.
    std::size_t n_cells_point = std::ceil(std::max(point_problem.x_max - point_problem.x_min, point_problem.y_max - point_problem.y_min) / 0.25);
    std::size_t n_cells_manip = 500;
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_cells_point);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(n_cells_manip);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();

    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo(wf_algo, point_agent_ctor);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // Return a path for the point-agent and manipulator using c-space planning.
    // Path2D path = point_algo.plan(point_problem);
    // std::cout << path.length() << std::endl;
    // Visualizer::makeFigure(point_problem, path); // Visualize path in workspace
    // Visualizer::makeFigure(*point_algo.getCSpace(), path); // Visualize path in cspace

    // for (auto&& manip_problem : {HW6::getHW4Problem1(), HW6::getHW4Problem2(), HW6::getHW4Problem3()})
    // {
    //     ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    //     // std:: cout << trajectory.length() << std::endl;
    //     Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    //     Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory);   
    // }
    // std::cerr << manip_problem.q_init.transpose() << std::endl;
    // std::cerr << manip_problem.q_init.transpose() << std::endl;
    // ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    // HW6::checkLinkManipulatorPlan(trajectory, manipulator, manip_problem);
    // Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    // Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory);

    // For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, amp::SearchHeuristic{});
    result.print();
    HW6::checkGraphSearchResult(result, problem, heuristic);

    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("Juan.GarciaBonilla@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctor), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    // Visualizer::showFigures();
    return 0;
}