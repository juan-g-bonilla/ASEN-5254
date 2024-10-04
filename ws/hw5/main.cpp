// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

amp::Problem2D getWorkspace1()
{
    std::vector<Eigen::Vector2d> wo1 = {{1, 1}, {2, 1}, {2, 5}, {1, 5}};
    std::vector<Eigen::Vector2d> wo2 = {{3, 3}, {3, 13}, {13, 13}, {13, 5}, {6, 5}, {6, 6}, {12, 6}, {12, 12}, {4, 12}, {4, 3}};
    std::reverse(wo2.begin(), wo2.end()); // make CCW
    // std::vector<Eigen::Vector2d> wo2 = {{3, 3}, {4, 3}, {4, 12}, {3, 12}};
    // std::vector<Eigen::Vector2d> wo3 = {{3, 12}, {12, 12}, {12, 13}, {3, 13}};
    // std::vector<Eigen::Vector2d> wo4 = {{12, 5}, {13, 5}, {13, 13}, {12, 13}};
    // std::vector<Eigen::Vector2d> wo5 = {{6, 5}, {12, 5}, {12, 6}, {6, 6}};

    amp::Problem2D ws;
    ws.q_init = {0,0};
    ws.q_goal = {10,10};
    ws.x_min = -1;
    ws.x_max = 14;
    ws.y_min = -1;
    ws.y_max = 14;
    ws.obstacles = {wo1, wo2};
    
    return ws;
}

amp::Problem2D getWorkspace2()
{
    std::vector<Eigen::Vector2d> wo1 = {
        {30, 0}, {30, 6}, {-6, 6}, {-6, -6}, {25, -6}, {25, 1}, {24, 1}, {24, -5}, {15, -5}, {15, 1}, {14, 1}, {14, -5}, {5, -5}, {5, 1}, {4, 1}, {4, -5}, {-5, -5}, {-5, 5}, 
        {9, 5}, {9, 0}, {10, 0}, {10, 5}, {19, 5}, {19, 0}, {20, 0}, {20, 5}, {29, 5}, {29, 0}
    };
    // std::vector<Eigen::Vector2d> wo1 = {{-6, -6}, {25, -6}, {25, -5}, {-6, -5}};
    // std::vector<Eigen::Vector2d> wo2 = {{-6, 5}, {30, 5}, {30, 6}, {-6, 6}};
    // std::vector<Eigen::Vector2d> wo3 = {{-6, -5}, {-5, -5}, {-5, 5}, {-6, 5}};
    // std::vector<Eigen::Vector2d> wo4 = {{4, -5}, {5, -5}, {5, 1}, {4, 1}};
    // std::vector<Eigen::Vector2d> wo5 = {{9, 0}, {10, 0}, {10, 5}, {9, 5}};
    // std::vector<Eigen::Vector2d> wo6 = {{14, -5}, {15, -5}, {15, 1}, {14, 1}};
    // std::vector<Eigen::Vector2d> wo7 = {{19, 0}, {20, 0}, {20, 5}, {19, 5}};
    // std::vector<Eigen::Vector2d> wo8 = {{24, -5}, {25, -5}, {25, 1}, {24, 1}};
    // std::vector<Eigen::Vector2d> wo9 = {{29, 0}, {30, 0}, {30, 5}, {29, 5}};

    amp::Problem2D ws;
    ws.q_init = {0,0};
    ws.q_goal = {35,0};
    ws.x_min = -7;
    ws.x_max = 36;
    ws.y_min = -7;
    ws.y_max = 7;
    ws.obstacles = {wo1};
    
    return ws;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded()); // amp::RNG::randiUnbounded()

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(0.01, 0.25, PotentialConfig{0.5, 10, 0.1, 3}, 50000);
    amp::Path2D path;
    
    // amp::Problem2D prob;
    // bool success = HW5::generateAndCheck(algo, path, prob);
    // Visualizer::makeFigure(prob, path);

    amp::Problem2D prob = getWorkspace2();
    path = algo.plan(prob);
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    amp::Visualizer::makeFigure(MyPotentialFunction{prob, algo.config}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 0.01, 0.25, PotentialConfig{0.5, 10, 0.1, 0.1}, 50000);
    return 0;
}