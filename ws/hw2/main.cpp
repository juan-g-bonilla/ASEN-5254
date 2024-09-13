// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"
#include "main.h"

#include <algorithm> 

using namespace amp;

amp::Problem2D HW2::getWorkspace1()
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

amp::Problem2D HW2::getWorkspace2()
{
    std::vector<Eigen::Vector2d> wo1 = {{-6, -6}, {25, -6}, {25, -5}, {-6, -5}};
    std::vector<Eigen::Vector2d> wo2 = {{-6, 5}, {30, 5}, {30, 6}, {-6, 6}};
    std::vector<Eigen::Vector2d> wo3 = {{-6, -5}, {-5, -5}, {-5, 5}, {-6, 5}};
    std::vector<Eigen::Vector2d> wo4 = {{4, -5}, {5, -5}, {5, 1}, {4, 1}};
    std::vector<Eigen::Vector2d> wo5 = {{9, 0}, {10, 0}, {10, 5}, {9, 5}};
    std::vector<Eigen::Vector2d> wo6 = {{14, -5}, {15, -5}, {15, 1}, {14, 1}};
    std::vector<Eigen::Vector2d> wo7 = {{19, 0}, {20, 0}, {20, 5}, {19, 5}};
    std::vector<Eigen::Vector2d> wo8 = {{24, -5}, {25, -5}, {25, 1}, {24, 1}};
    std::vector<Eigen::Vector2d> wo9 = {{29, 0}, {30, 0}, {30, 5}, {29, 5}};

    amp::Problem2D ws;
    ws.q_init = {0,0};
    ws.q_goal = {35,0};
    ws.x_min = -7;
    ws.x_max = 36;
    ws.y_min = -7;
    ws.y_max = 7;
    ws.obstacles = {wo1, wo2, wo3, wo4, wo5, wo6, wo7, wo8, wo9};
    
    return ws;
}

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace1();

    // Use WO1 from Exercise 2
    // Problem2D problem = HW2::getWorkspace2();

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */

    // Declare your algorithm object 
    Bug1Algorithm algo;
    
    {
        // Call your algorithm on the problem
        amp::Path2D path = algo.plan(problem);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW2::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    // Let's get crazy and generate a random environment and test your algorithm
    // {
    //     amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
    //     amp::Problem2D random_prob; 
    //     std::vector<Eigen::Vector2d> collision_points;
    //     bool random_trial_success = HW2::generateAndCheck(algo, path, random_prob, collision_points);
    //     LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

    //     LOG("path length: " << path.length());

    //     // Visualize the path environment, and any collision points with obstacles
    //     Visualizer::makeFigure(random_prob, path, collision_points);
    // }

    Visualizer::showFigures();

    // HW2::grade(algo, "nonhuman.biologic@myspace.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}