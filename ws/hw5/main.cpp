// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

void savePotentialToFile(const PotentialConfig& config, const amp::Problem2D& problem, std::string filename) {

    MyPotentialFunction potFun{problem, config}; 

    // Open the file in write mode
    std::ofstream file(filename);

    auto N = 100;
    Eigen::VectorXd xGrid = Eigen::VectorXd::LinSpaced(N, problem.x_min, problem.x_max);
    Eigen::VectorXd yGrid = Eigen::VectorXd::LinSpaced(N, problem.y_min, problem.y_max);

    if (file.is_open()) {
        // Loop through each element in the matrix
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                file << potFun({xGrid(i), yGrid(j)}) << " ";
            }
            // New line after each row
            file << "\n";
        }
        
        file.close();
        std::cout << "Matrix saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}


amp::Problem2D getWorkspaceA()
{
    std::vector<Eigen::Vector2d> wo1 = {{4.5, 0.5}, {4.5, 1.5}, {3.5, 1.5}, {3.5, 0.5}};
    std::vector<Eigen::Vector2d> wo2 = {{7.5, -1.5}, {7.5, -0.5}, {6.5, -0.5}, {6.5, -1.5}};
    std::reverse(wo2.begin(), wo2.end()); // make CCW

    amp::Problem2D ws;
    ws.q_init = {0,0};
    ws.q_goal = {10,0};
    ws.x_min = -1;
    ws.x_max = 11;
    ws.y_min = -2;
    ws.y_max = 2;
    ws.obstacles = {wo1, wo2};
    
    return ws;
}

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
    amp::RNG::seed(0); // amp::RNG::randiUnbounded()

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(0.01, 0.25, PotentialConfig{0.5, 10, 0.1, 3}, 50000);
    
    // amp::Path2D path;
    // bool success = HW5::generateAndCheck(algo, path, prob);
    
    amp::Problem2D probA = getWorkspaceA();
    amp::Path2D pathA = algo.plan(probA);
    std::cout << "a) length " << pathA.length() << std::endl;
    savePotentialToFile(algo.config, probA, "potA_orig.txt");
    Visualizer::makeFigure(probA, pathA);

    amp::Problem2D prob1 = getWorkspace1();
    amp::Path2D path1 = algo.plan(prob1);
    std::cout << "b1) length " << path1.length() << std::endl;
    savePotentialToFile(algo.config, prob1, "pot1_orig.txt");
    Visualizer::makeFigure(prob1, path1);

    amp::Problem2D prob2 = getWorkspace2();
    amp::Path2D path2 = algo.plan(prob2);
    std::cout << "b2) length " << path2.length() << std::endl;
    savePotentialToFile(algo.config, prob2, "pot2_orig.txt");
    Visualizer::makeFigure(prob2, path2);

    // // Visualize your potential function
    // // amp::Visualizer::makeFigure(MyPotentialFunction{prob, algo.config}, prob, 50);
    // Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    // HW5::grade<MyGDAlgorithm>( "Juan.GarciaBonilla@colorado.edu", argc, argv, 0.01, 0.25, PotentialConfig{0.5, 10, 0.1, 3}, 50000);
    return 0;
}