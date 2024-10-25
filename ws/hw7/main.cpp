// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include "MyAStar.h"

using namespace amp;

struct BenchmarkResult
{
    std::string xlabel;
    std::list<std::vector<double>> pathlength;
    std::list<std::vector<double>> runtime;
    std::vector<double> successRatePercentage;
    std::vector<std::string> labels;
    std::string title;

    void makeFigures()
    {
        static const std::string ylabel_a = "Path Length";
        static const std::string ylabel_b = "Runtime [ms]";
        static const std::string ylabel_c = "Success Rate [%]";

        Visualizer::makeBoxPlot(pathlength, labels, title, xlabel, ylabel_a);
        Visualizer::makeBoxPlot(runtime, labels, title, xlabel, ylabel_b);
        Visualizer::makeBarGraph(successRatePercentage, labels, title, xlabel, ylabel_c);
    }
};

BenchmarkResult benchmarkPRM(const Problem2D& problem, std::vector<std::pair<size_t, double>> n_rs, std::shared_ptr<amp::AStar> graphSearch, bool smoothing, size_t N = 100)
{
    BenchmarkResult result{"$(n, r)$"};

    for (auto&&[n, r] : n_rs)
    {
        std::stringstream ss;
        ss << "(" << n << ", " << r << ")";
        result.labels.push_back(ss.str());

        auto& lengths = result.pathlength.emplace_back();
        auto& runtimes = result.runtime.emplace_back();
        auto& success = result.successRatePercentage.emplace_back(0);

        for (auto i = 0; i < N; i++)
        {
            MyPRM prm{n, r, graphSearch, smoothing};
            auto result = prm.detailedPlan(problem);
            if (result.path.valid)
            {
                lengths.push_back(result.path.length());
                runtimes.push_back(result.runtime);
                success += 100. / N;
            }
        }
    }

    return result;
}

BenchmarkResult benchmarkRRT(
    const Problem2D& problem, 
    std::vector<std::pair<double, double>> step_pGoals, 
    std::shared_ptr<amp::AStar> graphSearch, 
    size_t maxIters = 5000,
    size_t N = 100)
{
    BenchmarkResult result{"$(r, p_{goal})$"};

    for (auto&&[step, pGoal] : step_pGoals)
    {
        std::stringstream ss;
        ss << "(" << step << ", " << pGoal << ")";
        result.labels.push_back(ss.str());

        auto& lengths = result.pathlength.emplace_back();
        auto& runtimes = result.runtime.emplace_back();
        auto& success = result.successRatePercentage.emplace_back(0);

        for (auto i = 0; i < N; i++)
        {
            MyRRT rrt{step, step/2, pGoal, graphSearch, maxIters};
            auto result = rrt.detailedPlan(problem);
            if (result.path.valid)
            {
                lengths.push_back(result.path.length());
                runtimes.push_back(result.runtime);
                success += 100. / N;
            }
        }
    }

    return result;
}

int main(int argc, char** argv) {

    auto aStarAlgo = std::make_shared<MyAStarAlgo>();

    Problem2D problemHW5 = HW5::getWorkspace1();
    problemHW5.y_min = -3;
    problemHW5.y_max = 3;

    // Ex 1 a)
    if (false) {
        
        MyPRM prm{200, 1, aStarAlgo, false};
        auto result = prm.detailedPlan(problemHW5);
        Visualizer::makeFigure(problemHW5, result.path, *result.graph, result.nodePos);
        std::cout << "Ex 1 a) length: " << (result.path.valid ? result.path.length() : -1) << std::endl;
        // Visualizer::showFigures();

        std::vector<std::pair<size_t, double>> n_rs = {
            {200, .5},
            {200, 1},
            {200, 1.5},
            {200, 2},
            {500, .5},
            {500, 1},
            {500, 1.5},
            {500, 2},
        };

        auto res1 = benchmarkPRM(problemHW5, n_rs, aStarAlgo, false, 100);
        res1.title = "Exercise 1a), Without Smoothing";
        res1.makeFigures();
        // Visualizer::showFigures();

        auto res2 = benchmarkPRM(problemHW5, n_rs, aStarAlgo, true, 100);
        res2.title = "Exercise 1a), With Smoothing";
        res2.makeFigures();
        
        Visualizer::showFigures();
    }

    // Ex 1 b)
    if (false) {
        auto titles = std::vector{"HW2 W1", " HW2 W2"};
        const auto problems = std::vector{HW2::getWorkspace1(), HW2::getWorkspace2()};
        for (auto i = 0; i < titles.size(); i++)
        {
            auto&& title = titles.at(i);
            auto&& problem = problems.at(i);
            MyPRM prm{200, 2, aStarAlgo, false};
            auto result = prm.detailedPlan(problem);
            Visualizer::makeFigure(problem, result.path, *result.graph, result.nodePos);
            std::cout << "Ex 1 b) " << title << " length: " << (result.path.valid ? result.path.length() : -1) << std::endl;

            std::vector<std::pair<size_t, double>> n_rs = {
                {200, 1},
                {200, 2},
                {500, 1},
                {500, 2},
                {1000, 1},
                {1000, 2},
            };

            std::string benchTitle = "Exercise 1b), ";
            benchTitle += title;
            
            auto res1 = benchmarkPRM(problem, n_rs, aStarAlgo, false, 100);
            res1.title = benchTitle + ", Without Smoothing";
            res1.makeFigures();

            auto res2 = benchmarkPRM(problem, n_rs, aStarAlgo, true, 100);
            res2.title = benchTitle + ", With Smoothing";
            res2.makeFigures();
            
            Visualizer::showFigures();
        }
    }

    // Ex 2
    if (false) {
        auto titles = std::vector{"HW5", "HW2 W1", " HW2 W2"};
        const auto problems = std::vector{problemHW5, HW2::getWorkspace1(), HW2::getWorkspace2()};
        for (auto i = 0; i < titles.size(); i++)
        {
            auto&& title = titles.at(i);
            auto&& problem = problems.at(i);

            MyRRT rrt{0.5, 0.25,  0.05, aStarAlgo, 5000};
            auto result = rrt.detailedPlan(problem);
            Visualizer::makeFigure(problem, result.path, *result.graph, result.nodePos);
            std::cout << "Ex 2 " << title << " length: " << (result.path.valid ? result.path.length() : -1) << std::endl;

            std::vector<std::pair<double, double>> step_pGoals = {
                {0.5, 0.025},
                {0.5, 0.05},
                {0.5, 0.1},
                {1, 0.025},
                {1, 0.05},
                {1, 0.1},
            };
            
            auto res1 = benchmarkRRT(problem, step_pGoals, aStarAlgo, false, 100);
            res1.title = "Exercise 2), ";
            res1.title += title;
            res1.makeFigures();
            
            Visualizer::showFigures();
        }
    }
    


    // Generate a random problem and test RRT
    // MyRRT rrt;
    // Path2D path;
    // HW7::generateAndCheck(rrt, path, problem);
    // Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    

    // Grade method
    HW7::grade<MyPRM, MyRRT>("Juan.GarciaBonilla@colorado.edu", argc, argv, std::make_tuple(1000, 2, aStarAlgo, false), std::make_tuple(0.5, 0.25,  0.05, aStarAlgo, 5000));
    return 0;
}