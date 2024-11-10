// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "MyAStar.h"

using namespace amp;

struct BenchmarkResult
{
    std::string xlabel;
    std::string title;
    std::list<std::vector<double>> treesize;
    std::list<std::vector<double>> runtime;
    std::vector<double> successRatePercentage;
    std::vector<std::string> labels;

    void makeFigures()
    {
        Visualizer::makeBoxPlot(treesize, labels, title, xlabel, "Tree Number of Nodes");
        Visualizer::makeBoxPlot(runtime, labels, title, xlabel, "Runtime [ms]");
        Visualizer::makeBarGraph(successRatePercentage, labels, title, xlabel, "Success Rate [%]");

        auto compute_average = [](const std::vector<double>& vec) {
            return vec.empty() ? 0.0 : std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
        };

        auto get_averages = [&compute_average](const std::list<std::vector<double>>& values)
        {
            std::vector<double> averages;
            std::transform(values.begin(), values.end(), std::back_inserter(averages), compute_average);
            return averages;
        };
        
        Visualizer::makeBarGraph(get_averages(treesize), labels, title, xlabel, "Tree Number of Nodes");
        Visualizer::makeBarGraph(get_averages(runtime), labels, title, xlabel, "Runtime [ms]");
    }
};

template<typename T>
BenchmarkResult benchmark(std::string title, T& planner, std::vector<size_t> ms, size_t N = 100)
{
    BenchmarkResult result{"$m$", title};

    for (auto&& m : ms)
    {
        std::cout << "Running benchmark for m="<< m <<std::endl;

        std::stringstream ss;
        ss << m;
        result.labels.push_back(ss.str());

        auto& treesizes = result.treesize.emplace_back();
        auto& runtimes = result.runtime.emplace_back();
        auto& success = result.successRatePercentage.emplace_back(0);

        MultiAgentProblem2D problem = HW8::getWorkspace1(m);

        for (auto i = 0; i < N; i++)
        {
            if (i % 10 == 0)
                std::cout << "  "<< i<<"/"<<N <<std::endl;

            auto result = planner.detailedPlan(problem);
            if (result.path.valid)
            {
                treesizes.push_back(result.treesize);
                runtimes.push_back(result.runtime);
                success += 100. / N;
            }
        }
    }

    return result;
}

int main(int argc, char** argv) {
    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());

    if (false)
    {
        MultiAgentProblem2D problem = HW8::getWorkspace1(2);
        // Solve using a centralized approach
        MyCentralPlanner planner{0.5, 0.25, 0.05, 7500};
        MultiAgentPath2D path = planner.plan(problem);
        bool isValid = HW8::check(path, problem);
        // Visualizer::makeFigure(problem, path);
    }

    if (false)
    {
        MyCentralPlanner planner{0.5, 0.25, 0.05, 7500};
        benchmark("Excercise 1", planner, {1,2,3}).makeFigures();
    }
    
    if (true)
    {
        MultiAgentProblem2D problem = HW8::getWorkspace1(6);
        // Solve using a decentralized approach
        MyDecentralPlanner planner{0.5, 0.25, 0.05, 7500};
        MultiAgentPath2D path = planner.plan(problem);
        HW8::check(path, problem);
        // Visualizer::makeFigure(problem, path);
    }

    if (false)
    {
        MyDecentralPlanner planner{0.5, 0.25, 0.05, 7500};
        benchmark("Excercise 2", planner, {1,2,3,4,5,6}).makeFigures();
    }

    // Visualize and grade methods
    Visualizer::showFigures();
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("Juan.GarciaBonilla@colorado.edu", argc, argv, std::make_tuple(0.5, 0.25, 0.05, 7500), std::make_tuple(0.5, 0.25, 0.05, 7500));
    return 0;
}