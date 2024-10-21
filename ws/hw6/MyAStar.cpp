#include "MyAStar.h"

#include <unordered_map>
#include <queue>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    // std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;

    using cost = decltype(problem.graph)::element_type::EdgeType;
    std::unordered_map<amp::Node, cost> g, f;
    g[problem.init_node] = 0;
    f[problem.init_node] = 0;

    std::unordered_map<amp::Node, amp::Node> parent;

    std::set<amp::Node> open{problem.init_node};
    std::set<amp::Node> closed;

    size_t iters = 0;

    while( open.size() > 0 )
    {
        iters++;

        auto to_analyze_iter = std::min_element(open.begin(), open.end(), [&](double a, double b){return f[a] < f[b];});
        auto to_analyze = *to_analyze_iter;
        open.erase(to_analyze_iter);
        closed.emplace(to_analyze);
        
        // std::cout << to_analyze << std::endl;

        if (to_analyze == problem.goal_node)
        {
            break;
        }

        auto&& children = problem.graph->children(to_analyze);
        auto&& edge_costs = problem.graph->outgoingEdges(to_analyze);
        for (auto i = 0; i < children.size(); i++)
        {
            auto&& child = children.at(i);
            auto&& edge_cost = edge_costs.at(i);

            if (closed.count(child) > 0) continue;

            // std::cout << "  checking child " << child << std::endl; 

            if (open.count(child) == 0 || g.at(to_analyze) + edge_cost < g.at(child))
            {
                open.emplace(child); // noop if already on the open list
                parent[child] = to_analyze;
                g[child] = g.at(to_analyze) + edge_cost;
                f[child] = g[child] + heuristic(child);

                // std::cout << "   new parent! in open? " << open.count(child) << " g: " << g.at(to_analyze) << " + "<< edge_cost << "=" << g[child] << " f:" << f[child] << std::endl; 

            }
        }
    }

    std::cout << "Iters " << iters << std::endl;

    if (g.count(problem.goal_node) == 0)
    {
        return {false, {}, 0.0};
    }

    GraphSearchResult result{true, {problem.goal_node}, g.at(problem.goal_node)}; //

    auto current = problem.goal_node;
    while (current != problem.init_node)
    {
        current = parent.at(current);
        result.node_path.push_front(current);
    }

    return result;
}
