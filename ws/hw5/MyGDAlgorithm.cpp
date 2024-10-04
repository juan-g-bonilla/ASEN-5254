#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    
    MyPotentialFunction fn{problem, config}; 

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    
    for (auto iter = 0; iter < max_iter; iter++)
    {
        Eigen::Vector2d direction = fn.gradient(path.waypoints.back()).normalized();
        path.waypoints.push_back(path.waypoints.back() - alpha*direction);
        // std::cout << direction.transpose() << " " << path.waypoints.back().transpose() << std::endl;

        if ((problem.q_goal - path.waypoints.back()).squaredNorm() < std::pow(epsilon, 2))
        {
            path.waypoints.push_back(problem.q_goal);
            return path;
        }
    }
    
    return path;
}

std::pair<int, int>
findIndexInGrid(const Eigen::VectorXd& xGrid,
                const Eigen::VectorXd& yGrid,
                double x, double y)
{
    // Find the indices of the grid cell containing (x, y)
    auto x_it = std::lower_bound(xGrid.data(), xGrid.data() + xGrid.size(), x);
    auto y_it = std::lower_bound(yGrid.data(), yGrid.data() + yGrid.size(), y);

    int x_index = std::max(int(x_it - xGrid.data()) - 1, 0);
    int y_index = std::max(int(y_it - yGrid.data()) - 1, 0);

    // Handle boundary conditions
    if (x_index >= xGrid.size() - 1) x_index = xGrid.size() - 2;
    if (y_index >= yGrid.size() - 1) y_index = yGrid.size() - 2;

    return {x_index, y_index};
}

// Function to perform bilinear interpolation
double bilinearInterpolation(const Eigen::MatrixXd& gridValues,
                             const Eigen::VectorXd& xGrid,
                             const Eigen::VectorXd& yGrid,
                             double x, double y) {
    // Ensure xGrid and yGrid are sorted in ascending order
    assert((xGrid.size() >= 2) && "xGrid must have at least two points.");
    assert((yGrid.size() >= 2) && "yGrid must have at least two points.");
    assert((gridValues.rows() == yGrid.size()) && (gridValues.cols() == xGrid.size()) && "gridValues dimensions must match xGrid and yGrid.");

    auto[x_index, y_index] = findIndexInGrid(xGrid, yGrid, x, y);

    // Coordinates of the grid cell corners
    double x1 = xGrid[x_index];
    double x2 = xGrid[x_index + 1];
    double y1 = yGrid[y_index];
    double y2 = yGrid[y_index + 1];

    // Function values at the corners
    double fQ11 = gridValues(x_index, y_index);
    double fQ21 = gridValues(x_index, y_index + 1);
    double fQ12 = gridValues(x_index + 1, y_index);
    double fQ22 = gridValues(x_index + 1, y_index + 1);

    // Compute the interpolation weights
    double t_x = (x - x1) / (x2 - x1);
    double t_y = (y - y1) / (y2 - y1);

    // Bilinear interpolation formula
    double f_xy =
        fQ11 * (1 - t_x) * (1 - t_y) +
        fQ21 * (1 - t_x) * t_y +
        fQ12 * t_x * (1 - t_y) +
        fQ22 * t_x * t_y;

    // std::cout << "\tbilinearInterpolation " << x << " " << y << ": " << x1  << "," << x2 << " "<< y1  << "," << y2 << " " << fQ11 << "," << fQ22 << ": " << f_xy << std::endl;

    return f_xy;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
waveFrontPotential(const amp::Problem2D &problem)
{
    auto obstacles = bufferMultiPolygon(convertPolygonsToMultiPolygon(problem.obstacles), 0.1);

    double veryLarge = 1e8;

    auto N = 1000;
    Eigen::VectorXd xGrid = Eigen::VectorXd::LinSpaced(N, problem.x_min-2, problem.x_max+2);
    Eigen::VectorXd yGrid = Eigen::VectorXd::LinSpaced(N, problem.y_min-2, problem.y_max+2);
    Eigen::MatrixXd gridValues = veryLarge * Eigen::MatrixXd::Ones(N, N);

    std::queue<std::pair<int, int>> queue;
    // start from goal
    auto goal = findIndexInGrid(xGrid, yGrid, problem.q_goal.x(), problem.q_goal.y());
    // std::cout << problem.q_goal.transpose() << std::endl;
    // std::cout << xGrid(goal.first) << " " << yGrid(goal.second) << std::endl;
    // std::cout << std::endl;

    queue.push(goal);
    gridValues(goal.first, goal.second) = 0;

    const int xDirs[] = {-1, 1, 0, 0};
    const int yDirs[] = {0, 0, -1, 1};

    double maxValue = 0;

    while (!queue.empty()) {
        auto [current_row, current_col] = queue.front();
        queue.pop();

        // Explore 4-connected neighbors
        for (int i = 0; i < 4; ++i) {
            int new_row = current_row + xDirs[i];
            int new_col = current_col + yDirs[i];

            if (new_row<0 || new_row>=N || new_col<0 || new_col>=N)
                continue; // outside of bounds!

            if (gridValues(new_row,new_col) != veryLarge)
                continue; // Already visited

            if (boost::geometry::within(
                BGPoint{xGrid(new_row), yGrid(new_col)},
                obstacles
            ))
                continue; // within obstacle!

            gridValues(new_row,new_col) = gridValues(current_row,current_col) + 1;
            queue.push({new_row, new_col});
            maxValue = std::max(maxValue, gridValues(new_row,new_col));
        }
    }

    // std::cout << xGrid << std::endl;
    // std::cout << yGrid << std::endl;

    // std::cout << gridValues << std::endl;
    
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            if (gridValues(i,j) == veryLarge)
                gridValues(i,j) = maxValue;


    gridValues /= maxValue; // Normalize to 1


    return {gridValues, xGrid, yGrid};
}

double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const
{
    // double U_att = (q - goal).norm() < config.d_star 
    //     ? config.zetta/2*(q - goal).squaredNorm() 
    //     : config.d_star*config.zetta*(q - goal).norm() - config.zetta/2*std::pow(config.d_star, 2);

    double U_att = bilinearInterpolation(
        std::get<0>(this->waveFront),
        std::get<1>(this->waveFront),
        std::get<2>(this->waveFront),
        q.x(), q.y()
    );

    double distance = boost::geometry::distance(obstacles, BGPoint{q.x(), q.y()});
    double U_rep = distance < config.Q_star ? config.eta*(config.Q_star/std::max(distance, 1e-5) - 1) : 0;
    U_rep = 0;
    return U_att + U_rep;
}

MyPotentialFunction::MyPotentialFunction(const amp::Problem2D &problem, PotentialConfig config)
: goal(problem.q_goal), config(config), waveFront(waveFrontPotential(problem))
{
    this->obstacles = convertPolygonsToMultiPolygon(problem.obstacles);

    // double spacing = 0.001;

    // for (auto&& poly : problem.obstacles)
    // {
    //     const auto& vertices = poly.verticesCCW();

    //     // Iterate over each edge of the polygon
    //     for (size_t i = 0; i < vertices.size(); ++i) {
    //         // Get current and next vertex (wrap around for the last edge)
    //         const Eigen::Vector2d& v1 = vertices[i];
    //         const Eigen::Vector2d& v2 = vertices[(i + 1) % vertices.size()];

    //         // Compute the length of the edge
    //         double edge_length = (v1 - v2).norm();

    //         // Compute the number of points to place on this edge
    //         int num_points = static_cast<int>(std::floor(edge_length / spacing));

    //         // Generate equispaced points along this edge
    //         for (int j = 0; j <= num_points; ++j) {
    //             double t = j * spacing / edge_length;  // interpolation factor
    //             Eigen::Vector2d point = (1.0 - t) * v1 + t * v2;
    //             obstacles.push_back(point);
    //         }
    //     }
    // }

}

Eigen::Vector2d MyPotentialFunction::gradient(const Eigen::Vector2d &x, double h) const
{
		Eigen::Vector2d grad = Eigen::Vector2d::Zero();
		Eigen::Vector2d x_perturbed = x;

		// Loop over each dimension to compute the partial derivatives
		for (int i = 0; i < x.size(); ++i) {
			// Perturb only the i-th component
			x_perturbed(i) += h;
			double f_x_plus_h = (*this)(x_perturbed);
            // std::cout << "\t" << i << " " << f_x_plus_h << " @ " << x_perturbed.transpose() << std::endl;

			// Reset the perturbed component
			x_perturbed(i) = x(i);

			// Compute the finite difference for the i-th component
			grad(i) = (f_x_plus_h - (*this)(x)) / h;
		}

		return grad;
	}
