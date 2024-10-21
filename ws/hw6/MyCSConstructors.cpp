#include "MyCSConstructors.h"
#include "hw/HW6.h"

#include <queue>
#include "GeometryHelper.h"

////////////////////// THIS IS FROM HW4 //////////////////////

// Override this method for returning whether or not a point is in collision

double averageAngle(double angle1, double angle2) {
    double avgX = cos(angle1) + cos(angle2);
    double avgY = sin(angle1) + sin(angle2);
    auto res = atan2(avgY, avgX);
    // std::cout << "averageAngle " << angle1 << " " << angle2 << " " << res << std::endl;
    return res;
}

// Function to check if three points p, q, r are counter-clockwise
bool are_points_ccw(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
    return (r.y() - p.y()) * (q.x() - p.x()) > (q.y() - p.y()) * (r.x() - p.x());
}

// Function to check if two segments intersect
bool do_segments_intersect(const Eigen::Vector2d& seg1_start, const Eigen::Vector2d& seg1_end,
                           const Eigen::Vector2d& seg2_start, const Eigen::Vector2d& seg2_end) {
    return are_points_ccw(seg1_start, seg2_start, seg2_end) != are_points_ccw(seg1_end, seg2_start, seg2_end) &&
           are_points_ccw(seg1_start, seg1_end, seg2_start) != are_points_ccw(seg1_start, seg1_end, seg2_end);
}

// A segment intersects a polygon if it intersects any of its edges
bool does_segment_and_poly_intersect(const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end, 
                                     const amp::Polygon& poly) {
    int n = poly.verticesCCW().size();
    for (int i = 0; i < n; ++i) {
        Eigen::Vector2d poly_start = poly.verticesCCW()[i];
        Eigen::Vector2d poly_end = poly.verticesCCW()[(i + 1) % n];
        if (do_segments_intersect(seg_start, seg_end, poly_start, poly_end)) {
            return true;
        }
    }
    return false;
}

// A chain of segments intersects a polygon if any of its segments intersect the polygon
bool does_chain_and_poly_intersect(const Eigen::MatrixXd& chain, const amp::Polygon& poly) {
    int n = chain.rows() - 1;
    for (int i = 0; i < n; ++i) {
        Eigen::Vector2d chain_start = chain.row(i);
        Eigen::Vector2d chain_end = chain.row(i + 1);
        if (does_segment_and_poly_intersect(chain_start, chain_end, poly)) {
            return true;
        }
    }
    return false;
}

// A chain of segments intersects a group of polygons if it intersects any of them
bool does_chain_and_polys_intersect(const Eigen::MatrixXd& chain, const std::vector<amp::Polygon>& polys) {
    for (const auto& poly : polys) {
        if (does_chain_and_poly_intersect(chain, poly)) {
            return true;
        }
    }
    return false;
}

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = this->size().first * (x0 - this->x0Bounds().first) / (this->x0Bounds().second - this->x0Bounds().first); // x index of cell
    std::size_t cell_y = this->size().second * (x1 - this->x1Bounds().first) / (this->x1Bounds().second - this->x1Bounds().first);; // x index of cell
    if (cell_x == this->size().first)
        cell_x--;
    if (cell_y == this->size().second)
        cell_y--;
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // std::cerr << "TEST:" << std::endl;
    // std::cerr << "  " << manipulator.getLinkLengths()[0] << " " << manipulator.getLinkLengths()[1] << manipulator.getBaseLocation().transpose() << std::endl;
    // size_t i = 0;
    // for (auto&& obs : env.obstacles)
    // {
    //     std::cerr << "std::vector<Eigen::Vector2d> wo" << i << " = {" << std::endl;
        
    //     for (auto&& vert : obs.verticesCCW())
    //     {
    //         std::cerr << "{"<< vert.x() <<","<< vert.y() <<"}, ";
    //     }

    //     std::cerr <<  std::endl << "};" << std::endl;
    //     i++;
    // }

    // std::cerr << "ws.obstacles = {";
    // for (auto j = 0; j <i; j++)
    // {
    //     std::cerr << "wo" << j << ", ";
    // }
    // std::cerr << "};" << std::endl;

    // auto obstacles = mergePolygons(env.obstacles, 0.05);
    // auto obstacles = env.obstacles;
    auto multi = bufferMultiPolygon(convertPolygonsToMultiPolygon(env.obstacles), 0.05);

    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, -M_PI, M_PI, -M_PI, M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    Eigen::VectorXd thetas_0 = Eigen::VectorXd::LinSpaced(cspace.size().first+1, cspace.x0Bounds().first, cspace.x0Bounds().second).head(cspace.size().first);
    Eigen::VectorXd thetas_1 = Eigen::VectorXd::LinSpaced(cspace.size().second+1, cspace.x1Bounds().first, cspace.x1Bounds().second).head(cspace.size().second);

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for (int i = 0; i < cspace.size().first; ++i) {
        for (int j = 0; j < cspace.size().second; ++j) {
            // Get the current angles for the chain links
            Eigen::Vector2d state {thetas_0[i], thetas_1[j]};

            // Get forward kinematics for the current configuration
            Eigen::MatrixXd link_ends(3, 2);
            BGPolygon chain;
            for (int k = 0; k < 3; ++k) {
                link_ends.row(k) = manipulator.getJointLocation(state, k);
                boost::geometry::append(chain.outer(), eigenToBGPoint(link_ends.row(k)));
            }

            // Check if the chain of links intersects any of the obstacles
            if (boost::geometry::intersects(multi, chain)) { // does_chain_and_polys_intersect(link_ends, obstacles)
                cspace(i, j) = true;
                // std::cout << i << " " << j << std::endl;
            }
            else
            {
                cspace(i,j) = false;
            }

            // std::cout << state << " " << link_ends << " -> " << cspace(i, j) << std::endl << std::endl;
        }
    }

    cspace.manipulator = &manipulator;
    cspace.env = &env;

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D &env)
{
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    MyGridCSpace2D& cspace = *cspace_ptr;

    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(cspace.size().first+1, cspace.x0Bounds().first, cspace.x0Bounds().second);
    Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(cspace.size().second+1, cspace.x1Bounds().first, cspace.x1Bounds().second);

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for (int i = 0; i < cspace.size().first; ++i) {
        for (int j = 0; j < cspace.size().second; ++j) {

            // Check on a square
            Eigen::MatrixXd link_ends(5, 2); 
            link_ends.row(0) = Eigen::Vector2d{x[i], y[j]};
            link_ends.row(1) = Eigen::Vector2d{x[i+1], y[j]};
            link_ends.row(2) = Eigen::Vector2d{x[i+1], y[j+1]};
            link_ends.row(3) = Eigen::Vector2d{x[i], y[j+1]};
            link_ends.row(4) = Eigen::Vector2d{x[i], y[j]};

            // Check if the chain of links intersects any of the obstacles
            if (does_chain_and_polys_intersect(link_ends, env.obstacles)) {
                cspace(i, j) = true;
            }
            else
            {
                cspace(i,j) = false;
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace, bool isManipulator)
{
    const int xDirs[] = {-1, 1, 0, 0, -1, -1,  1,  1};
    const int yDirs[] = {0, 0, -1, 1, -1,  1, -1, -1};

    auto init = grid_cspace.getCellFromPoint(q_init.x(), q_init.y());
    auto goal = grid_cspace.getCellFromPoint(q_goal.x(), q_goal.y());

    // if (isManipulator && grid_cspace(init.first, init.second))
    // {
    //     auto cs = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);
    //     // std::cout << cs << std::endl;
    //     Eigen::Vector2d endLoc = cs->manipulator->getJointLocation(q_init, 2);
    //     for (int first = 0; first < 2; first ++)
    //     {
    //         auto newinit = dynamic_cast<const MyManipulator2D*>(cs->manipulator)->getConfigurationFromIK2Link(endLoc, first);
    //         init = grid_cspace.getCellFromPoint(newinit.x(), newinit.y());
    //         if (!grid_cspace(init.first, init.second)) break;
    //     }
    // }

    if (isManipulator && grid_cspace(init.first, init.second))
    {
        
        auto cs = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);
        std::cout << "INIT: " << q_init.transpose() << " " << cs->manipulator->getJointLocation(q_init, 2).transpose() << std::endl;

        bool succ = false;
        for (int i = 0; i < 8; ++i) {
            auto new_x = q_init.x()+.05*xDirs[i];
            auto new_y = q_init.y()+.05*yDirs[i];
            if (!(new_x>-M_PI && new_x<M_PI && new_y>-M_PI && new_y<M_PI)) continue;
            init = grid_cspace.getCellFromPoint(new_x, new_y);
            if (grid_cspace(init.first, init.second))
            {
                std::cout << "  tried and failed: " << new_x << " " << new_y <<  std::endl;
            }
            else
            {
                std::cout << "  good: " << new_x << " " << new_y <<  std::endl;
                succ = true;
                break;
            }
        }

        if (!succ)
        {
            std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(grid_cspace.size().first);
            std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
            amp::ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

            amp::Problem2D problem;
            problem.x_min = cs->env->x_min;
            problem.x_max = cs->env->x_max;
            problem.y_min = cs->env->y_min;
            problem.y_max = cs->env->y_max;
            problem.obstacles = cs->env->obstacles;
            problem.q_init = cs->manipulator->getJointLocation(q_init, 2).transpose();
            problem.q_goal = cs->manipulator->getJointLocation(q_goal, 2).transpose();

            // amp::Path2D dummy;
            // dummy.waypoints.push_back(q_init);
            // dummy.waypoints.push_back(q_goal);

            // amp::Visualizer::makeFigure(problem, *cs->manipulator, dummy);
            // amp::Visualizer::makeFigure(grid_cspace, dummy);
            // amp::Visualizer::showFigures();

            std::cerr << "ERROR: INIT IS IN AN INVALID INITIAL POSITION" << std::endl;
            return {};
        }
    }

    // if (isManipulator && grid_cspace(goal.first, goal.second))
    // {
    //     auto cs = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);
    //     Eigen::Vector2d endLoc = cs->manipulator->getJointLocation(q_goal, 2);
    //     for (int first = 0; first < 2; first ++)
    //     {
    //         auto newqgoal = dynamic_cast<const MyManipulator2D*>(cs->manipulator)->getConfigurationFromIK2Link(endLoc, first);
    //         goal = grid_cspace.getCellFromPoint(newqgoal.x(), newqgoal.y());
    //         if (!grid_cspace(goal.first, goal.second)) break;
    //     }
    // }

    if (isManipulator && grid_cspace(goal.first, goal.second))
    {
        auto cs = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);
        std::cout << "GOAL: " << q_goal.transpose() << " " << cs->manipulator->getJointLocation(q_goal, 2).transpose() << std::endl;

        bool succ = false;
        for (int i = 0; i < 8; ++i) {
            auto new_x = q_goal.x()+.05*xDirs[i];
            auto new_y = q_goal.y()+.05*yDirs[i];
            if (!(new_x>-M_PI && new_x<M_PI && new_y>-M_PI && new_y<M_PI)) continue;
            goal = grid_cspace.getCellFromPoint(new_x, new_y);
            if (!grid_cspace(goal.first, goal.second))
            {
                succ = true;
                break;
            }
        }

        if (!succ)
        {
            std::cerr << "ERROR: GOAL IS IN AN INVALID INITIAL POSITION" << std::endl;
            return {};
        }
    }
    // if (grid_cspace(init.first, init.second) || grid_cspace(goal.first, goal.second))
    // {
    //     if (isManipulator){
    //         auto cs = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);

    //         std::cout << "INIT: " << grid_cspace(init.first, init.second) << " " << q_init.transpose() << " " << cs->manipulator->getJointLocation(q_init, 2).transpose() << std::endl;
    //         std::cout << "GOAL: " << grid_cspace(goal.first, goal.second) << " " << q_goal.transpose() << " " << cs->manipulator->getJointLocation(q_goal, 2).transpose() << std::endl;

    //         std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(grid_cspace.size().first);
    //         std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();

    //         amp::ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

            

    //         amp::Problem2D problem;
    //         problem.x_min = cs->env->x_min;
    //         problem.x_max = cs->env->x_max;
    //         problem.y_min = cs->env->y_min;
    //         problem.y_max = cs->env->y_max;
    //         problem.obstacles = cs->env->obstacles;
    //         problem.q_init = q_init;
    //         problem.q_goal = q_goal;

    //         amp::Visualizer::makeFigure(problem, *cs->manipulator, amp::Path2D{});
    //         amp::Visualizer::makeFigure(grid_cspace);
    //         amp::Visualizer::showFigures();
    //         // std::cerr << "here!" << std::endl;

    //         // amp::ManipulatorTrajectory2Link trajectory = manip_algo.plan(*cs->manipulator, problem);
    //         //         std::cerr << "here!" << std::endl;

    //         // bool success = amp::HW6::checkLinkManipulatorPlan(path, *cs->manipulator, problem, true);

    //         // if (true)
    //         // {
    //         //     amp::Visualizer::makeFigure(problem, *cs->manipulator, path);
    //         //     // amp::Visualizer::makeFigure(*manip_algo.getCSpace(), path);
    //         //     amp::Visualizer::showFigures();
    //         // }
    //     }

    //     std::cerr << "ERROR: INIT IS IN AN INVALID INITIAL POSITION" << std::endl;
    //     return {};
    // }

    const double veryLarge = 1e5;
    Eigen::MatrixXd gridValues = veryLarge * Eigen::MatrixXd::Ones(grid_cspace.size().first, grid_cspace.size().second);

    std::queue<std::pair<int, int>> queue;


    queue.push(goal);
    gridValues(goal.first, goal.second) = 0;

    while (!queue.empty()) {
        auto [current_row, current_col] = queue.front();
        queue.pop();

        // Explore 4-connected neighbors
        for (int i = 0; i < 4; ++i) {
            int new_row = current_row + xDirs[i];
            int new_col = current_col + yDirs[i];

            if (!isManipulator && (new_row<0 || new_row>=grid_cspace.size().first || new_col<0 || new_col>=grid_cspace.size().second))
                continue; // outside of bounds!

            new_row = (new_row + grid_cspace.size().first) % grid_cspace.size().first;
            new_col = (new_col + grid_cspace.size().second) % grid_cspace.size().second;

            if (gridValues(new_row,new_col) != veryLarge)
                continue; // Already visited

            if (grid_cspace(new_row, new_col))
                continue; // within obstacle!

            gridValues(new_row,new_col) = gridValues(current_row,current_col) + 1;
            queue.push({new_row, new_col});
            // std::cout << new_row << " " << new_col << " " << gridValues(new_row,new_col) << std::endl;
        }
    }

    // std::cout << gridValues << std::endl;

    amp::Path2D path;

    path.waypoints.push_back(q_init);

    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(grid_cspace.size().first+1, grid_cspace.x0Bounds().first, grid_cspace.x0Bounds().second);
    Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(grid_cspace.size().second+1, grid_cspace.x1Bounds().first, grid_cspace.x1Bounds().second);
    auto getPointFromCell = [&](std::pair<size_t, size_t> p){
        if (isManipulator)
            return Eigen::Vector2d{
                averageAngle(x[p.first], x[p.first+1]), averageAngle(y[p.second], y[p.second+1])
            };

        return Eigen::Vector2d{
            (x[p.first] + x[p.first+1])/2, (y[p.second] + y[p.second+1])/2,
        };
    };

    auto posInGrid = init;
    // std::cout << q_init.transpose() << " " << getPointFromCell(posInGrid).transpose() << std::endl;

    while (true)
    {
        // std::cout << gridValues(posInGrid.first,posInGrid.second) << " " << posInGrid.first << ", " << posInGrid.second  << " " << getPointFromCell(posInGrid).transpose() << std::endl;

        path.waypoints.push_back(getPointFromCell(posInGrid));

        if (posInGrid == goal)
        {
            path.waypoints.push_back(q_goal);
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int new_row = posInGrid.first + xDirs[i];
            int new_col = posInGrid.second + yDirs[i];

            if (!isManipulator && (new_row<0 || new_row>=grid_cspace.size().first || new_col<0 || new_col>=grid_cspace.size().second))
                continue; // outside of bounds!

            new_row = (new_row + grid_cspace.size().first) % grid_cspace.size().first;
            new_col = (new_col + grid_cspace.size().second) % grid_cspace.size().second;

            if (gridValues(new_row,new_col) < gridValues(posInGrid.first,posInGrid.second))
            {
                posInGrid = {new_row,new_col};
                break;
            }
        }
    }

    if (isManipulator)
    {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(-M_PI, -M_PI);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(M_PI, M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }

    return path;
}
