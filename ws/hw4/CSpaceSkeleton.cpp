#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

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
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
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
            for (int k = 0; k < 3; ++k) {
                link_ends.row(k) = manipulator.getJointLocation(state, k);
            }
            
            // Check if the chain of links intersects any of the obstacles
            if (does_chain_and_polys_intersect(link_ends, env.obstacles)) {
                cspace(i, j) = true;
                // std::cout << i << " " << j << std::endl;
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
