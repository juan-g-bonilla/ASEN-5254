#include "MyBugAlgorithm.h"

// Function to check for line segment intersection
// Returns optional intersection point if found
std::optional<Eigen::Vector2d> segmentIntersect(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
    const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    
    // Calculate the direction vectors
    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;

    double rxs = r.x() * s.y() - r.y() * s.x();  // Cross product of r and s
    Eigen::Vector2d qp = q1 - p1;
    double qpCrossR = qp.x() * r.y() - qp.y() * r.x(); // Cross product of (q1-p1) and r

    // If rxs == 0, the lines are parallel or collinear
    if (std::abs(rxs) < 1e-8) {
        return std::nullopt;
    }

    double t = (qp.x() * s.y() - qp.y() * s.x()) / rxs;
    double u = qpCrossR / rxs;

    // Check if t and u are within [0, 1] which means segments intersect
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        return p1 + t * r;  // Intersection point
    }

    return std::nullopt;  // No intersection
}

double getDistance(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2) {
    return (point1 - point2).norm();
}

// Function to find the first hit point
std::optional<HitResult> findFirstHit(
    const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
    const std::vector<amp::Obstacle2D>& obstacles) {

    std::optional<HitResult> closestHit;
    double closestDistance = std::numeric_limits<double>::max();

    for (const auto& obstacle : obstacles) {
        const auto& vertices = obstacle.verticesCCW();
        int vertexCount = vertices.size();

        for (int i = 0; i < vertexCount; ++i) {
            const Eigen::Vector2d& v1 = vertices[i];
            const Eigen::Vector2d& v2 = vertices[(i + 1) % vertexCount];  // Next vertex, wrap around

            // Check if the segment intersects with this edge
            if (auto intersection = segmentIntersect(start, end, v1, v2)) {
                double distance = getDistance(intersection.value(), start);  // Compute distance from start

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestHit = HitResult{intersection.value(), &obstacle, i};
                }
            }
        }
    }

    return closestHit;  // Return the closest hit if found, otherwise nullopt
}

// Function to find the closest point on a line segment to a given point
Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d ab = b - a;  // Vector from point a to point b
    double t = (p - a).dot(ab) / ab.squaredNorm();  // Project point p onto the line a-b

    // Clamp t to the range [0, 1] to ensure the closest point lies on the segment
    t = std::max(0.0, std::min(1.0, t));

    // Return the closest point on the segment
    return a + t * ab;
}

// Function to find the closest point on the polygon boundary to a given point
Eigen::Vector2d closestPointOnPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point) {
    Eigen::Vector2d closestPoint;
    double minDistance = std::numeric_limits<double>::max();

    // Iterate over each edge of the polygon (vertex pairs)
    for (size_t i = 0; i < polygon.size(); ++i) {
        Eigen::Vector2d a = polygon[i];
        Eigen::Vector2d b = polygon[(i + 1) % polygon.size()];  // Wrap around to form a closed polygon
        
        // Find the closest point on this edge
        Eigen::Vector2d candidate = closestPointOnSegment(point, a, b);
        
        // Calculate the distance from the given point to this candidate point
        double distance = (point - candidate).norm();
        
        // If this is the closest point so far, store it
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = candidate;
        }
    }
    
    return closestPoint;
}

// Function to check if a point lies on a line segment between two vertices
bool isPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b, double epsilon) {
    // Check if the point p is collinear with a and b and lies between them
    Eigen::Vector2d ab = b - a;
    Eigen::Vector2d ap = p - a;

    // Check if the point is collinear with the segment
    double crossProduct = ab.x() * ap.y() - ab.y() * ap.x();
    if (std::abs(crossProduct) > epsilon) {
        return false;  // Not collinear
    }

    // Check if the point lies between a and b
    double dotProduct = ap.dot(ab);
    if (dotProduct < 0 || dotProduct > ab.squaredNorm()) {
        return false;  // Point is not between a and b
    }

    return true;
}

// Function to insert a vertex into the polygon's list of vertices
void insertVertexInPolygon(std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& newVertex) {
    if (findVertexIndex(polygon, newVertex) > -1)
    {
        return;
    }

    for (size_t i = 0; i < polygon.size(); ++i) {
        // Get the current edge (segment) from polygon[i] to polygon[i+1]
        Eigen::Vector2d a = polygon[i];
        Eigen::Vector2d b = polygon[(i + 1) % polygon.size()];  // Wrap around for closed polygon

        // Check if the new vertex lies on this segment
        if (isPointOnSegment(newVertex, a, b)) {
            // Insert the new vertex between a and b
            polygon.insert(polygon.begin() + (i + 1), newVertex);
            return;
        }
    }
}

int findVertexIndex(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& target, double epsilon) {
    for (size_t i = 0; i < vertices.size(); ++i) {
        if ((vertices[i] - target).norm() < epsilon) {
            return i;  // Return the index if the target vertex is found
        }
    }
    return -1;
}

// Function to check if moving from point P to T intersects the polygon boundary
bool willIntersectBoundary(const Eigen::Vector2d& P, const Eigen::Vector2d& T, 
                           const std::vector<Eigen::Vector2d>& polygon) {
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector2d A = polygon[i];
        Eigen::Vector2d B = polygon[(i + 1) % n]; // Wrap around to form closed polygon

        if (segmentIntersect(P, T, A, B)) {
            return true;
        }
    }

    return false;
}

double calculatePathLength(const std::vector<Eigen::Vector2d>& waypoints) {
    double totalLength = 0.0;

    if (waypoints.size() < 2) {
        std::cerr << "Error: Need at least two waypoints to calculate path length." << std::endl;
        printVertices(waypoints);
        return 0.0;
    }

    // Iterate through the waypoints and sum up the distances between consecutive points
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Eigen::Vector2d segment = waypoints[i + 1] - waypoints[i];
        totalLength += segment.norm(); // Calculate the Euclidean distance
    }

    return totalLength;
}

void printVertices(const std::vector<Eigen::Vector2d>& vertices) {
    std::cout << "Vertices:" << std::endl;
    for (const auto& vertex : vertices) {
        std::cout << "(" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
    }
}

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem)
{
        amp::Path2D path;
        path.waypoints.push_back(problem.q_init);
        
        auto current = problem.q_init;

        for (auto maxIters = 0; maxIters < 10000; maxIters++)
        {
            auto hit = findFirstHit(current, problem.q_goal, problem.obstacles);

            if (hit)
            {       
                LOG("Hit!");
                const auto& vertices = hit->obstacle->verticesCCW();
                int vertexCount = vertices.size();

                if (findVertexIndex(vertices, hit->intersection_point) == -1)
                {
                    path.waypoints.push_back(hit->intersection_point);
                }

                // Go through every single vertex
                for (int i = 0; i < vertexCount; ++i) {
                    const Eigen::Vector2d& vertex = vertices.at((vertexCount + hit->vertex_on_left - i) % vertexCount);
                    path.waypoints.push_back(vertex);
                }

                LOG("Hit point " << hit->intersection_point.transpose());

                Eigen::Vector2d leave_point = closestPointOnPolygon(hit->obstacle->verticesCCW(), problem.q_goal);
                LOG("Leave point " << leave_point.transpose());

                auto mutableVertices = hit->obstacle->verticesCCW();
                insertVertexInPolygon(mutableVertices, hit->intersection_point);
                insertVertexInPolygon(mutableVertices, leave_point);
                vertexCount = mutableVertices.size();

                auto leave_i = findVertexIndex(mutableVertices, leave_point);
                auto hit_i = findVertexIndex(mutableVertices, hit->intersection_point);

                amp::Path2D path_option_1, path_option_2;
                for (int i = 0; i < vertexCount; ++i) {
                    auto vertex_index = (hit_i + i) % vertexCount;
                    const Eigen::Vector2d& vertex = mutableVertices.at(vertex_index);
                    path_option_1.waypoints.push_back(vertex);
                    if (vertex_index == leave_i) break;
                }
                for (int i = 0; i < vertexCount; ++i) {
                    auto vertex_index =(vertexCount + hit_i - i) % vertexCount;
                    const Eigen::Vector2d& vertex = mutableVertices.at(vertex_index);
                    path_option_2.waypoints.push_back(vertex);
                    if (vertex_index == leave_i) break;
                }

                std::cout << "Option 1" << std::endl;
                printVertices(path_option_1.waypoints);

                std::cout << std::endl << "Option 2" << std::endl;
                printVertices(path_option_2.waypoints);

                auto& selected_path = calculatePathLength(path_option_1.waypoints) < calculatePathLength(path_option_2.waypoints) ? path_option_1 : path_option_2;
                path.waypoints.insert(path.waypoints.end(), selected_path.waypoints.begin(), selected_path.waypoints.end());

                Eigen::Vector2d slightly_out_leave_point = leave_point;
                slightly_out_leave_point += (problem.q_goal - leave_point).normalized() * 1e-5;

                if (willIntersectBoundary(slightly_out_leave_point, problem.q_goal, hit->obstacle->verticesCCW()))
                {
                    path.valid = false;
                    return path;
                }

                current = slightly_out_leave_point;
            }
            else
            {
                path.waypoints.push_back(problem.q_goal);
                LOG("FOUND PATH!");
                printVertices(path.waypoints);
                path.valid = true;
                return path;
            }
        }

        throw std::runtime_error("Endless iteration");
    }
