#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

#include <limits>
#include <optional>

// A helper struct to store the intersection result
struct HitResult {
    Eigen::Vector2d intersection_point;
    const amp::Obstacle2D* obstacle;
    int vertex_on_left;
};

// Function to check for line segment intersection
// Returns optional intersection point if found
std::optional<Eigen::Vector2d> segmentIntersect(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
    const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);

double getDistance(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2);

// Function to find the closest point on a line segment to a given point
Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b);

// Function to find the closest point on the polygon boundary to a given point
Eigen::Vector2d closestPointOnPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point);

// Function to check if a point lies on a line segment between two vertices
bool isPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b, double epsilon = 1e-6);

// Function to insert a vertex into the polygon's list of vertices
void insertVertexInPolygon(std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& newVertex);

int findVertexIndex(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& target, double epsilon = 1e-6);

// Function to check if moving from point P to T intersects the polygon boundary
bool willIntersectBoundary(const Eigen::Vector2d& P, const Eigen::Vector2d& T, 
                           const std::vector<Eigen::Vector2d>& polygon);

void printVertices(const std::vector<Eigen::Vector2d>& vertices);

double calculatePathLength(const std::vector<Eigen::Vector2d>& waypoints);

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1Algorithm : public amp::BugAlgorithm 
{
public:
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

};