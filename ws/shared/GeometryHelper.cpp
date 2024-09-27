#include "GeometryHelper.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

using BGPoint = boost::geometry::model::d2::point_xy<double>;
using BGPolygon = boost::geometry::model::polygon< BGPoint, false >;
using BGMultiPolygon = boost::geometry::model::multi_polygon<BGPolygon>;

// Function to convert Eigen vector to Boost.Geometry point
BGPoint eigenToBGPoint(const Eigen::Vector2d& vec) {
    return BGPoint(vec.x(), vec.y());
}

// Function to convert a std::vector<Eigen::Vector2d> to Boost.Geometry polygon
BGPolygon convertToBoostPolygon(const std::vector<Eigen::Vector2d>& eigenPoints) {
    BGPolygon boostPolygon;
    
    // Add points to the outer boundary
    for (const auto& point : eigenPoints) {
        boost::geometry::append(boostPolygon.outer(), eigenToBGPoint(point));
    }
    
    // Ensure the polygon is closed by appending the first point at the end
    if (!eigenPoints.empty()) {
        boost::geometry::append(boostPolygon.outer(), eigenToBGPoint(eigenPoints.front()));
    }
    
    // Correct the polygon to ensure it's properly oriented
    boost::geometry::correct(boostPolygon);
    
    return boostPolygon;
}

std::vector<amp::Polygon> mergePolygons(std::vector<amp::Polygon> polygons, float buffer_distance)
{
    BGMultiPolygon merged; // the unioned polygons
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::join_miter join_strategy;
    boost::geometry::strategy::buffer::end_flat end_strategy;
    boost::geometry::strategy::buffer::point_square circle_strategy;
    boost::geometry::strategy::buffer::side_straight side_strategy;

    for (auto&& p : polygons) {
        BGPolygon bg_poly = convertToBoostPolygon(p.verticesCCW());
        // BGPolygon bg_buffered_poly;
        
        // add another polygon each iteration
        BGMultiPolygon tmp_poly;
        boost::geometry::union_(merged, bg_poly, tmp_poly);
        merged = tmp_poly;
    }

    BGMultiPolygon buffered;
    boost::geometry::buffer(merged, buffered, distance_strategy, side_strategy,
        join_strategy, end_strategy, circle_strategy);

    std::vector<amp::Polygon> result;
    for (auto&& p : buffered)
    {
        std::vector<Eigen::Vector2d> vertices;

        for (auto&& v : p.outer())
        {
            vertices.push_back({v.x(), v.y()});
        }
        vertices.pop_back(); // need to remove last (redundant point)

        result.emplace_back(vertices);
    }

    return result;
}