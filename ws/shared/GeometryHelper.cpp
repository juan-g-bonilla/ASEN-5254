#include "GeometryHelper.h"

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

BGMultiPolygon convertPolygonsToMultiPolygon(const std::vector<amp::Polygon>& polygons)
{
    BGMultiPolygon merged; // the unioned polygons

    for (auto&& p : polygons) {
        BGPolygon bg_poly = convertToBoostPolygon(p.verticesCCW());
        // BGPolygon bg_buffered_poly;
        
        // add another polygon each iteration
        BGMultiPolygon tmp_poly;
        boost::geometry::union_(merged, bg_poly, tmp_poly);
        merged = tmp_poly;
    }

    return merged;
}

BGMultiPolygon bufferMultiPolygon(const BGMultiPolygon& multi, float buffer_distance)
{
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::join_miter join_strategy;
    boost::geometry::strategy::buffer::end_flat end_strategy;
    boost::geometry::strategy::buffer::point_square circle_strategy;
    boost::geometry::strategy::buffer::side_straight side_strategy;

    BGMultiPolygon buffered;
    boost::geometry::buffer(multi, buffered, distance_strategy, side_strategy,
        join_strategy, end_strategy, circle_strategy);
    return buffered;
}

std::vector<amp::Polygon> multiPolygonToAmp(BGMultiPolygon multi)
{
    std::vector<amp::Polygon> result;
    for (auto&& p : multi)
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

std::vector<amp::Polygon> mergePolygons(const std::vector<amp::Polygon>& polygons, float buffer_distance)
{
    return multiPolygonToAmp(bufferMultiPolygon(convertPolygonsToMultiPolygon(polygons), buffer_distance));
}