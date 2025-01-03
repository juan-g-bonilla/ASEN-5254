#pragma once

#include "AMPCore.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

using BGPoint = boost::geometry::model::d2::point_xy<double>;
using BGSegment = boost::geometry::model::segment<BGPoint>;
using BGLineString = boost::geometry::model::linestring<BGPoint>;
using BGPolygon = boost::geometry::model::polygon< BGPoint, false >;
using BGMultiPolygon = boost::geometry::model::multi_polygon<BGPolygon>;

BGPoint eigenToBGPoint(const Eigen::Vector2d &vec);

BGPolygon convertToBoostPolygon(const std::vector<Eigen::Vector2d> &eigenPoints);

BGMultiPolygon convertPolygonsToMultiPolygon(const std::vector<amp::Polygon> &polygons);

BGMultiPolygon bufferMultiPolygon(const BGMultiPolygon &multi, float buffer_distance);

std::vector<amp::Polygon> multiPolygonToAmp(BGMultiPolygon multi);

std::vector<amp::Polygon> mergePolygons(const std::vector<amp::Polygon>& polygon, float buffer_distance = 0);
