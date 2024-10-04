#include "AMPCore.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

using BGPoint = boost::geometry::model::d2::point_xy<double>;
using BGPolygon = boost::geometry::model::polygon< BGPoint, false >;
using BGMultiPolygon = boost::geometry::model::multi_polygon<BGPolygon>;

BGMultiPolygon convertPolygonsToMultiPolygon(const std::vector<amp::Polygon>& polygons);

BGMultiPolygon bufferMultiPolygon(const BGMultiPolygon &multi, float buffer_distance);

std::vector<amp::Polygon> mergePolygons(const std::vector<amp::Polygon>& polygon, float buffer_distance = 0);
