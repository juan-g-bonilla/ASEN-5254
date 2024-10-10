#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

#include "GeometryHelper.h"

struct PotentialConfig
{
	double d_star, zetta, Q_star, eta;
};

class MyPotentialFunction : public amp::PotentialFunction2D {
public:
	MyPotentialFunction(const amp::Problem2D& problem, PotentialConfig config);

	// Returns the potential function value (height) for a given 2D point. 
	virtual double operator()(const Eigen::Vector2d& q) const override;

	Eigen::Vector2d getGradient(const Eigen::Vector2d& x) const override;

public:
	double h = 1e-3;

private:
	Eigen::Vector2d goal;
	BGMultiPolygon obstacles;
	PotentialConfig config;
	std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd> waveFront;
};

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double alpha, double epsilon, PotentialConfig config, size_t max_iter = 10000) :
			alpha(alpha),
			epsilon(epsilon),
			config(config),
			max_iter(max_iter) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;

		double alpha, epsilon;
		PotentialConfig config;
		size_t max_iter;
		
		// Add additional member variables here...
};