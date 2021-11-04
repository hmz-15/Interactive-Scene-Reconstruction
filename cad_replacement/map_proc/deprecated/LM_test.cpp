#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <unsupported/Eigen/NonLinearOptimization>

#include <ros/ros.h>


struct LMFunctor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf measuredValues;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

		float aParam = x(0);
		float bParam = x(1);
		float cParam = x(2);

		for (int i = 0; i < values(); i++) {
			float xValue = measuredValues(i, 0);
			float yValue = measuredValues(i, 1);

			fvec(i) = yValue - (aParam * xValue * xValue + bParam * xValue + cParam);
		}
		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fjac' has dimensions m x n
		// It will contain the jacobian of the errors, calculated numerically in this case.

		float epsilon;
		epsilon = 1e-7f;

		for (int i = 0; i < x.size(); i++) {
			Eigen::VectorXf xPlus(x);
			xPlus(i) += epsilon;
			Eigen::VectorXf xMinus(x);
			xMinus(i) -= epsilon;

			Eigen::VectorXf fvecPlus(values());
			operator()(xPlus, fvecPlus);

			Eigen::VectorXf fvecMinus(values());
			operator()(xMinus, fvecMinus);

			Eigen::VectorXf fvecDiff(values());
			fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

			fjac.block(0, i, values(), 1) = fvecDiff;
		}

		return 0;
	}

	// Number of data points, i.e. values.
	int m;

	// Returns 'm', the number of values.
	int values() const { return m; }

	// The number of parameters, i.e. inputs.
	int n;

	// Returns 'n', the number of inputs.
	int inputs() const { return n; }

};


int main(int argc, char *argv[])
{
    // ros::init(argc, argv, "LM_node");
    // ros::start();
	// //
	// Goal
	//
	// Given a non-linear equation: f(x) = a(x^2) + b(x) + c
	// and 'm' data points (x1, f(x1)), (x2, f(x2)), ..., (xm, f(xm))
	// our goal is to estimate 'n' parameters (3 in this case: a, b, c)
	// using LM optimization.
	//

	//
	// Read values from file.
	// Each row has two numbers, for example: 5.50 223.70
	// The first number is the input value (5.50) i.e. the value of 'x'.
	// The second number is the observed output value (223.70),
	// i.e. the measured value of 'f(x)'.
	

	// 'm' is the number of data points.
	int m = 100;

	// Move the data into an Eigen Matrix.
	// The first column has the input values, x. The second column is the f(x) values.
    float a = 0.1;
    float b = 0.2;
    float c = 0.3;
	Eigen::MatrixXf measuredValues(m, 2);
	for (int i = 0; i < m; i++) {
		measuredValues(i, 0) = (float) rand()/RAND_MAX * 10;
		measuredValues(i, 1) = a*measuredValues(i, 0)*measuredValues(i, 0) + b*measuredValues(i, 0) + c + (float) rand()/RAND_MAX/10;
	}

	// 'n' is the number of parameters in the function.
	// f(x) = a(x^2) + b(x) + c has 3 parameters: a, b, c
	int n = 3;

	// 'x' is vector of length 'n' containing the initial values for the parameters.
	// The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
	// The LM optimization inputs should not be confused with the x input values.
	Eigen::VectorXf x(n);
	x(0) = 0.0;             // initial value for 'a'
	x(1) = 0.0;             // initial value for 'b'
	x(2) = 0.0;             // initial value for 'c'

	//
	// Run the LM optimization
	// Create a LevenbergMarquardt object and pass it the functor.
	//

	LMFunctor functor;
	functor.measuredValues = measuredValues;
	functor.m = m;
	functor.n = n;

	Eigen::LevenbergMarquardt<LMFunctor, float> lm(functor);
	int status = lm.minimize(x);
	std::cout << "LM optimization status: " << status << std::endl;

	//
	// Results
	// The 'x' vector also contains the results of the optimization.
	//
	std::cout << "Optimization results" << std::endl;
	std::cout << "\ta: " << x(0) << std::endl;
	std::cout << "\tb: " << x(1) << std::endl;
	std::cout << "\tc: " << x(2) << std::endl;
    
    // ros::shutdown();
	return 0;
}