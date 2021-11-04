#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

#include "common.h"
#include "utils.h"

namespace MapProcessing
{

struct FineAlignmentError
{
	// 'm' pairs of (x, f(x))
    int m;
    // relative to supported plane (height)
    Eigen::VectorXf supporting_planes_cad;
    Eigen::VectorXf supporting_planes_object;

    int n;
    // relative to bounding box frame
	Eigen::MatrixXf planes_cad;
    Eigen::MatrixXf planes_object;

    // int c;
    // std::vector<OBBox> children_boxes;
    OBBox object_initial_box;

    int w;
    Eigen::MatrixXf planes_wall;

    // int co;
    // std::vector<std::pair<OBBox, float>> collided_boxes;

    float half_ground_dim_cad;
    float half_ground_dim_object;
    // x,y,z correspondance
    Eigen::Vector3f ground;

    float half_u_dim_cad;
    float half_u_dim_object;
    float half_v_dim_cad;
    float half_v_dim_object;

	// Error vector
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
        fvec.resize(values());
        fvec.setZero();

        // Variables to be optimized
		float s = x(0);
		float d_theta = x(1);
		float d_u = x(2);
        float d_v = x(3);

        // m height-misalignment error
        for (int i = 0; i < m; i++)
            fvec(i) = 8*(supporting_planes_object(i) - s * supporting_planes_cad(i));
        
        // Tb_c.inverse().transpose()
        Eigen::Matrix3f rotation = Eigen::Matrix3f::Zero();
        rotation(ground(0), ground(0)) = cos(d_theta);
        rotation(ground(0), ground(1)) = -sin(d_theta);
        rotation(ground(1), ground(0)) = sin(d_theta);
        rotation(ground(1), ground(1)) = cos(d_theta);
        rotation(ground(2), ground(2)) = 1.0;

        Eigen::Vector3f trans;
        trans(ground(0)) = d_u;
        trans(ground(1)) = d_v;
        trans(ground(2)) = half_ground_dim_cad * s - half_ground_dim_object;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.topLeftCorner(3,3) = rotation;
        transform.row(3).head(3) = -trans.transpose()*rotation;
        transform(3, 3) = s;


        for (int i = 0; i < n; i++)
        {
            Eigen::VectorXf current_plane_cad = transform * planes_cad.col(i);
            Eigen::VectorXf current_plane_object = planes_object.col(i);
            fvec(m+i*2) = 5*(1.0 - current_plane_cad.head(3).transpose() * current_plane_object.head(3));
            fvec(m+i*2+1) = 5*(current_plane_cad(3) - current_plane_object(3));
        }

        OBBox box = object_initial_box;
        Eigen::Matrix3f initial_transform = box.quat.toRotationMatrix();
        box.aligned_dims(ground(0)) = 2*s*half_u_dim_cad;
        box.aligned_dims(ground(1)) = 2*s*half_v_dim_cad;
        box.aligned_dims(ground(2)) = 2*s*half_ground_dim_cad;
        box.pos += initial_transform * trans;
        Eigen::Quaternionf quat (initial_transform*rotation);
        box.quat = quat;

        for (int i = 0; i < w; i++)
            fvec(m+2*n+i) = 3*ComputeBoxPlanePenetration (box, planes_wall.col(i));

        fvec(m+2*n+w) = 0.5*std::max(0.0f, s * half_u_dim_cad - half_u_dim_object + d_u) ;
        fvec(m+2*n+w+1) = 0.5*std::min(0.0f, s * half_u_dim_cad + half_u_dim_object + d_u);

        fvec(m+2*n+w+2) = 0.5*std::max(0.0f, - s * half_u_dim_cad - half_u_dim_object + d_u) ;
        fvec(m+2*n+w+3) = 0.5*std::min(0.0f, -s * half_u_dim_cad + half_u_dim_object + d_u);

        fvec(m+2*n+w+4) = 0.5*std::max(0.0f, s * half_v_dim_cad - half_v_dim_object + d_v);
        fvec(m+2*n+w+5) = 0.5*std::min(0.0f, s * half_v_dim_cad + half_v_dim_object + d_v);

        fvec(m+2*n+w+6) = 0.5*std::max(0.0f, -s * half_v_dim_cad - half_v_dim_object + d_v);
        fvec(m+2*n+w+7) = 0.5*std::min(0.0f, -s * half_v_dim_cad + half_v_dim_object + d_v);

        fvec(m+2*n+w+8) = 2*(half_ground_dim_cad * s - half_ground_dim_object);
        fvec(m+2*n+w+9) = 0.0;

		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
	{
		// 'x' has dimensions p x 1
		// It contains the current estimates for the parameters.

		// 'fjac' has dimensions N x p
		// It will contain the jacobian of the errors, calculated numerically in this case.

		float epsilon;
		epsilon = 1e-6f;

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
	int N;

	// Returns 'N', the number of values.
	int values() const { return N; }

	// The number of parameters, i.e. inputs.
	int p;

	// Returns 'p', the number of inputs.
	int inputs() const { return p; }

};

}