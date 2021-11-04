#ifndef _PGM_NODE_TYPE_H
#define _PGM_NODE_TYPE_H

#include <iostream>
#include <vector>
#include <utility>


namespace pgm{

struct Point{
    double x;
    double y;
    double z;

    Point(double x_=0, double y_=0, double z_=0){
        x = x_;
        y = y_;
        z = z_;
    }

    Point(const Point &pt){
        x = pt.x;
        y = pt.y;
        z = pt.z;
    }

    std::vector<double> vectorize(){
        return std::vector<double>({x, y, z});
    }
};

struct Quaternion{
    double x;
    double y;
    double z;
    double w;

    Quaternion(double x_=0, double y_=0, double z_=0, double w_=1){
        x = x_;
        y = y_;
        z = z_;
        w = w_;
    }

    Quaternion(const Quaternion &quat){
        x = quat.x;
        y = quat.y;
        z = quat.z;
        w = quat.w;
    }

    std::vector<double> vectorize(){
        return std::vector<double>({x, y, z, w});
    }
};
    
enum class NodeType
{
    ObjectNode,
    ConceptNode
};

// Define vector of pairs
// Usage: VecPair<int, float>
template<typename T1, typename T2>
using VecPair = std::vector<std::pair<T1, T2> >;


std::ostream &operator<<(std::ostream&, const NodeType);

std::ostream &operator<<(std::ostream&, const Quaternion);

std::ostream &operator<<(std::ostream&, const Point);

} // end of namespace pgm

#endif