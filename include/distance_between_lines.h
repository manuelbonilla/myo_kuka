#ifndef DISTANCE_BETWEEN_LINES_H
#define DISTANCE_BETWEEN_LINES_H


#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <algorithm>

class LineCollisions
{

public:
  LineCollisions(){};
  ~LineCollisions(){};

  typedef Eigen::Vector3d Point;
  struct Line{
    Point P1, P2;
    double norm;
    Line(){};
    Line(Point P1_in, Point P2_in){P1=P1_in; P2=P2_in; norm = (P2-P1).norm();};
    bool operator<(const Line& other)
    {
      return norm < other.norm;
    }
  };

  double getPointDistance(Point P1, Point P2){return (P1 - P2).norm();};
  Line getClosestPoints(
    Line line1, Line line2);

};

    // This part computer the minimum distance between extremal point
    // std::list<LineCollisions::Line> list_lines;
    // std::vector<LineCollisions::Line> v_dist{ LineCollisions::Line(L1.P1, L2.P1),
    //                                      LineCollisions::Line(L1.P1, L2.P2),
    //                                      LineCollisions::Line(L1.P2, L2.P1),
    //                                      LineCollisions::Line(L1.P2, L2.P2) };
    // std::vector< LineCollisions::Line >::iterator result = std::min_element(std::begin(v_dist), std::end(v_dist));
    // line_out = v_dist[ std::distance(std::begin(v_dist), result) ];
    // return line_out;


#endif