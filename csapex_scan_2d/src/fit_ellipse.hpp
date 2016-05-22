#ifndef FIT_ELLIPSE_HPP
#define FIT_ELLIPSE_HPP

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace fit_ellipse {
class Ellipse {
public:
    typedef Eigen::Vector2d           Point;
    typedef std::pair<double, double> Lengths;

    Ellipse() :
        size(0),
        C(Eigen::MatrixXd::Zero(6,6)),
        a_bar(Eigen::VectorXd::Zero(6))
    {
        C(0,2) =  2;
        C(2,0) =  2;
        C(1,1) = -1;
    }

    inline void addPoint(const Point &_p)
    {
        points.emplace_back(_p);
        ++size;
    }


    inline void addPoints(const std::vector<Point> &_points)
    {
        points.insert(points.end(), _points.begin(), _points.end());
        size += _points.size();
    }

    inline void compute()
    {
        D = Eigen::MatrixXd::Ones(points.size(), 6);
        for(std::size_t i = 0 ; i < size ; ++i) {
            const Point &p = points[i];
            D(i,0) = p(0) * p(0);
            D(i,1) = p(0) * p(1);
            D(i,2) = p(1) * p(1);
            D(i,3) = p(0);
            D(i,4) = p(1);
        }

        S = D.transpose() * D;
        Eigen::EigenSolver<Eigen::MatrixXd> eigen(S.inverse() * C);
        eigen_vectors = eigen.eigenvectors().real();
        eigen_values  = eigen.eigenvalues().real();

        argmax();
        a = eigen_vectors.col(n);
        a_bar(0) = a(0);
        a_bar(1) = a(1) * .5;
        a_bar(2) = a(2);
        a_bar(3) = a(3) * .5;
        a_bar(4) = a(4) * .5;
        a_bar(5) = a(5);


        /// center point
        double norm = a_bar(1) * a_bar(1) - a_bar(0) * a_bar(2);
        center(0) = (a_bar(2) * a_bar(3) - a_bar(1) * a_bar(4)) / norm;
        center(1) = (a_bar(0) * a_bar(4) - a_bar(1) * a_bar(3)) / norm;
        /// main axis rotation
        if(a_bar(1) == 0) {
            if(a_bar(0) > a_bar(2)){
                rotation_main_axis = 0.0;
            } else {
                rotation_main_axis = M_PI_2;
            }
        } else {
            if(a_bar(0) > a_bar(2)) {
                rotation_main_axis = atan2(2*a_bar(1),(a_bar(0) - a_bar(2))) * 0.5;
            } else {
                rotation_main_axis = M_PI_2 + atan2(2*a_bar(1), a_bar(0) - a_bar(2)) * 0.5;
            }
        }
        /// axis lengths
        double up = 2*(a_bar(0)*a_bar(4)*a_bar(4)+a_bar(2)*a_bar(3)*a_bar(3)+a_bar(5)*a_bar(1)*a_bar(1)-2*a_bar(1)*a_bar(3)*a_bar(4)-a_bar(0)*a_bar(2)*a_bar(5));
        double down_1=(a_bar(1)*a_bar(1)-a_bar(0)*a_bar(2))*( (a_bar(2)-a_bar(0))*sqrt(1+4*a_bar(1)*a_bar(1)/((a_bar(0)-a_bar(2))*(a_bar(0)-a_bar(2))))-(a_bar(2)+a_bar(0)));
        double down_2=(a_bar(1)*a_bar(1)-a_bar(0)*a_bar(2))*( (a_bar(0)-a_bar(2))*sqrt(1+4*a_bar(1)*a_bar(1)/((a_bar(0)-a_bar(2))*(a_bar(0)-a_bar(2))))-(a_bar(2)+a_bar(0)));

        length_first_axis = sqrt(fabs(up) / fabs(down_1));
        length_second_axis = sqrt(fabs(up) / fabs(down_2));
    }

    inline Point getCenter() const
    {
        return center;
    }

    inline double getRotation() const
    {
        return rotation_main_axis;
    }

    inline Lengths getAxisLengths() const
    {
        return std::make_pair(length_first_axis, length_second_axis);
    }


private:
    std::size_t        size;
    std::vector<Point> points;

    Eigen::MatrixXd    D;
    Eigen::MatrixXd    S;
    Eigen::MatrixXd    C;
    Eigen::MatrixXd    eigen_vectors;
    Eigen::MatrixXd    eigen_values;
    std::size_t        n;
    Eigen::VectorXd    a;
    Eigen::VectorXd    a_bar;

    Point               center;
    double              length_first_axis;
    double              length_second_axis;
    double              rotation_main_axis;

    inline void argmax()
    {
        n = 0;
        double max = std::numeric_limits<double>::lowest();
        for(std::size_t i = 0 ;  i < 6 ; ++i) {
            if(fabs(eigen_values(i) >  max)) {
                n = i ;
                max = fabs(eigen_values(i));
            }
        }
    }


};
}
#endif // FIT_ELLIPSE_HPP
