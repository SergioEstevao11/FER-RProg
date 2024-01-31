#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


int main() {
    Eigen::MatrixXd m1(5, 5);
    Eigen::MatrixXd m2(5, 5);
    Eigen::VectorXd v1(5);

    Eigen::RowVectorXd jmbagInitials(5);
    jmbagInitials << 0, 0, 3, 6, 5;

    m1 = jmbagInitials.colwise().replicate(5);
    m2 = m1 + Eigen::MatrixXd::Identity(5, 5);
    v1 << 6, 3, 0, 3,5;

    std::cout << "Matrix m1:\n" << m1 << "\n\n";
    std::cout << "Matrix m2:\n" << m2 << "\n\n";
    std::cout << "Vector v1:\n" << v1 << "\n\n";

    Eigen::VectorXd r1 = m2 * v1;
    std::cout << "Result of m2 * v1:\n" << r1.transpose() << "\n\n";
    Eigen::MatrixXd r2 = v1 * v1.transpose();
    std::cout << "Result of v1 * v1^T:\n" << r2 << "\n\n";
    Eigen::MatrixXd r3 = m1 + m2;
    std::cout << "Sum of m1 and m2:\n" << r3 << "\n\n";
    Eigen::MatrixXd r4 = m2.inverse();
    std::cout << "Inverse of m2:\n" << r4 << "\n\n";
    Eigen::MatrixXd r5 = m2.transpose();
    std::cout << "Transpose of m2:\n" << r5 << "\n\n";

    return 0;
}
