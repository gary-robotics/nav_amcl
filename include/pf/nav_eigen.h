//
// Created by ethan on 19-1-30.
//

#ifndef NAV_AMCL_NAV_EIGEN_H
#define NAV_AMCL_NAV_EIGEN_H

void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);//将3*3矩阵分解为特征向量V（一列就是一个特征向量）和3个特征值d

#endif //NAV_AMCL_NAV_EIGEN_H
