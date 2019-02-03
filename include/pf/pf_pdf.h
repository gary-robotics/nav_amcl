//
// Created by ethan on 19-1-30.
//

#ifndef NAV_AMCL_PF_PDF_H
#define NAV_AMCL_PF_PDF_H

#include "pf_vector.h"

// 一个高斯分布
typedef struct
{
    // 均值和方差
    pf_vector_t x;
    pf_matrix_t cx;
    // 分解成的旋转矩阵（特征向量组成）和特征值（方差组成）
    pf_matrix_t cr;
    pf_vector_t cd;
} pf_pdf_gaussian_t;

// 创建一个高斯分布（就是将协方差矩阵分解为旋转矩阵和一个对角矩阵）
pf_pdf_gaussian_t *pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx);

// 以sigma为方差，0为均值采样
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma);

// 根据高斯分布采样
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t *pdf);

#endif //NAV_AMCL_PF_PDF_H
