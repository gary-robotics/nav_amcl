//
// Created by ethan on 19-1-30.
//
#include "nav_pf.h"
#include "nav_eigen.h"
pf_vector_t pf_vector_zero()
{
    pf_vector_t c;
    c.v[0] = 0.0;
    c.v[1] = 0.0;
    c.v[2] = 0.0;
    return c;
}

pf_matrix_t pf_matrix_zero()
{
    int i, j;
    pf_matrix_t c;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            c.m[i][j] = 0.0;

    return c;
}

// Decompose a covariance matrix [a] into a rotation matrix [r] and a diagonal
// matrix [d] such that a = r d r^T.
void pf_matrix_unitary(pf_matrix_t *r, pf_matrix_t *d, pf_matrix_t a)
{

    // 将协方差矩阵先分解为特征值和特征向量
    int i, j;

    double aa[3][3];
    double eval[3];
    double evec[3][3];

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            aa[i][j] = a.m[i][j];
        }
    }

    eigen_decomposition(aa,evec,eval);

    *d = pf_matrix_zero();
    for (i = 0; i < 3; i++)
    {
        d->m[i][i] = eval[i];//对角矩阵的对角线上都是特征值
        for (j = 0; j < 3; j++)
        {
            r->m[i][j] = evec[i][j];//旋转矩阵就是特征向量组成的
        }
    }

    return;
}