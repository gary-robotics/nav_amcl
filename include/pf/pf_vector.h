//
// Created by ethan on 19-1-28.
//

#ifndef NAV_AMCL_PF_VECTOR_H
#define NAV_AMCL_PF_VECTOR_H
//---------------向量-----------------
typedef struct
{
    double v[3];
} pf_vector_t;

pf_vector_t pf_vector_zero();
//---------------矩阵------------------
typedef struct
{
    double m[3][3];
} pf_matrix_t;

pf_matrix_t pf_matrix_zero();

//将矩阵分解为对角矩阵和旋转矩阵：a=rdr-T
void pf_matrix_unitary(pf_matrix_t *r, pf_matrix_t *d, pf_matrix_t a);
#endif //NAV_AMCL_PF_VECTOR_H
