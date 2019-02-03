//
// Created by ethan on 19-1-30.
//
#include <stdlib.h>
#include "pf_pdf.h"
#include "pf_vector.h"
#include "math.h"
static unsigned int pf_pdf_seed;

/**************************************************************************
 * Gaussian
 *************************************************************************/

// Create a gaussian pdf 主要是1，开辟空间 2，将协方差矩阵分解为特征值和特征向量
pf_pdf_gaussian_t *pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
{
    pf_matrix_t cd;
    pf_pdf_gaussian_t *pdf;

    pdf =(pf_pdf_gaussian_t*) calloc(1, sizeof(pf_pdf_gaussian_t));

    pdf->x = x;
    pdf->cx = cx;
    //将协方差矩阵分解为特征值组成的对角矩阵d和特征向量组成的旋转矩阵r.a=rtr-T
    pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
    //将对角矩阵对角线上的值赋给向量
    pdf->cd.v[0] = sqrt(cd.m[0][0]);
    pdf->cd.v[1] = sqrt(cd.m[1][1]);
    pdf->cd.v[2] = sqrt(cd.m[2][2]);

    srand48(++pf_pdf_seed);

    return pdf;
}




// Generate a sample from the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t *pdf)
{
    int i, j;
    pf_vector_t r;
    pf_vector_t x;

    // Generate a random vector
    for (i = 0; i < 3; i++)
    {
        //r.v[i] = gsl_ran_gaussian(pdf->rng, pdf->cd.v[i]);
        r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);//按照公式，根据特征值（实际就是方差），采样。此时这个采样是0均值的，圆形分布（即没体现出协方差的作用）的
    }

    for (i = 0; i < 3; i++)
    {
        x.v[i] = pdf->x.v[i];
        for (j = 0; j < 3; j++)
            x.v[i] += pdf->cr.m[i][j] * r.v[j];//上面的采样乘以协方差的特征向量，体现了协方差的作用【【【具体还不太懂】】】
    }

    return x;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma)
{
    double x1, x2, w, r;

    do
    {
        do { r = drand48(); } while (r==0.0);
        x1 = 2.0 * r - 1.0;
        do { r = drand48(); } while (r==0.0);
        x2 = 2.0 * r - 1.0;
        w = x1*x1 + x2*x2;
    } while(w > 1.0 || w==0.0);

    return(sigma * x2 * sqrt(-2.0*log(w)/w));
}
