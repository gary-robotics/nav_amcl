//
// Created by ethan on 19-1-26.
//
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include "nav_pf.h"
#include "pf_vector.h"
#include "pf_pdf.h"
#include "pf_kdtree.h"

pf_t* pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data)//函数指针的用法，将来调用时会传进来一个返回值和参数表都符合的函数的指针
{
    int i, j;
    pf_t *pf;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    pf =(pf_t *) calloc(1, sizeof(pf_t));//calloc相比malloc，它会将分配的内存初始化为0
    //pf =(pf_t *)malloc(sizeof(pf_t));


    pf->random_pose_fn = random_pose_fn;//指定它的随机采样函数是什么

    pf->min_samples = min_samples;
    pf->max_samples = max_samples;

    pf->dist_threshold = 0.5;

    //为粒子开辟空间并进行位姿和权重的初始化
    pf->set=new pf_sample_set_t;//*********给set分配空间，即给它一个地址
    pf->set->sample_count = max_samples;
    pf->set->samples = (pf_sample_t*)calloc(max_samples, sizeof(pf_sample_t));
    for (i = 0; i < pf->set->sample_count; i++)
    {
        sample = pf->set->samples + i;
        sample->pose.v[0] = 0.0;
        sample->pose.v[1] = 0.0;
        sample->pose.v[2] = 0.0;
        sample->weight = 1.0 / max_samples;
    }
    pf->set->kdtree = pf_kdtree_alloc(3 * max_samples);//为粒子开辟空间，以kdtree的形式存放

    pf->set->cluster_count = 0;
    pf->set->cluster_max_count = max_samples;
    pf->set->clusters = (pf_cluster_t*)calloc(pf->set->cluster_max_count, sizeof(pf_cluster_t));

    pf->set->mean = pf_vector_zero();
    pf->set->cov = pf_matrix_zero();

    pf->w_slow = 0.0;
    pf->w_fast = 0.0;

    pf->alpha_slow = alpha_slow;
    pf->alpha_fast = alpha_fast;

    pf->set->converged=0;
    pf->converged=0;

    return pf;
}

void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)//初始化滤波器 LX
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_pdf_gaussian_t *pdf;

    set = pf->set;


    set->sample_count = pf->max_samples;//@TODO:为什么是最大值
    pf_kdtree_clear(set->kdtree);// 创建一个空的kdtree
    // 按照给定的均值和方差创建高斯pdf,并将协方差矩阵分解为特征值和特征向量
    pdf = pf_pdf_gaussian_alloc(mean, cov);
    // 采样
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = 1.0 / pf->max_samples;
        sample->pose = pf_pdf_gaussian_sample(pdf);//高斯采样
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);//将粒子放入kdtree，因为kdtree方便查找离得近的粒子，便于统计
    }

    pf->w_slow = pf->w_fast = 0.0;

    free(pdf);//删除高斯pdf占用的空间

    // Re-compute cluster statistics
    pf_cluster_stats(pf, set);

    //set converged to 0
    pf_init_converged(pf);

    return;
}

void pf_init_converged(pf_t *pf){
    pf_sample_set_t *set;
    set = pf->set;
    set->converged = 0;
    pf->converged = 0;
}

//计算每个簇的统计，和所有粒子的整体统计
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
    int i, j, k, cidx;
    pf_sample_t *sample;
    pf_cluster_t *cluster;

    // Workspace
    double m[4], c[2][2];
    size_t count;
    double weight;

    // Cluster the samples 给簇里的粒子做标记
    pf_kdtree_cluster(set->kdtree);

    // Initialize cluster stats
    set->cluster_count = 0;

    for (i = 0; i < set->cluster_max_count; i++)
    {
        cluster = set->clusters + i;
        cluster->count = 0;
        cluster->weight = 0;
        cluster->mean = pf_vector_zero();
        cluster->cov = pf_matrix_zero();

        for (j = 0; j < 4; j++)
            cluster->m[j] = 0.0;
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
                cluster->c[j][k] = 0.0;
    }

    // Initialize overall filter stats
    count = 0;
    weight = 0.0;
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
    for (j = 0; j < 4; j++)
        m[j] = 0.0;
    for (j = 0; j < 2; j++)
        for (k = 0; k < 2; k++)
            c[j][k] = 0.0;

    // Compute cluster stats
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;

        //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

        // Get the cluster label for this sample
        cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);//看它是哪个簇的
        assert(cidx >= 0);
        if (cidx >= set->cluster_max_count)
            continue;
        if (cidx + 1 > set->cluster_count)
            set->cluster_count = cidx + 1;

        cluster = set->clusters + cidx;//找到这个簇

        cluster->count += 1;
        cluster->weight += sample->weight;//簇的权重=粒子权重值和

        count += 1;
        weight += sample->weight;

        // Compute mean 某一簇的位姿均值：Σ(粒子的位姿×权重)/权重之和
        cluster->m[0] += sample->weight * sample->pose.v[0];//x的均值
        cluster->m[1] += sample->weight * sample->pose.v[1];//y的
        cluster->m[2] += sample->weight * cos(sample->pose.v[2]);//*********cos（角度）的均值
        cluster->m[3] += sample->weight * sin(sample->pose.v[2]);
        // 所有粒子的位姿均值
        m[0] += sample->weight * sample->pose.v[0];
        m[1] += sample->weight * sample->pose.v[1];
        m[2] += sample->weight * cos(sample->pose.v[2]);
        m[3] += sample->weight * sin(sample->pose.v[2]);

        // Compute covariance in linear components 计算协方差 step1
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
            {
                cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
                c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
            }
    }

    // Normalize
    for (i = 0; i < set->cluster_count; i++)
    {
        cluster = set->clusters + i;

        cluster->mean.v[0] = cluster->m[0] / cluster->weight;
        cluster->mean.v[1] = cluster->m[1] / cluster->weight;
        cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);//*****没有/权重之和，因为分子分母抵消了

        cluster->cov = pf_matrix_zero();

        // Covariance in linear components  计算协方差 step2
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
                cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -//【【【为什么减掉这一项？】】】
                                       cluster->mean.v[j] * cluster->mean.v[k];

        // Covariance in angular components; I think this is the correct //【【【？？？】】】
        // formula for circular statistics.
        cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                             cluster->m[3] * cluster->m[3]));

        //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
        //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
        //pf_matrix_fprintf(cluster->cov, stdout, "%e");
    }

    // Compute overall filter stats 所有粒子的均值
    set->mean.v[0] = m[0] / weight;
    set->mean.v[1] = m[1] / weight;
    set->mean.v[2] = atan2(m[3], m[2]);

    // Covariance in linear components
    for (j = 0; j < 2; j++)
        for (k = 0; k < 2; k++)
            set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

    return;
}