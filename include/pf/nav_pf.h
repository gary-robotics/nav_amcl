//
// Created by ethan on 19-1-26.
//

#ifndef NAV_AMCL_NAV_PF_H
#define NAV_AMCL_NAV_PF_H

#include "pf_vector.h"
#include "pf_kdtree.h"
// 随机采点的函数指针
typedef pf_vector_t (*pf_init_model_fn_t) (void *init_data);//*******定义一个函数指针pf_init_model_fn_t，它指向的函数参数为 void型指针（即现在还不确定，用的时候确定），返回值为pf_vector_t

// 一个粒子
typedef struct
{
    // 位姿
    pf_vector_t pose;
    // 权重
    double weight;
} pf_sample_t;

// 粒子簇
typedef struct
{
    // 粒子数
    int count;

    // 权重之和
    double weight;

    // 粒子统计
    pf_vector_t mean;
    pf_matrix_t cov;

    // Workspace 用于临时存放计算均值(角度分成了cos和sin)、协方差的中间结果
    double m[4], c[2][2];

} pf_cluster_t;

typedef struct _pf_sample_set_t
{
    //粒子数和地址
    int sample_count;
    pf_sample_t *samples;
    pf_kdtree_t *kdtree;
    // 簇
    int cluster_count, cluster_max_count;
    pf_cluster_t *clusters;

    // 粒子的统计数据
    pf_vector_t mean;
    pf_matrix_t cov;
    int converged;
} pf_sample_set_t;

// 滤波器
typedef struct _pf_t
{
    // 粒子数限制
    int min_samples, max_samples;

    // 粒子群
    pf_sample_set_t* set;

    // 自适应采样相关参数
    double w_slow, w_fast;
    double alpha_slow, alpha_fast;

    // 随机撒点功能：初始化时会用，定位失败时也会用
    pf_init_model_fn_t random_pose_fn;//一个函数指针


    // 收敛条件：距离阈值
    double dist_threshold;
    int converged;
} pf_t;

pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data);

void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov);

//计算每个簇的统计，和所有粒子的整体统计
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set);

void pf_init_converged(pf_t *pf);
#endif //NAV_AMCL_NAV_PF_H
