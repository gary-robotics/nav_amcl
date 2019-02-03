//
// Created by ethan on 19-1-26.
//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include <signal.h>
#include <ros/assert.h>
#include "nav_map.h"
#include "nav_pf.h"


class NavAmcl{
public:
    NavAmcl();
private:
    ros::NodeHandle nh_;

    //地图相关
    ros::Subscriber map_sub;
    bool map_recieved;
    map_t* map_;
    static std::vector<std::pair<int,int> > free_space_indices;//加static是因为将会被static成员函数访问
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    //粒子滤波器
    pf_t* pf_;
    int min_particles_,max_particles_;
    double alpha_slow_,alpha_fast_;
    static pf_vector_t uniformPoseGenerator(void* arg);//这个函数会赋给某个函数指针，但这个函数是在类里的，被调用时得是调用具体的对象的函数，所以加了个static
    void updatePoseFromServer();//从参数服务器读取位姿，用来布置粒子一开始的位置
    double init_pose_[3];
    double init_cov_[3];
    bool pf_init_;
    ros::Publisher particlecloud_pub_;//发布粒子消息
};

// ********类的static变量其实相当于一个全局变量，在类里的出现其实只是声明。真正的定义需要在类外定义
std::vector<std::pair<int,int> > NavAmcl::free_space_indices;//地图的free区域

NavAmcl::NavAmcl() {
    //读取参数
    nh_.param("min_particles", min_particles_, 100);
    nh_.param("max_particles", max_particles_, 5000);
    nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
    nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
    //地图相关
    map_sub=nh_.subscribe("map", 1, &NavAmcl::mapReceived, this);
    //粒子滤波
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
}

void sigintHandler(int sig)//LX
{
    // Save latest pose as we're shutting down.
    //amcl_node_ptr->savePoseToServer();
    ros::shutdown();
}

void NavAmcl::mapReceived(const nav_msgs::OccupancyGridConstPtr &msg) {  //LX
    map_recieved=true;
    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
             msg->info.width,
             msg->info.height,
             msg->info.resolution);
    //转换为可处理的格式
    map_ = map_alloc();
    ROS_ASSERT(map_);
    map_->size_x = msg->info.width;
    map_->size_y = msg->info.height;
    map_->scale = msg->info.resolution;
    map_->origin_x = msg->info.origin.position.x + (map_->size_x / 2) * map_->scale;//将地图原点设为图片的中心
    map_->origin_y = msg->info.origin.position.y + (map_->size_y / 2) * map_->scale;
    map_->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map_->size_x*map_->size_y);
    ROS_ASSERT(map_->cells);
    for(int i=0;i<map_->size_x * map_->size_y;i++)
    {
        if(msg->data[i] == 0)
            map_->cells[i].occ_state = -1;
        else if(msg->data[i] == 100)//占据的栅格，其值就是100
            map_->cells[i].occ_state = +1;
        else
            map_->cells[i].occ_state = 0;
    }
    //记录空闲栅格的索引
    for(int i = 0; i < map_->size_x; i++)
        for(int j = 0; j < map_->size_y; j++)
            if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
                free_space_indices.push_back(std::make_pair(i,j));
    //创建一个滤波器
    pf_=pf_alloc(min_particles_,max_particles_,alpha_slow_,alpha_fast_,(pf_init_model_fn_t)NavAmcl::uniformPoseGenerator,(void*)map_);
    updatePoseFromServer();//读取初始位姿,一开始粒子就先放到这个地方附近
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = init_pose_[0];
    pf_init_pose_mean.v[1] = init_pose_[1];
    pf_init_pose_mean.v[2] = init_pose_[2];
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = init_cov_[0];
    pf_init_pose_cov.m[1][1] = init_cov_[1];
    pf_init_pose_cov.m[2][2] = init_cov_[2];
    //初始化滤波器：在某地高斯地撒点粒子
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false;//表示滤波器还没有初始化（到现在，只是把粒子随便放一个地方）
//<------- test ---------
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.poses.resize(pf_->set->sample_count);
    for(int i=0;i<pf_->set->sample_count;i++)
    {
        cloud_msg.poses[i].position.x = pf_->set->samples[i].pose.v[0];
        cloud_msg.poses[i].position.y = pf_->set->samples[i].pose.v[1];
        cloud_msg.poses[i].position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pf_->set->samples[i].pose.v[2]);//@TODO:角度转换为四元数,四元数再转换为角度???
        tf2::convert(q, cloud_msg.poses[i].orientation);
    }
    particlecloud_pub_.publish(cloud_msg);
//------->
}

void NavAmcl::updatePoseFromServer() {
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
    // Check for NAN on input from param server, #5239
    double tmp_pos;
    nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
    if(!std::isnan(tmp_pos))
        init_pose_[0] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose X position");
    nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
    if(!std::isnan(tmp_pos))
        init_pose_[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Y position");
    nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
    if(!std::isnan(tmp_pos))
        init_pose_[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Yaw");
    nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
    if(!std::isnan(tmp_pos))
        init_cov_[0] =tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance XX");
    nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
    if(!std::isnan(tmp_pos))
        init_cov_[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance YY");
    nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
    if(!std::isnan(tmp_pos))
        init_cov_[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance AA");
}

pf_vector_t
NavAmcl::uniformPoseGenerator(void* arg){
    map_t* map = (map_t*)arg;//不论传入什么样格式的地图，都转化为map_t格式
    unsigned int rand_index = drand48() * free_space_indices.size();
    std::pair<int,int> free_point = free_space_indices[rand_index];
    pf_vector_t p;
    p.v[0] = MAP_WXGX(map, free_point.first);//将点的地图坐标(格子数)转换为世界坐标(米)
    p.v[1] = MAP_WYGY(map, free_point.second);
    p.v[2] = drand48() * 2 * M_PI - M_PI;//-pi~pi
    return p;
}


std::shared_ptr<NavAmcl> nav_amcl_ptr;
int main(int argc,char** argv)
{

    ros::init(argc, argv, "nav_amcl");
    ros::NodeHandle nh;


    signal(SIGINT, sigintHandler);// 接收CTRL+C   SIGINT 是一种系统定义的信号变量#include<signal.h>，由Interrupt Key产生，通常是CTRL+C或者DELETE


    nav_amcl_ptr.reset(new NavAmcl());// 创建类对象的第二种方法：指针方法


    ros::spin();//开启回调线程// run using ROS input


    nav_amcl_ptr.reset();// 释放指针 Without this, our boost locks are not shut down nicely

    return(0);

}