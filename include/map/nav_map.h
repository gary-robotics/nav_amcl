//
// Created by ethan on 19-1-26.
//

#ifndef NAV_AMCL_MAP_H
#define NAV_AMCL_MAP_H

// 一个格子的内容
typedef struct
{
    //占据状态 (-1 = free, 0 = unknown, +1 = occ)
    int occ_state;

    // 最近的占据栅格的距离
    double occ_dist;//@TODO

} map_cell_t;

// 地图结构体
typedef struct
{
    //地图原点：把原点放在图片中的哪个位置，默认0,0
    double origin_x, origin_y;

    //比例尺：m/格
    double scale;

    //尺寸:多少个格子
    int size_x, size_y;

    // 存储栅格的指针
    map_cell_t *cells;

    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double max_occ_dist;//@TODO

} map_t;

map_t* map_alloc();

// 给定一个坐标，计算它的索引
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)
#endif //NAV_AMCL_MAP_H
