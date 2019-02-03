//
// Created by ethan on 19-1-26.
//
#include <stdlib.h>
#include "nav_map.h"
map_t* map_alloc()
{
    map_t* map=(map_t*) malloc(sizeof(map_t));
    map->origin_x=0;
    map->origin_y=0;
    map->size_x=0;
    map->size_y=0;
    map->scale=0;
    map->cells=(map_cell_t*)NULL;
    return map;
}