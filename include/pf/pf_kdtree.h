/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

#ifdef INCLUDE_RTKGUI
#include "rtk.h"

#endif

#include "pf_vector.h"

// Info for a node in the tree
typedef struct pf_kdtree_node
{
  // Depth in the tree
  int leaf, depth;//是叶子吗？有多深？

  // Pivot dimension and value
  int pivot_dim;//比较哪一维
  double pivot_value;//比较基准

  // The key for this node
  int key[3];//本节点的pose值，放大为int型

  // The value for this node
  double value;//本节点的权重

  // The cluster label (leaf nodes)
  int cluster;//属于哪个簇

  // Child nodes
  struct pf_kdtree_node *children[2];//子节点们

} pf_kdtree_node_t;


// A kd tree
typedef struct
{
  // Cell size
  double size[3];//pose的放大系数

  // The root node of the tree
  pf_kdtree_node_t *root;//根节点

  // The number of nodes in the tree
  int node_count, node_max_count;
  pf_kdtree_node_t *nodes;//给树malloc的内存地址

  // The number of leaf nodes in the tree
  int leaf_count;

} pf_kdtree_t;


// Create a tree
extern pf_kdtree_t *pf_kdtree_alloc(int max_size);

// Destroy a tree
extern void pf_kdtree_free(pf_kdtree_t *self);

// Clear all entries from the tree
extern void pf_kdtree_clear(pf_kdtree_t *self);

// Insert a pose into the tree
extern void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value);

// Cluster the leaves in the tree
extern void pf_kdtree_cluster(pf_kdtree_t *self);

// Determine the probability estimate for the given pose
extern double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose);

// Determine the cluster label for the given pose
extern int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose);


#ifdef INCLUDE_RTKGUI

// Draw the tree
extern void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig);

#endif

#endif
