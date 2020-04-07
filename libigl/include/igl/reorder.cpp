// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "reorder.h"
#include "SortableRow.h"
#ifndef IGL_NO_EIGEN
#include <Eigen/Core>
#endif

// This implementation is O(n), but also uses O(n) extra memory
template< class T >
IGL_INLINE void igl::reorder(
  const std::vector<T> & unordered,
  std::vector<size_t> const & index_map,
  std::vector<T> & ordered)
{
  // copy for the reorder according to index_map, because unsorted may also be
  // sorted
  std::vector<T> copy = unordered;
  ordered.resize(index_map.size());
  for(int i = 0; i<(int)index_map.size();i++)
  {
    ordered[i] = copy[index_map[i]];
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template void igl::reorder<double>(std::vector<double, std::allocator<double> > const&, std::vector<size_t, std::allocator<size_t> > const&, std::vector<double, std::allocator<double> >&);
template void igl::reorder<int>(std::vector<int, std::allocator<int> > const&, std::vector<size_t, std::allocator<size_t> > const&, std::vector<int, std::allocator<int> >&);
#  ifndef IGL_NO_EIGEN
  template void igl::reorder<igl::SortableRow<Eigen::Matrix<int, -1, 1, 0, -1, 1> > >(std::vector<igl::SortableRow<Eigen::Matrix<int, -1, 1, 0, -1, 1> >, std::allocator<igl::SortableRow<Eigen::Matrix<int, -1, 1, 0, -1, 1> > > > const&, std::vector<size_t, std::allocator<size_t> > const&, std::vector<igl::SortableRow<Eigen::Matrix<int, -1, 1, 0, -1, 1> >, std::allocator<igl::SortableRow<Eigen::Matrix<int, -1, 1, 0, -1, 1> > > >&);
  template void igl::reorder<igl::SortableRow<Eigen::Matrix<double, -1, 1, 0, -1, 1> > >(std::vector<igl::SortableRow<Eigen::Matrix<double, -1, 1, 0, -1, 1> >, std::allocator<igl::SortableRow<Eigen::Matrix<double, -1, 1, 0, -1, 1> > > > const&, std::vector<size_t, std::allocator<size_t> > const&, std::vector<igl::SortableRow<Eigen::Matrix<double, -1, 1, 0, -1, 1> >, std::allocator<igl::SortableRow<Eigen::Matrix<double, -1, 1, 0, -1, 1> > > >&);
#  endif
template void igl::reorder<long>(std::vector<long, std::allocator<long> > const&, std::vector<size_t, std::allocator<size_t> > const&, std::vector<long, std::allocator<long> >&);
#endif
