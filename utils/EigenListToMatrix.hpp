/*
 * from https://github.com/libigl/libigl/blob/master/include/igl/list_to_matrix.cpp
 * by R. Falque
 * 03/07/2019
 */

#ifndef EIGEN_LIST_TO_MATRIX_HPP
#define EIGEN_LIST_TO_MATRIX_HPP

#include <iostream>
#include <Eigen/Core>
#include <vector>

template <typename T, typename Derived>
inline bool list_to_matrix(const std::vector<std::vector<T > > & V,Eigen::PlainObjectBase<Derived>& M)
{
  // number of rows
  int m = V.size();
  if(m == 0)
  {
    M.resize(0,0);
    return true;
  }
  // number of columns
  
  /* BEGIN OF EDIT: REMOVE CHECK FOR ALL ROWS HAVING THE SAME VALUE */
  int n = V[0].size();
  /* END OF EDIT */

  assert(n != -1);
  // Resize output
  M.resize(m,n);

  // Loop over rows
  for(int i = 0;i<m;i++)
  {
    // Loop over cols
    for(int j = 0;j<n;j++)
    {
      M(i,j) = V[i][j];
    }
  }

  return true;
};

template <typename T, typename Derived>
inline bool list_to_matrix(
  const std::vector<std::vector<T > > & V,
  const int n,
  const T & padding,
  Eigen::PlainObjectBase<Derived>& M)
{
  const int m = V.size();
  M.resize(m,n);
  for(int i = 0;i<m;i++)
  {
    const auto & row = V[i];
    if(row.size()>n)
    {
      return false;
    }
    int j = 0;
    for(;j<row.size();j++)
    {
      M(i,j) = row[j];
    }
    for(;j<n;j++)
    {
      M(i,j) = padding;
    }
  }
  return true;
};

template <typename T, typename Derived>
inline bool list_to_matrix(const std::vector<T > & V,Eigen::PlainObjectBase<Derived>& M)
{
  // number of rows
  int m = V.size();
  if(m == 0)
  {
    //fprintf(stderr,"Error: list_to_matrix() list is empty()\n");
    //return false;
    if(Derived::ColsAtCompileTime == 1)
    {
      M.resize(0,1);
    }else if(Derived::RowsAtCompileTime == 1)
    {
      M.resize(1,0);
    }else
    {
      M.resize(0,0);
    }
    return true;
  }
  // Resize output
  if(Derived::RowsAtCompileTime == 1)
  {
    M.resize(1,m);
  }else
  {
    M.resize(m,1);
  }

  // Loop over rows
  for(int i = 0;i<m;i++)
  {
    M(i) = V[i];
  }

  return true;
};

#endif
