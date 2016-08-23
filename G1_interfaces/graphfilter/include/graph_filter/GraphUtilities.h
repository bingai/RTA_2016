#ifndef GRAPHUTILITIES_H
#define GRAPHUTILITIES_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace Graph {

namespace Util {
  Eigen::Vector3f rotateDir(Eigen::Vector3f, Eigen::Matrix3f);
  
  std::vector<std::pair<int, int> > imageNgbr(std::pair<int, int> ind, 
    int x_cols, int y_rows);

  std::vector<std::pair<int, int> > imageLeftUpNgbr(std::pair<int, int> ind, 
    int x_cols, int y_rows);

  bool isValidCoord(int x, int y, int x_cols, int y_rows);

  float getVectorsMin(const std::vector<float> & v);

  float getVectorsMax(const std::vector<float> & v);

  typedef union
  {
    struct
    {
      unsigned char b; // Blue channel
      unsigned char g; // Green channel
      unsigned char r; // Red channel
      unsigned char a; // Alpha channel
    };
    float float_value;
    long long_value;
   } RGBValue;

   float  GetRandomColor();

}
}

#endif