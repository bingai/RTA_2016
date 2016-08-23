#include <graph_filter/GraphUtilities.h>

namespace Graph {

namespace Util {
  Eigen::Vector3f rotateDir(Eigen::Vector3f vin, Eigen::Matrix3f R) {

    return R * vin;
  }

  std::vector<std::pair<int, int> > imageNgbr(std::pair<int, int> ind, int x_cols, int y_rows) {
    std::vector<std::pair<int, int> > result;
    int x = ind.first, y = ind.second;
    if (!isValidCoord(x, y, x_cols, y_rows) ) {
      std::cout << "ERROR! INPUT INDEX OUT OF IMAGE BOUNDARY" << std::endl;
      return result;
    }

    if (isValidCoord(x-1, y, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x-1, y) );
    }

    if (isValidCoord(x+1, y, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x+1, y) );
    }

    if (isValidCoord(x, y-1, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x, y-1) );
    }

    if (isValidCoord(x, y+1, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x, y+1) );
    }

    return result;
  }


  std::vector<std::pair<int, int> > imageLeftUpNgbr(std::pair<int, int> ind, int x_cols, int y_rows) {
    std::vector<std::pair<int, int> > result;
    int x = ind.first, y = ind.second;
    if (!isValidCoord(x, y, x_cols, y_rows) ) {
      std::cout << "ERROR! INPUT INDEX OUT OF IMAGE BOUNDARY" << std::endl;
      return result;
    }

    if (isValidCoord(x-1, y, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x-1, y) );
    }


    if (isValidCoord(x, y-1, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x, y-1) );
    }

    if (isValidCoord(x-1, y-1, x_cols, y_rows)) {
      result.push_back(std::pair<int, int>(x, y+1) );
    }

    return result;
  }


  bool isValidCoord(int x, int y, int x_cols, int y_rows) {
  	return !(x < 0 || x >= x_cols || y < 0 || y > y_rows);
  }


  float getVectorsMin(const std::vector<float> & v) {
    // float d1max = std::numeric_limits<float>::lowest();
    float dmin = std::numeric_limits<float>::max();
    for (std::vector<float>::const_iterator it = v.begin(); it != v.end(); ++it) {
       if (*it < dmin) {
         dmin = *it;
       }
    }
    return dmin;
  }


  float getVectorsMax(const std::vector<float> & v) {
    float dmax = std::numeric_limits<float>::lowest();
    for (std::vector<float>::const_iterator it = v.begin(); it != v.end(); ++it) {
       if (*it > dmax) {
         dmax = *it;
       }
    }
    return dmax;
  }


  float  GetRandomColor()
  {
    RGBValue x;
    x.b = std::rand()%255;
    x.g = std::rand()%255;
    x.r = std::rand()%255;
    x.a = 0.; 
    return x.float_value;
  };




}


}
