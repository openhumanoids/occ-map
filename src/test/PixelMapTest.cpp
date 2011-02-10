/*
 * VoxelMapTest.cpp
 *
 *  Created on: Feb 9, 2011
 *      Author: abachrac
 */

#include <occ_map/PixelMap.hpp>

int main(int argc, char ** argv){
  double xyz0[2] = {0,0};
  double xyz1[2] = {10,10};
  double mpp = .2;
  occ_map::FloatPixelMap fvm(xyz0,xyz1,mpp,0);
}
