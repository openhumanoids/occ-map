#ifndef __OCC_MAP_PIXEL_MAP_HPP__
#define __OCC_MAP_PIXEL_MAP_HPP__

#include <lcmtypes/occ_map_pixel_map_t.h>
#include <zlib.h>
#include <math.h>
#include <assert.h>
#include <fstream>

namespace occ_map {

template<class T>
class PixelMap {
public:
  double xy0[2], xy1[2];
  double metersPerPixel;
  T* data;
  int dimensions[2];
  int num_cells;
  occ_map_pixel_map_t * msg;

  /*
   * normal constructor
   */
  PixelMap<T> (const double _xy0[2], const double _xy1[2], double mPP, T initValue = 0, bool allocate_data = true) :
    metersPerPixel(mPP), msg(NULL), data(NULL)
  {
    // make bottom right align with pixels
    xy0[0] = floor((1.0 / metersPerPixel) * _xy0[0]) * metersPerPixel;
    xy0[1] = floor((1.0 / metersPerPixel) * _xy0[1]) * metersPerPixel;

    //memcpy(xy0, _xy0, 2 * sizeof(double));
    //memcpy(xy1, _xy1, 2 * sizeof(double));

    dimensions[0] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[0] - xy0[0])) / 100); //multiply by 100 and take floor to avoid machine
    dimensions[1] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[1] - xy0[1])) / 100); //precision causing different sized maps

    //make top right align with pixels
    xy1[0] = xy0[0] + dimensions[0] * metersPerPixel;
    xy1[1] = xy0[1] + dimensions[1] * metersPerPixel;

    if (dimensions[0] <= 0 || dimensions[1] < 0) {
      printf("ERROR:dimensions[0] or dimensions[1] is less than 0\n");
      exit(1);
    }
    num_cells = dimensions[0] * dimensions[1];
    if (allocate_data) {
      data = (T *) malloc(num_cells * sizeof(T));
      reset(initValue);
    }
  }

  /*
   * Copy Constructor
   */
  template<class F>
  PixelMap<T> (const PixelMap<F> * to_copy, T(*transformFunc)(F) = NULL) :
    msg(NULL), metersPerPixel(to_copy->metersPerPixel), data(NULL)
  {
    memcpy(xy0, to_copy->xy0, 2 * sizeof(double));
    memcpy(xy1, to_copy->xy1, 2 * sizeof(double));

    memcpy(dimensions, to_copy->dimensions, 2 * sizeof(int));
    num_cells = to_copy->num_cells;
    data = (T *) malloc(num_cells * sizeof(T));
    int ixy[2];
    for (ixy[1] = 0; ixy[1] < dimensions[1]; ixy[1]++) {
      for (ixy[0] = 0; ixy[0] < dimensions[0]; ixy[0]++) {
        if (transformFunc != NULL)
          writeValue(ixy, transformFunc(to_copy->readValue(ixy)));
        else
          writeValue(ixy, to_copy->readValue(ixy));
      }
    }
  }

  /*
   * Constructor from a message
   */
  PixelMap<T> (const occ_map_pixel_map_t * _msg) :
    msg(NULL), data(NULL)
  {
    set_from_pixel_map_t(_msg);
  }

  /*
   * Constructor from a file (created with "saveToFile")
   */
  PixelMap<T> (const char * name) :
    msg(NULL), data(NULL)
  {
    std::ifstream ifs(name, std::ios::binary);
    int sz;
    ifs >> sz;
    char * data = (char *) malloc(sz * sizeof(char));
    ifs.read(data, sz * sizeof(char));
    ifs.close();
    occ_map_pixel_map_t tmpmsg;
    occ_map_pixel_map_t_decode(data, 0, sz, &tmpmsg);
    set_from_pixel_map_t(&tmpmsg);
    occ_map_pixel_map_t_decode_cleanup(&tmpmsg);
    free(data);
  }

  ~PixelMap<T> ()
  {
    if (data != NULL)
      free(data);
  }

  void reset(T resetVal = 0)
  {
    if (data != NULL)
      for (int i = 0; i < num_cells; i++)
        data[i] = resetVal;
  }

  inline int getInd(const int ixy[2]) const
  {
    return ixy[1] * dimensions[0] + ixy[0];
  }

  inline void indToLoc(int ind, int ixy[2]) const
  {
    ixy[1] = ind / (dimensions[0]);
    ind -= ixy[1] * dimensions[0];
    ixy[0] = ind;
  }

  inline void worldToTable(const double xy[2], int ixy[2]) const
  {
    ixy[0] = clamp_value(round((xy[0] - xy0[0]) / metersPerPixel), 0., (double) (dimensions[0] - 1));
    ixy[1] = clamp_value(round((xy[1] - xy0[1]) / metersPerPixel), 0., (double) (dimensions[1] - 1));
  }

  inline void tableToWorld(const int ixy[2], double xy[2]) const
  {
    //    *xy[0] = ((double)ixy[0]+0.5) * metersPerPixel + xy0[0]; //+.5 puts it in the center of the cell
    //    *xy[1] = ((double)ixy[1]+0.5) * metersPerPixel + xy0[1];
    xy[0] = ((double) ixy[0]) * metersPerPixel + xy0[0];
    xy[1] = ((double) ixy[1]) * metersPerPixel + xy0[1];

  }
  inline bool isInMap(int ixy[2]) const
  {
    if (ixy[0] < 0 || ixy[1] < 0)
      return false;
    else if (ixy[0] >= dimensions[0] - 1 || ixy[0] >= dimensions[1] - 1)
      return false;
    else
      return true;
  }

  inline bool isInMap(double xy[2]) const
  {
    if (xy[0] <= xy0[0] || xy[0] >= xy1[0])
      return false;
    else if (xy[1] <= xy0[1] || xy[1] >= xy1[1])
      return false;
    else
      return true;
  }

  inline T readValue(const int ixy[2]) const
  {
    int ind = getInd(ixy);
    return data[ind];

  }
  inline T readValue(const double xy[2]) const
  {
    int ixy[2];
    worldToTable(xy, ixy);
    return readValue(ixy);
  }
  inline void writeValue(const int ixy[2], T val)
  {
    int ind = getInd(ixy);
    data[ind] = val;
  }

  inline void writeValue(const double xy[2], T val)
  {
    int ixy[2];
    worldToTable(xy, ixy);
    writeValue(ixy, val);
  }

  inline void updateValue(const int ixy[2], T inc, T clamp_bounds[2] = NULL)
  {
    int ind = getInd(ixy);
    data[ind] += inc;
    if (clamp_bounds != NULL) {
      data[ind] = clamp_value(data[ind], clamp_bounds[0], clamp_bounds[1]);
    }
  }

  inline void updateValue(const double xy[2], T inc, T clamp_bounds[2] = NULL)
  {
    int ixy[2];
    worldToTable(xy, ixy);
    updateValue(ixy, inc, clamp_bounds);
  }

  inline void rayTrace(const double start[2], const double end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL)
  {
    int istart[2], iend[2];
    worldToTable(start, istart);
    worldToTable(end, iend);
    rayTrace(istart, iend, miss_inc, hit_inc, clamp_bounds);
  }

  /**
   * This function adapted from the Python Imaging Library
   */
  void rayTrace(const int start[2], const int end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL)
  {
    int curr[2] = { start[0], start[1] };

    // normalize
    int xstep = 1;
    int ystep = 1;
    int dx = end[0] - start[0];
    if (dx < 0) {
      dx = -dx;
      xstep = -1;
    }
    int dy = end[1] - start[1];
    if (dy < 0) {
      dy = -dy;
      ystep = -1;
    }

    if (dx == 0) {
      // vertical
      for (int i = 0; i <= dy; i++) {
        int wasHit = curr[0] == end[0] && curr[1] == end[1];
        updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
        curr[1] = curr[1] + ystep;
      }
    }
    else if (dy == 0) {
      // horizontal
      for (int i = 0; i <= dx; i++) {
        int wasHit = curr[0] == end[0] && curr[1] == end[1];
        updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
        curr[0] += xstep;
      }
    }
    else if (dx > dy) {
      // bresenham, horizontal slope
      int n = dx;
      dy += dy;
      int e = dy - dx;
      dx += dx;

      for (int i = 0; i <= n; i++) {
        int wasHit = curr[0] == end[0] && curr[1] == end[1];
        updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
        if (e >= 0) {
          curr[1] += ystep;
          e -= dx;
        }
        e += dy;
        curr[0] += xstep;
      }
    }
    else {
      // bresenham, vertical slope
      int n = dy;
      dx += dx;
      int e = dx - dy;
      dy += dy;

      for (int i = 0; i <= n; i++) {
        int wasHit = curr[0] == end[0] && curr[1] == end[1];
        updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
        if (e >= 0) {
          curr[0] += xstep;
          e -= dy;
        }
        e += dx;
        curr[1] += ystep;
      }
    }
  }

  const occ_map_pixel_map_t *get_pixel_map_t(int64_t utime)
  {
    if (msg == NULL)
      msg = (occ_map_pixel_map_t*) calloc(1, sizeof(occ_map_pixel_map_t));
    memcpy(msg->xy0, xy0, 2 * sizeof(double));
    memcpy(msg->xy1, xy1, 2 * sizeof(double));
    msg->mpp = metersPerPixel;
    memcpy(msg->dimensions, dimensions, 2 * sizeof(int));

    uLong uncompressed_size = num_cells * sizeof(T);

    uLong compress_buf_size = uncompressed_size * 1.01 + 12; //with extra space for zlib
    msg->mapData = (uint8_t *) realloc(msg->mapData, compress_buf_size);
    int compress_return = compress2((Bytef *) msg->mapData, &compress_buf_size, (Bytef *) data, uncompressed_size,
        Z_BEST_SPEED);
    if (compress_return != Z_OK) {
      fprintf(stderr, "ERROR: Could not compress voxel map!\n");
      exit(1);
    }
    //    fprintf(stderr, "uncompressed_size=%ld compressed_size=%ld\n", uncompressed_size, compress_buf_size);
    msg->datasize = compress_buf_size;
    msg->compressed = 1;
    msg->utime = utime;
    return msg;

  }
  void set_from_pixel_map_t(const occ_map_pixel_map_t * _msg)
  {
    memcpy(xy0, _msg->xy0, 2 * sizeof(double));
    memcpy(xy1, _msg->xy1, 2 * sizeof(double));
    metersPerPixel = _msg->mpp;
    memcpy(dimensions, _msg->dimensions, 2 * sizeof(int));

    num_cells = dimensions[0] * dimensions[1];
    uLong uncompressed_size = num_cells * sizeof(T);
    data = (T*) realloc(data, uncompressed_size);
    if (_msg->compressed) {
      uLong uncompress_size_result = uncompressed_size;
      uLong uncompress_return = uncompress((Bytef *) data, (uLong *) &uncompress_size_result, (Bytef *) _msg->mapData,
          (uLong) _msg->datasize);
      if (uncompress_return != Z_OK || uncompress_size_result != uncompressed_size) {
        fprintf(stderr, "ERROR uncompressing the map, ret = %lu\n", uncompress_return);
        exit(1);
      }
    }
    else {
      assert((uLong)_msg->datasize == uncompressed_size);
      memcpy(data, _msg->mapData, uncompressed_size);
    }
  }

  void saveToFile(const char * name)
  {
    const occ_map_pixel_map_t * msg = get_pixel_map_t(0);
    int sz = occ_map_pixel_map_t_encoded_size(msg);
    char * buf = (char *) malloc(sz * sizeof(char));
    occ_map_pixel_map_t_encode(buf, 0, sz, msg);
    std::ofstream ofs(name, std::ios::binary);
    ofs << sz;
    ofs.write(buf, sz);
    ofs.close();
    free(buf);
  }

  inline T clamp_value(T x, T min, T max) const
  {
    if (x < min)
      return min;
    if (x > max)
      return max;
    return x;
  }
};

//typedefs for ease of use
typedef PixelMap<float> FloatPixelMap;
typedef PixelMap<int32_t> IntPixelMap;

}

#endif /*GRIDMAP_H_*/
