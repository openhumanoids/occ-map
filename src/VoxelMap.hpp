#ifndef __OCC_MAP_VOXELMAP_HPP__
#define __OCC_MAP_VOXELMAP_HPP__

#include <lcmtypes/occ_map_voxel_map_t.h>
#include <zlib.h>
#include <math.h>
#include <fstream>
#include <assert.h>

namespace occ_map {

template<class T>
class VoxelMap {
public:
  //metadata
  double xyz0[3], xyz1[3];
  double metersPerPixel[3];
  int dimensions[3];
  //storage arrays
  int num_cells;
  T * data;
  occ_map_voxel_map_t *msg;

  VoxelMap<T> (const double _xyz0[3], const double _xyz1[3], const double _metersPerPixel[3], T initValue = T(),
      bool allocate_data = true) :
    data(NULL), msg(NULL)
  {
    memcpy(xyz0, _xyz0, 3 * sizeof(double));
    memcpy(xyz1, _xyz1, 3 * sizeof(double));
    memcpy(metersPerPixel, _metersPerPixel, 3 * sizeof(double));

    num_cells = 1;
    for (int i = 0; i < 3; i++) {
      dimensions[i] = ceil((1.0 / metersPerPixel[i]) * (xyz1[i] - xyz0[i]));
      xyz1[i] = xyz0[i] + dimensions[i] * metersPerPixel[i];
      num_cells *= dimensions[i];
    }
    if (allocate_data) {
      data = new T[num_cells];
      reset(initValue);
    }
  }

  template<class F>
  VoxelMap<T> (const VoxelMap<F> * to_copy, T(*transformFunc)(F) = NULL) :
    msg(NULL)
  {
    memcpy(xyz0, to_copy->xyz0, 3 * sizeof(double));
    memcpy(xyz1, to_copy->xyz1, 3 * sizeof(double));
    memcpy(metersPerPixel, to_copy->metersPerPixel, 3 * sizeof(double));
    memcpy(dimensions, to_copy->dimensions, 3 * sizeof(int));

    num_cells = 1;
    for (int i = 0; i < 3; i++)
      num_cells *= dimensions[i];
    data = new T[num_cells];
    int ixyz[3];
    for (ixyz[2] = 0; ixyz[2] < dimensions[2]; ixyz[2]++) {
      for (ixyz[1] = 0; ixyz[1] < dimensions[1]; ixyz[1]++) {
        for (ixyz[0] = 0; ixyz[0] < dimensions[0]; ixyz[0]++) {
          if (transformFunc != NULL)
            writeValue(ixyz, transformFunc(to_copy->readValue(ixyz)));
          else
            writeValue(ixyz, to_copy->readValue(ixyz));
        }
      }
    }
  }

  VoxelMap<T> (const occ_map_voxel_map_t * _msg) :
    msg(NULL)
  {
    set_from_voxel_map_t(_msg);
  }
  /*
   * Constructor from a file (created with "saveToFile")
   */
  VoxelMap<T> (const char * name) :
    msg(NULL), data(NULL)
  {
    std::ifstream ifs(name, std::ios::binary);
    int sz;
    ifs >> sz;
    char * data = (char *) malloc(sz * sizeof(char));
    ifs.read(data, sz * sizeof(char));
    ifs.close();
    occ_map_voxel_map_t tmpmsg;
    occ_map_voxel_map_t_decode(data, 0, sz, &tmpmsg);
    set_from_voxel_map_t(&tmpmsg);
    occ_map_voxel_map_t_decode_cleanup(&tmpmsg);
    free(data);
  }

  virtual ~VoxelMap<T> ()
  {
    delete[] data;
  }

  //get linear index into storage arrays
  inline int getInd(const int ixyz[3]) const
  {
    return ixyz[2] * (dimensions[0] * dimensions[1]) + ixyz[1] * dimensions[0] + ixyz[0];
  }
  inline void indToLoc(int ind, int ixyz[3]) const
  {
    ixyz[2] = ind / (dimensions[0] * dimensions[1]);
    ind -= ixyz[2] * (dimensions[0] * dimensions[1]);
    ixyz[1] = ind / (dimensions[0]);
    ind -= ixyz[1] * dimensions[0];
    ixyz[0] = ind;
  }

  inline bool worldToTable(const double xyz[3], int ixyz[3]) const
  {
    bool clamped = false;
    for (int i = 0; i < 3; i++) {
      ixyz[i] = round((xyz[i] - xyz0[i]) / metersPerPixel[i]);
      if (ixyz[i] <= 0 || ixyz[i] >= dimensions[i] - 1) {
        ixyz[i] = clamp_value(ixyz[i], 0, dimensions[i] - 1);
        clamped = true;
      }
    }
    return clamped;
  }

  inline void tableToWorld(const int ixyz[3], double * xyz) const
  {
    for (int i = 0; i < 3; i++)
      xyz[i] = ((double) ixyz[i]) * metersPerPixel[i] + xyz0[i];
  }

  inline bool isInMap(const int ixyz[3]) const
  {
    for (int i = 0; i < 3; i++)
      if (ixyz[i] < 0 || ixyz[i] >= dimensions[i] - 1)
        return false;
    return true;
  }
  inline bool isInMap(const double xyz[3]) const
  {
    for (int i = 0; i < 3; i++)
      if (xyz[i] <= xyz0[i] || xyz[i] >= xyz1[i])
        return false;
    return true;
  }

  void reset(T resetVal = 0)
  {
    if (data != NULL)
      for (int i = 0; i < num_cells; i++)
        data[i] = resetVal;
  }

  inline T readValue(const int ixyz[3]) const
  {
    int ind = getInd(ixyz);
    return data[ind];
  }
  inline float readValue(const double xyz[3]) const
  {
    int ixyz[3];
    worldToTable(xyz, ixyz);
    return readValue(ixyz);
  }
  inline void writeValue(const int ixyz[3], T value)
  {
    int ind = getInd(ixyz);
    data[ind] = value;
  }
  inline void writeValue(const double xyz[3], T value)
  {
    int ixyz[3];
    worldToTable(xyz, ixyz);
    writeValue(ixyz, value);
  }
  inline void updateValue(const int ixyz[3], T value, const T clamp_bounds[2] = NULL)
  {
    int ind = getInd(ixyz);
    data[ind] += value;
    if (clamp_bounds != NULL) {
      data[ind] = clamp_value(data[ind],clamp_bounds[0],clamp_bounds[1]);
    }

  }
  inline void updateValue(const double xyz[3], T value, const T clamp_bounds[2] = NULL)
  {
    int ixyz[3];
    worldToTable(xyz, ixyz);
    updateValue(ixyz, value, clamp_bounds);
  }

  void raytrace(const int origin[3], const int endpoint[3], T miss_inc, T hit_inc,const  T clamp_bounds[2] = NULL)
  {
    //3D Bresenham implimentation copied from:
    //http://www.cit.griffith.edu.au/~anthony/info/graphics/bresenham.procs
    //

    int x1, y1, z1, x2, y2, z2;
    x1 = origin[0];
    y1 = origin[1];
    z1 = origin[2];
    x2 = endpoint[0];
    y2 = endpoint[1];
    z2 = endpoint[2];
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
    int voxel[3];

    voxel[0] = x1;
    voxel[1] = y1;
    voxel[2] = z1;
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;
    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;

    if ((l >= m) && (l >= n)) {
      err_1 = dy2 - l;
      err_2 = dz2 - l;
      for (i = 0; i < l; i++) {
        updateValue(voxel, miss_inc, clamp_bounds);
        if (err_1 > 0) {
          voxel[1] += y_inc;
          err_1 -= dx2;
        }
        if (err_2 > 0) {
          voxel[2] += z_inc;
          err_2 -= dx2;
        }
        err_1 += dy2;
        err_2 += dz2;
        voxel[0] += x_inc;
      }
    }
    else if ((m >= l) && (m >= n)) {
      err_1 = dx2 - m;
      err_2 = dz2 - m;
      for (i = 0; i < m; i++) {
        updateValue(voxel, miss_inc, clamp_bounds);
        if (err_1 > 0) {
          voxel[0] += x_inc;
          err_1 -= dy2;
        }
        if (err_2 > 0) {
          voxel[2] += z_inc;
          err_2 -= dy2;
        }
        err_1 += dx2;
        err_2 += dz2;
        voxel[1] += y_inc;
      }
    }
    else {
      err_1 = dy2 - n;
      err_2 = dx2 - n;
      for (i = 0; i < n; i++) {
        updateValue(voxel, miss_inc, clamp_bounds);

        if (err_1 > 0) {
          voxel[1] += y_inc;
          err_1 -= dz2;
        }
        if (err_2 > 0) {
          voxel[0] += x_inc;
          err_2 -= dz2;
        }
        err_1 += dy2;
        err_2 += dx2;
        voxel[2] += z_inc;
      }
    }
    updateValue(voxel, hit_inc, clamp_bounds);
  }

  void raytrace(const double origin[3],const  double endpoint[3], T miss_inc, T hit_inc,const  T clamp_bounds[2] = NULL)
  {
    int iorigin[3];
    int iendpoint[3];
    worldToTable(origin, iorigin);
    worldToTable(endpoint, iendpoint);
    raytrace(iorigin, iendpoint, miss_inc, hit_inc, clamp_bounds);
  }

  const occ_map_voxel_map_t * get_voxel_map_t(int64_t utime)
  {
    if (msg == NULL)
      msg = (occ_map_voxel_map_t *) calloc(1, sizeof(occ_map_voxel_map_t));

    memcpy(msg->xyz0, xyz0, 3 * sizeof(double));
    memcpy(msg->xyz1, xyz1, 3 * sizeof(double));
    memcpy(msg->mpp, metersPerPixel, 3 * sizeof(double));
    memcpy(msg->dimensions, dimensions, 3 * sizeof(int));

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

  void set_from_voxel_map_t(const occ_map_voxel_map_t * _msg)
  {
    memcpy(xyz0, _msg->xyz0, 3 * sizeof(double));
    memcpy(xyz1, _msg->xyz1, 3 * sizeof(double));
    memcpy(metersPerPixel, _msg->mpp, 3 * sizeof(double));
    memcpy(dimensions, _msg->dimensions, 3 * sizeof(int));

    num_cells = 1;
    for (int i = 0; i < 3; i++)
      num_cells *= dimensions[i];
    uLong uncompressed_size = num_cells * sizeof(T);
    data = (T *) malloc(uncompressed_size); //TODO: does this cause problems with the delete[] in the destructor??

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
      assert(_msg->datasize == uncompressed_size);
      memcpy(data, _msg->mapData, uncompressed_size);
    }
  }

  void saveToFile(const char * name)
  {
    const occ_map_voxel_map_t * msg = get_voxel_map_t(0);
    int sz = occ_map_voxel_map_t_encoded_size(msg);
    char * buf = (char *) malloc(sz * sizeof(char));
    occ_map_voxel_map_t_encode(buf, 0, sz, msg);
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

typedef VoxelMap<float> FloatVoxelMap;

}

#endif /*VOXELMAP_H_*/
