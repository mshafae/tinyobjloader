//
// Simple .obj viewer(vertex only)
//
#include <GL/glew.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <limits>
#include <map>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>

#define GLM_FORCE_SWIZZLE
#include <glm/glm.hpp>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <GLFW/glfw3.h>

#define TINYOBJLOADER_IMPLEMENTATION
// TINYOBJLOADER_USE_MAPBOX_EARCUT: Enable better triangulation. Requires C++11
// #define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include "../../tiny_obj_loader.h"
#include "trackball.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#endif

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#ifdef _WIN32
#ifdef __cplusplus
extern "C" {
#endif
#include <windows.h>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

#include <mmsystem.h>
#ifdef __cplusplus
}
#endif
#pragma comment(lib, "winmm.lib")
#else
#if defined(__unix__) || defined(__APPLE__)
#include <sys/time.h>
#else
#include <ctime>
#endif
#endif

class timerutil {
 public:
#ifdef _WIN32
  typedef DWORD time_t;

  timerutil() { ::timeBeginPeriod(1); }
  ~timerutil() { ::timeEndPeriod(1); }

  void start() { t_[0] = ::timeGetTime(); }
  void end() { t_[1] = ::timeGetTime(); }

  time_t sec() { return (time_t)((t_[1] - t_[0]) / 1000); }
  time_t msec() { return (time_t)((t_[1] - t_[0])); }
  time_t usec() { return (time_t)((t_[1] - t_[0]) * 1000); }
  time_t current() { return ::timeGetTime(); }

#else
#if defined(__unix__) || defined(__APPLE__)
  typedef unsigned long int time_t;

  void start() { gettimeofday(tv + 0, &tz); }
  void end() { gettimeofday(tv + 1, &tz); }

  time_t sec() { return (time_t)(tv[1].tv_sec - tv[0].tv_sec); }
  time_t msec() {
    return this->sec() * 1000 +
           (time_t)((tv[1].tv_usec - tv[0].tv_usec) / 1000);
  }
  time_t usec() {
    return this->sec() * 1000000 + (time_t)(tv[1].tv_usec - tv[0].tv_usec);
  }
  time_t current() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (time_t)(t.tv_sec * 1000 + t.tv_usec);
  }

#else  // C timer
  // using namespace std;
  typedef clock_t time_t;

  void start() { t_[0] = clock(); }
  void end() { t_[1] = clock(); }

  time_t sec() { return (time_t)((t_[1] - t_[0]) / CLOCKS_PER_SEC); }
  time_t msec() { return (time_t)((t_[1] - t_[0]) * 1000 / CLOCKS_PER_SEC); }
  time_t usec() { return (time_t)((t_[1] - t_[0]) * 1000000 / CLOCKS_PER_SEC); }
  time_t current() { return (time_t)clock(); }

#endif
#endif

 private:
#ifdef _WIN32
  DWORD t_[2];
#else
#if defined(__unix__) || defined(__APPLE__)
  struct timeval tv[2];
  struct timezone tz;
#else
  time_t t_[2];
#endif
#endif
};

typedef struct {
  GLuint vb_id;  // vertex buffer id
  int numTriangles;
  size_t material_id;
} DrawObject;

std::vector<DrawObject> gDrawObjects;

int width = 768;
int height = 768;

double prevMouseX, prevMouseY;
bool mouseLeftPressed;
bool mouseMiddlePressed;
bool mouseRightPressed;
float curr_quat[4];
float prev_quat[4];
float eye[3], lookat[3], up[3];
bool g_show_wire = true;
bool g_cull_face = false;

GLFWwindow* window;

static std::string GetBaseDir(const std::string& filepath) {
  if (filepath.find_last_of("/\\") != std::string::npos)
    return filepath.substr(0, filepath.find_last_of("/\\"));
  return "";
}

static bool FileExists(const std::string& abs_filename) {
  bool ret;
  FILE* fp = fopen(abs_filename.c_str(), "rb");
  if (fp) {
    ret = true;
    fclose(fp);
  } else {
    ret = false;
  }

  return ret;
}

static void CheckErrors(std::string desc) {
  GLenum e = glGetError();
  if (e != GL_NO_ERROR) {
    fprintf(stderr, "OpenGL error in \"%s\": %d (%d)\n", desc.c_str(), e, e);
    exit(20);
  }
}

static void CalcNormal(float N[3], float v0[3], float v1[3], float v2[3]) {
  float v10[3];
  v10[0] = v1[0] - v0[0];
  v10[1] = v1[1] - v0[1];
  v10[2] = v1[2] - v0[2];

  float v20[3];
  v20[0] = v2[0] - v0[0];
  v20[1] = v2[1] - v0[1];
  v20[2] = v2[2] - v0[2];

  N[0] = v10[1] * v20[2] - v10[2] * v20[1];
  N[1] = v10[2] * v20[0] - v10[0] * v20[2];
  N[2] = v10[0] * v20[1] - v10[1] * v20[0];

  float len2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
  if (len2 > 0.0f) {
    float len = sqrtf(len2);

    N[0] /= len;
    N[1] /= len;
    N[2] /= len;
  }
}

namespace  // Local utility functions
{
struct vec3 {
  float v[3];
  vec3() {
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 0.0f;
  }
};

void normalizeVector(vec3& v) {
  float len2 = v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2];
  if (len2 > 0.0f) {
    float len = sqrtf(len2);

    v.v[0] /= len;
    v.v[1] /= len;
    v.v[2] /= len;
  }
}

/*
  There are 2 approaches here to automatically generating vertex normals. The
  old approach (computeSmoothingNormals) doesn't handle multiple smoothing
  groups properly, as it effectively merges all smoothing groups present in the
  OBJ file into a single group. However, it can be useful when the OBJ file
  contains vertex normals which you want to use, but is missing some, as it
  will attempt to fill in the missing normals without generating new shapes.

  The new approach (computeSmoothingShapes, computeAllSmoothingNormals) handles
  multiple smoothing groups but is a bit more complicated, as handling this
  correctly requires potentially generating new vertices (and hence shapes).
  In general, the new approach is most useful if your OBJ file is missing
  vertex normals entirely, and instead relies on smoothing groups to correctly
  generate them as a pre-process. That said, it can be used to reliably
  generate vertex normals in the general case. If you want to always generate
  normals in this way, simply force set regen_all_normals to true below. By
  default, it's only true when there are no vertex normals present. One other
  thing to keep in mind is that the statistics printed apply to the model
  *prior* to shape regeneration, so you'd need to print them again if you want
  to see the new statistics.

  TODO(syoyo): import computeSmoothingShapes and computeAllSmoothingNormals to
  tinyobjloader as utility functions.
*/

// Check if `mesh_t` contains smoothing group id.
bool hasSmoothingGroup(const tinyobj::shape_t& shape) {
  for (size_t i = 0; i < shape.mesh.smoothing_group_ids.size(); i++) {
    if (shape.mesh.smoothing_group_ids[i] > 0) {
      return true;
    }
  }
  return false;
}

void computeSmoothingNormals(const tinyobj::attrib_t& attrib,
                             const tinyobj::shape_t& shape,
                             std::map<int, vec3>& smoothVertexNormals) {
  smoothVertexNormals.clear();
  std::map<int, vec3>::iterator iter;

  for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
    // Get the three indexes of the face (all faces are triangular)
    tinyobj::index_t idx0 = shape.mesh.indices[3 * f + 0];
    tinyobj::index_t idx1 = shape.mesh.indices[3 * f + 1];
    tinyobj::index_t idx2 = shape.mesh.indices[3 * f + 2];

    // Get the three vertex indexes and coordinates
    int vi[3];      // indexes
    float v[3][3];  // coordinates

    for (int k = 0; k < 3; k++) {
      vi[0] = idx0.vertex_index;
      vi[1] = idx1.vertex_index;
      vi[2] = idx2.vertex_index;
      assert(vi[0] >= 0);
      assert(vi[1] >= 0);
      assert(vi[2] >= 0);

      v[0][k] = attrib.vertices[3 * vi[0] + k];
      v[1][k] = attrib.vertices[3 * vi[1] + k];
      v[2][k] = attrib.vertices[3 * vi[2] + k];
    }

    // Compute the normal of the face
    float normal[3];
    CalcNormal(normal, v[0], v[1], v[2]);

    // Add the normal to the three vertexes
    for (size_t i = 0; i < 3; ++i) {
      iter = smoothVertexNormals.find(vi[i]);
      if (iter != smoothVertexNormals.end()) {
        // add
        iter->second.v[0] += normal[0];
        iter->second.v[1] += normal[1];
        iter->second.v[2] += normal[2];
      } else {
        smoothVertexNormals[vi[i]].v[0] = normal[0];
        smoothVertexNormals[vi[i]].v[1] = normal[1];
        smoothVertexNormals[vi[i]].v[2] = normal[2];
      }
    }

  }  // f

  // Normalize the normals, that is, make them unit vectors
  for (iter = smoothVertexNormals.begin(); iter != smoothVertexNormals.end();
       iter++) {
    normalizeVector(iter->second);
  }

}  // computeSmoothingNormals

static void computeAllSmoothingNormals(tinyobj::attrib_t& attrib,
                                       std::vector<tinyobj::shape_t>& shapes) {
  vec3 p[3];
  for (size_t s = 0, slen = shapes.size(); s < slen; ++s) {
    const tinyobj::shape_t& shape(shapes[s]);
    size_t facecount = shape.mesh.num_face_vertices.size();
    assert(shape.mesh.smoothing_group_ids.size());

    for (size_t f = 0, flen = facecount; f < flen; ++f) {
      for (unsigned int v = 0; v < 3; ++v) {
        tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
        assert(idx.vertex_index != -1);
        p[v].v[0] = attrib.vertices[3 * idx.vertex_index];
        p[v].v[1] = attrib.vertices[3 * idx.vertex_index + 1];
        p[v].v[2] = attrib.vertices[3 * idx.vertex_index + 2];
      }

      // cross(p[1] - p[0], p[2] - p[0])
      float nx = (p[1].v[1] - p[0].v[1]) * (p[2].v[2] - p[0].v[2]) -
                 (p[1].v[2] - p[0].v[2]) * (p[2].v[1] - p[0].v[1]);
      float ny = (p[1].v[2] - p[0].v[2]) * (p[2].v[0] - p[0].v[0]) -
                 (p[1].v[0] - p[0].v[0]) * (p[2].v[2] - p[0].v[2]);
      float nz = (p[1].v[0] - p[0].v[0]) * (p[2].v[1] - p[0].v[1]) -
                 (p[1].v[1] - p[0].v[1]) * (p[2].v[0] - p[0].v[0]);

      // Don't normalize here.
      for (unsigned int v = 0; v < 3; ++v) {
        tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
        attrib.normals[3 * idx.normal_index] += nx;
        attrib.normals[3 * idx.normal_index + 1] += ny;
        attrib.normals[3 * idx.normal_index + 2] += nz;
      }
    }
  }

  assert(attrib.normals.size() % 3 == 0);
  for (size_t i = 0, nlen = attrib.normals.size() / 3; i < nlen; ++i) {
    tinyobj::real_t& nx = attrib.normals[3 * i];
    tinyobj::real_t& ny = attrib.normals[3 * i + 1];
    tinyobj::real_t& nz = attrib.normals[3 * i + 2];
    tinyobj::real_t len = sqrtf(nx * nx + ny * ny + nz * nz);
    tinyobj::real_t scale = len == 0 ? 0 : 1 / len;
    nx *= scale;
    ny *= scale;
    nz *= scale;
  }
}

static void computeSmoothingShape(
    const tinyobj::attrib_t& inattrib, const tinyobj::shape_t& inshape,
    std::vector<std::pair<unsigned int, unsigned int>>& sortedids,
    unsigned int idbegin, unsigned int idend,
    std::vector<tinyobj::shape_t>& outshapes, tinyobj::attrib_t& outattrib) {
  unsigned int sgroupid = sortedids[idbegin].first;
  bool hasmaterials = inshape.mesh.material_ids.size();
  // Make a new shape from the set of faces in the range [idbegin, idend).
  outshapes.emplace_back();
  tinyobj::shape_t& outshape = outshapes.back();
  outshape.name = inshape.name;
  // Skip lines and points.

  std::unordered_map<unsigned int, unsigned int> remap;
  for (unsigned int id = idbegin; id < idend; ++id) {
    unsigned int face = sortedids[id].second;

    outshape.mesh.num_face_vertices.push_back(3);  // always triangles
    if (hasmaterials)
      outshape.mesh.material_ids.push_back(inshape.mesh.material_ids[face]);
    outshape.mesh.smoothing_group_ids.push_back(sgroupid);
    // Skip tags.

    for (unsigned int v = 0; v < 3; ++v) {
      tinyobj::index_t inidx = inshape.mesh.indices[3 * face + v], outidx;
      assert(inidx.vertex_index != -1);
      auto iter = remap.find(inidx.vertex_index);
      // Smooth group 0 disables smoothing so no shared vertices in that case.
      if (sgroupid && iter != remap.end()) {
        outidx.vertex_index = (*iter).second;
        outidx.normal_index = outidx.vertex_index;
        outidx.texcoord_index =
            (inidx.texcoord_index == -1) ? -1 : outidx.vertex_index;
      } else {
        assert(outattrib.vertices.size() % 3 == 0);
        unsigned int offset =
            static_cast<unsigned int>(outattrib.vertices.size() / 3);
        outidx.vertex_index = outidx.normal_index = offset;
        outidx.texcoord_index = (inidx.texcoord_index == -1) ? -1 : offset;
        outattrib.vertices.push_back(inattrib.vertices[3 * inidx.vertex_index]);
        outattrib.vertices.push_back(
            inattrib.vertices[3 * inidx.vertex_index + 1]);
        outattrib.vertices.push_back(
            inattrib.vertices[3 * inidx.vertex_index + 2]);
        outattrib.normals.push_back(0.0f);
        outattrib.normals.push_back(0.0f);
        outattrib.normals.push_back(0.0f);
        if (inidx.texcoord_index != -1) {
          outattrib.texcoords.push_back(
              inattrib.texcoords[2 * inidx.texcoord_index]);
          outattrib.texcoords.push_back(
              inattrib.texcoords[2 * inidx.texcoord_index + 1]);
        }
        remap[inidx.vertex_index] = offset;
      }
      outshape.mesh.indices.push_back(outidx);
    }
  }
}

static void computeSmoothingShapes(const tinyobj::attrib_t& inattrib,
                                   const std::vector<tinyobj::shape_t>& inshapes,
                                   std::vector<tinyobj::shape_t>& outshapes,
                                   tinyobj::attrib_t& outattrib) {
  for (size_t s = 0, slen = inshapes.size(); s < slen; ++s) {
    const tinyobj::shape_t& inshape = inshapes[s];

    unsigned int numfaces =
        static_cast<unsigned int>(inshape.mesh.smoothing_group_ids.size());
    assert(numfaces);
    std::vector<std::pair<unsigned int, unsigned int>> sortedids(numfaces);
    for (unsigned int i = 0; i < numfaces; ++i)
      sortedids[i] = std::make_pair(inshape.mesh.smoothing_group_ids[i], i);
    sort(sortedids.begin(), sortedids.end());

    unsigned int activeid = sortedids[0].first;
    unsigned int id = activeid, idbegin = 0, idend = 0;
    // Faces are now bundled by smoothing group id, create shapes from these.
    while (idbegin < numfaces) {
      while (activeid == id && ++idend < numfaces) id = sortedids[idend].first;
      computeSmoothingShape(inattrib, inshape, sortedids, idbegin, idend,
                            outshapes, outattrib);
      activeid = id;
      idbegin = idend;
    }
  }
}

}  // namespace

void LoadDiffuseTexture(const tinyobj::material_t* mp,
                        const std::string& base_dir,
                        std::map<std::string, GLuint>& textures) {
  if (mp->diffuse_texname.length() > 0) {
    // Only load the texture if it is not already loaded
    if (textures.find(mp->diffuse_texname) == textures.end()) {
      GLuint texture_id;
      int w, h;
      int comp;

      std::string texture_filename = mp->diffuse_texname;
      std::cerr << "Working on texture filename: " << texture_filename << "\n";
      if (!FileExists(texture_filename)) {
        // Append base dir.
        texture_filename = base_dir + mp->diffuse_texname;
        if (!FileExists(texture_filename)) {
          std::cerr << "Unable to find file: " << mp->diffuse_texname
                    << std::endl;
          exit(1);
        }
      }

      unsigned char* image =
          stbi_load(texture_filename.c_str(), &w, &h, &comp, STBI_default);
      if (!image) {
        std::cerr << "Unable to load texture: " << texture_filename
                  << std::endl;
        exit(1);
      }
      std::cout << "Loaded texture: " << texture_filename << ", w = " << w
                << ", h = " << h << ", comp = " << comp << std::endl;

      glGenTextures(1, &texture_id);
      glBindTexture(GL_TEXTURE_2D, texture_id);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      if (comp == 3) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, image);
      } else if (comp == 4) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image);
      } else {
        assert(0);  // TODO
      }
      glBindTexture(GL_TEXTURE_2D, 0);
      stbi_image_free(image);
      textures.insert(std::make_pair(mp->diffuse_texname, texture_id));
    }
  }
}

std::string IlluminationModelString(int illum_number) {
  // See https://www.fileformat.info/format/material/
  switch (illum_number) {
    case 0:
      return "Color on and Ambient off";
      break;
    case 1:
      return "Color on and Ambient on";
      break;
    case 2:
      return "Highlight on";
      break;
    case 3:
      return "Reflection on and Ray trace on";
      break;
    case 4:
      return "Transparency: Glass on\nReflection: Ray trace on";
      break;
    case 5:
      return "Reflection: Fresnel on and Ray trace on";
      break;
    case 6:
      return "Transparency: Refraction on\nReflection: Fresnel off and Ray "
             "trace on";
      break;
    case 7:
      return "Transparency: Refraction on\nReflection: Fresnel on and Ray "
             "trace on";
      break;
    case 8:
      return "Reflection on and Ray trace off";
      break;
    case 9:
      return "Transparency: Glass on\nReflection: Ray trace off";
      break;
    case 10:
      return "Casts shadows onto invisible surfaces";
      break;
    default:
      throw std::runtime_error("Unknown illumination model number");
      break;
  }
}

void PrintOBJMaterial(std::ostream& out, const tinyobj::material_t& mp) {
  out << "Material \"" << mp.name << "\"\n";
  out << "ambient: " << mp.ambient[0] << ", " << mp.ambient[1] << ", "
      << mp.ambient[2] << "\n";
  out << "diffuse: " << mp.diffuse[0] << ", " << mp.diffuse[1] << ", "
      << mp.diffuse[2] << "\n";
  out << "specular: " << mp.specular[0] << ", " << mp.specular[1] << ", "
      << mp.specular[2] << "\n";
  out << "transmittance: " << mp.transmittance[0] << ", " << mp.transmittance[1]
      << ", " << mp.transmittance[2] << "\n";
  out << "emission: " << mp.emission[0] << ", " << mp.emission[1] << ", "
      << mp.emission[2] << "\n";
  out << "shininess: " << mp.shininess << "\n";
  out << "index of refraction: " << mp.ior << "\n";
  out << "dissolve: " << mp.dissolve << "\n";
  out << "illumination model: (" << mp.illum << ") "
      << IlluminationModelString(mp.illum) << "\n";
  out << "ambient_texname: " << mp.ambient_texname << "\n";
  out << "diffuse_texname: " << mp.diffuse_texname << "\n";
  out << "specular_texname: " << mp.specular_texname << "\n";
  out << "specular_highlight_texname: " << mp.specular_highlight_texname
      << "\n";
  out << "bump_texname: " << mp.bump_texname << "\n";
  out << "displacement_texname: " << mp.displacement_texname << "\n";
  out << "alpha_texname: " << mp.alpha_texname << "\n";
  out << "reflection_texname: " << mp.reflection_texname << "\n";
}

void WriteBufferTest(const std::string& filename, const std::vector<tinyobj::real_t>& buffer) {
  std::ofstream fh{filename};
  if (!fh) {
    std::cerr << "Failed to open " << filename << "\n";
    exit(1);
  }
  fh << std::fixed << std::setprecision(3);
  int i{0};
  for (const auto& val : buffer) {
    fh << i++ << ": " << val << "\n";
  }
  fh.close();
}

bool LoadObjAndConvert(
  glm::vec3& bmin, glm::vec3& bmax,
  std::vector<DrawObject>* drawObjects,
                       std::vector<tinyobj::material_t>& _materials,
                       std::map<std::string, GLuint>& textures,
                       const char* inputfile) {
  timerutil tm;

  tm.start();

  std::string base_dir = GetBaseDir(inputfile);
  if (base_dir.empty()) {
    base_dir = ".";
  }
#ifdef _WIN32
  base_dir += "\\";
#else
  base_dir += "/";
#endif

  // defaults to triangulate(true), triangulation_method("simple"),
  // vertex_color(true)
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = base_dir;  // Path to material files

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error();
    }
    return false;
  }
  assert(reader.Valid());
  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  tm.end();

  printf("Parsing time: %d [ms]\n", (int)tm.msec());

  printf("# of vertices  = %d\n", (int)(attrib.vertices.size()) / 3);
  printf("# of normals   = %d\n", (int)(attrib.normals.size()) / 3);
  printf("# of texcoords = %d\n", (int)(attrib.texcoords.size()) / 2);
  printf("# of materials = %d\n", (int)materials.size());
  printf("# of shapes    = %d\n", (int)shapes.size());

  // Load diffuse textures
  {
    for (size_t m = 0; m < materials.size(); m++) {
      const tinyobj::material_t* mp = &materials[m];
      // Given the material property, open the named image file (given the
      // basedir) and store the texture_id as the value to the texture
      // filename's key. mp->diffuse_texname: texture_id
      if (mp->diffuse_texname.size() > 0) {
        LoadDiffuseTexture(mp, base_dir, textures);
      }
    }
  }

  // Bounding box init
  // bmin[0] = bmin[1] = bmin[2] = std::numeric_limits<float>::max();
  // bmax[0] = bmax[1] = bmax[2] = -std::numeric_limits<float>::max();

  bmin = glm::vec3{std::numeric_limits<tinyobj::real_t>::max()};
  bmax = glm::vec3{-std::numeric_limits<tinyobj::real_t>::max()};

  bool regen_all_normals = attrib.normals.size() == 0;

  tinyobj::attrib_t outattrib;
  std::vector<tinyobj::shape_t> outshapes;

  if (regen_all_normals) {
    std::cout << "Regenerate all normals!\n";
    computeSmoothingShapes(attrib, shapes, outshapes, outattrib);
    computeAllSmoothingNormals(outattrib, outshapes);
  } else {
    outshapes = shapes;
    outattrib = attrib;
  }

  printf("# of vertices  = %d\n", (int)(outattrib.vertices.size()) / 3);
  printf("# of normals   = %d\n", (int)(outattrib.normals.size()) / 3);
  printf("# of texcoords = %d\n", (int)(outattrib.texcoords.size()) / 2);
  printf("# of materials = %d\n", (int)materials.size());
  printf("# of shapes    = %d\n", (int)outshapes.size());

  int buffer_name_count{0};
  // Loop over shapes
  for (size_t s = 0; s < outshapes.size(); s++) {
    DrawObject draw_object;
    // pos(3float), normal(3float), color(3float)
    std::vector<tinyobj::real_t> buffer;

    // Check for smoothing group and compute smoothing normals
    std::map<int, vec3> smoothVertexNormals;
    if (!regen_all_normals && (hasSmoothingGroup(outshapes[s]) > 0)) {
      std::cout << "Compute smoothingNormal for shape [" << s << "]\n";
      computeSmoothingNormals(outattrib, outshapes[s], smoothVertexNormals);
    }


    // Loop over faces(polygon) 3 at a time
    size_t index_offset = 0;
    for (size_t f = 0; f < outshapes.at(s).mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(outshapes.at(s).mesh.num_face_vertices.at(f));
      // Triangulation is turned on in tinyobj. All faces should have 3
      // vertices.
      assert(fv == 3);

      // Loop over vertices in the face.
      glm::vec3 vertices[3];
      glm::vec3 normals[3];
      glm::vec2 texcoords[3];

      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = outshapes.at(s).mesh.indices.at(index_offset + v);
        tinyobj::real_t vx = outattrib.vertices.at(3 * size_t(idx.vertex_index) + 0);
        tinyobj::real_t vy = outattrib.vertices.at(3 * size_t(idx.vertex_index) + 1);
        tinyobj::real_t vz = outattrib.vertices.at(3 * size_t(idx.vertex_index) + 2);
        assert(vx != NAN && vy != NAN && vz != NAN);
        vertices[v] = glm::vec3{vx, vy, vz};
        bmin.x = glm::min(bmin.x, vx);
        bmin.y = glm::min(bmin.y, vy);
        bmin.z = glm::min(bmin.z, vz);

        bmax.x = glm::max(bmax.x, vx);
        bmax.y = glm::max(bmax.y, vy);
        bmax.z = glm::max(bmax.z, vz);


        // Check if `normal_index` is zero or positive. negative = no normal
        // data
        if (idx.normal_index >= 0) {
          tinyobj::real_t nx = outattrib.normals.at(3 * size_t(idx.normal_index) + 0);
          tinyobj::real_t ny = outattrib.normals.at(3 * size_t(idx.normal_index) + 1);
          tinyobj::real_t nz = outattrib.normals.at(3 * size_t(idx.normal_index) + 2);
          assert(nx != NAN && ny != NAN && nz != NAN);
          normals[v] = glm::vec3{nx, ny, nz};

        }

        // Check if `texcoord_index` is zero or positive. negative = no texcoord
        // data
        if (idx.texcoord_index >= 0) {
          // tinyobj::real_t tx =
          //     outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 0);
          // tinyobj::real_t ty =
          //     outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 1);
          // Flip Y coordinate
          tinyobj::real_t tx = outattrib.texcoords.at(2 * idx.texcoord_index);
          tinyobj::real_t ty = 1.0f - outattrib.texcoords.at(2 * idx.texcoord_index + 1);

          texcoords[v] = glm::vec2{tx, ty};
        }
        
        // Optional: vertex colors
        // tinyobj::real_t red   = outattrib.colors[3*size_t(idx.vertex_index)+0];
        // tinyobj::real_t green = outattrib.colors[3*size_t(idx.vertex_index)+1];
        // tinyobj::real_t blue  = outattrib.colors[3*size_t(idx.vertex_index)+2];

      } // end loop over vertices in the face
      // index_offset counts up by 3 but fv could be something other than 3
      // there is an assert to make sure it increases by 3, triangulation
      // is on by default
      index_offset += fv;

      // per-face material
      // shapes[s].mesh.material_ids[f];
      int current_material_id{outshapes.at(s).mesh.material_ids.at(f)};
      if ((current_material_id < 0) ||
          (current_material_id >= static_cast<int>(materials.size()))) {
        std::cerr << "Current shape " << s << " missing a material.\n";
        return false;
      }

      glm::vec3 diffuse{
        materials.at(current_material_id).diffuse[0],
        materials.at(current_material_id).diffuse[1],
        materials.at(current_material_id).diffuse[2],
      };

      float normal_factor = 0.2;
      float diffuse_factor = 1 - normal_factor;
      glm::vec3 colors[3];

      for (int k = 0; k < 3; k++) {
        const glm::vec3 color{
                    normals[k].x * normal_factor + diffuse.r * diffuse_factor,
                    normals[k].y * normal_factor + diffuse.g * diffuse_factor,
                    normals[k].z * normal_factor + diffuse.b * diffuse_factor };
        const glm::vec3 normalized_color = glm::normalize(color);
        const glm::vec3 v_color{
          normalized_color.r * 0.5 + 0.5,
          normalized_color.g * 0.5 + 0.5,
          normalized_color.b * 0.5 + 0.5,
        };
        colors[k] = v_color;
      }



      // glm::vec3 colors[3] = {
      //   glm::vec3{1.0, 0.0, 0.0},
      //   glm::vec3{1.0, 0.0, 0.0},
      //   glm::vec3{1.0, 0.0, 0.0},
      // };


      for (int k = 0; k < 3; k++) {
        buffer.push_back(vertices[k].x);
        buffer.push_back(vertices[k].y);
        buffer.push_back(vertices[k].z);

        buffer.push_back(normals[k].x);
        buffer.push_back(normals[k].y);
        buffer.push_back(normals[k].z);

        buffer.push_back(colors[k].r);
        buffer.push_back(colors[k].g);
        buffer.push_back(colors[k].b);

        buffer.push_back(texcoords[k].x);
        buffer.push_back(texcoords[k].y);

      }

    } // end Loop over faces(polygon) 3 at a time

    draw_object.vb_id = 0;
    draw_object.numTriangles = 0;
    // Use the material ID of the first face
    // Does not support per-face material
    draw_object.material_id = outshapes[s].mesh.material_ids[0];
    // printf("shape[%d] material_id %d\n", int(s),
           // int(draw_object.material_id));

    if (buffer.size() > 0) {
      glGenBuffers(1, &draw_object.vb_id);
      glBindBuffer(GL_ARRAY_BUFFER, draw_object.vb_id);
      glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(tinyobj::real_t),
                   buffer.data(), GL_STATIC_DRAW);
      draw_object.numTriangles = buffer.size() / (3 + 3 + 3 + 2) /
                                 3;  // 3:vtx, 3:normal, 3:col, 2:texcoord

      printf("shape[%d] # of triangles = %d\n", static_cast<int>(s),
             draw_object.numTriangles);
    }

    if (false) {
      std::ostringstream file_name_stream;
      file_name_stream << "buffer-view-" << buffer_name_count++ << ".txt";
      WriteBufferTest(file_name_stream.str(), buffer);
    }

    drawObjects->push_back(draw_object);
  } // end for every shape

  _materials = materials;

  printf("bmin = %f, %f, %f\n", bmin.x, bmin.y, bmin.z);
  printf("bmax = %f, %f, %f\n", bmax.x, bmax.y, bmax.z);
  return true;
}

static void reshapeFunc(GLFWwindow* window, int w, int h) {
  int fb_w, fb_h;
  // Get actual framebuffer size.
  glfwGetFramebufferSize(window, &fb_w, &fb_h);

  glViewport(0, 0, fb_w, fb_h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (float)w / (float)h, 0.01f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  width = w;
  height = h;
}

static void keyboardFunc(GLFWwindow* window, int key, int scancode, int action,
                         int mods) {
  (void)window;
  (void)scancode;
  (void)mods;
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    // Move camera
    float mv_x = 0, mv_y = 0, mv_z = 0;
    if (key == GLFW_KEY_K)
      mv_x += 1;
    else if (key == GLFW_KEY_J)
      mv_x += -1;
    else if (key == GLFW_KEY_L)
      mv_y += 1;
    else if (key == GLFW_KEY_H)
      mv_y += -1;
    else if (key == GLFW_KEY_P)
      mv_z += 1;
    else if (key == GLFW_KEY_N)
      mv_z += -1;
    // camera.move(mv_x * 0.05, mv_y * 0.05, mv_z * 0.05);
    // Close window
    if (key == GLFW_KEY_Q || key == GLFW_KEY_ESCAPE) {
      glfwSetWindowShouldClose(window, GL_TRUE);
    }

    if (key == GLFW_KEY_W) {
      // toggle wireframe
      g_show_wire = !g_show_wire;
    }

    if (key == GLFW_KEY_C) {
      // cull option
      g_cull_face = !g_cull_face;
    }

    // init_frame = true;
  }
}

static void clickFunc(GLFWwindow* window, int button, int action, int mods) {
  (void)window;
  (void)mods;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      mouseLeftPressed = true;
      trackball(prev_quat, 0.0, 0.0, 0.0, 0.0);
    } else if (action == GLFW_RELEASE) {
      mouseLeftPressed = false;
    }
  }
  if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    if (action == GLFW_PRESS) {
      mouseRightPressed = true;
    } else if (action == GLFW_RELEASE) {
      mouseRightPressed = false;
    }
  }
  if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    if (action == GLFW_PRESS) {
      mouseMiddlePressed = true;
    } else if (action == GLFW_RELEASE) {
      mouseMiddlePressed = false;
    }
  }
}

static void motionFunc(GLFWwindow* window, double mouse_x, double mouse_y) {
  (void)window;
  float rotScale = 1.0f;
  float transScale = 2.0f;

  if (mouseLeftPressed) {
    trackball(prev_quat, rotScale * (2.0f * prevMouseX - width) / (float)width,
              rotScale * (height - 2.0f * prevMouseY) / (float)height,
              rotScale * (2.0f * mouse_x - width) / (float)width,
              rotScale * (height - 2.0f * mouse_y) / (float)height);

    add_quats(prev_quat, curr_quat, curr_quat);
  } else if (mouseMiddlePressed) {
    eye[0] -= transScale * (mouse_x - prevMouseX) / (float)width;
    lookat[0] -= transScale * (mouse_x - prevMouseX) / (float)width;
    eye[1] += transScale * (mouse_y - prevMouseY) / (float)height;
    lookat[1] += transScale * (mouse_y - prevMouseY) / (float)height;
  } else if (mouseRightPressed) {
    eye[2] += transScale * (mouse_y - prevMouseY) / (float)height;
    lookat[2] += transScale * (mouse_y - prevMouseY) / (float)height;
  }

  // Update mouse point
  prevMouseX = mouse_x;
  prevMouseY = mouse_y;
}

static void Draw(const std::vector<DrawObject>& drawObjects,
                 const std::vector<tinyobj::material_t>& materials,
                 const std::map<std::string, GLuint>& textures) {
  glPolygonMode(GL_FRONT, GL_FILL);
  if (g_cull_face) {
    glPolygonMode(GL_BACK, GL_LINE);
  } else {
    glPolygonMode(GL_BACK, GL_FILL);
  }

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1.0);
  GLsizei stride = (3 + 3 + 3 + 2) * sizeof(float);
  for (size_t i = 0; i < drawObjects.size(); i++) {
    DrawObject o = drawObjects[i];
    if (o.vb_id < 1) {
      continue;
    }

    glBindBuffer(GL_ARRAY_BUFFER, o.vb_id);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glBindTexture(GL_TEXTURE_2D, 0);

    // MS: If there isn't a material, make one
    // if ((o.material_id < materials.size())) {
      assert(o.material_id < materials.size());
      std::string diffuse_texname = materials.at(o.material_id).diffuse_texname;
      if (textures.find(diffuse_texname) != textures.end()) {
        glBindTexture(GL_TEXTURE_2D, textures.at(diffuse_texname));
      }
    // }
    glVertexPointer(3, GL_FLOAT, stride, (const void*)0);
    glNormalPointer(GL_FLOAT, stride, (const void*)(sizeof(float) * 3));
    glColorPointer(3, GL_FLOAT, stride, (const void*)(sizeof(float) * 6));
    glTexCoordPointer(2, GL_FLOAT, stride, (const void*)(sizeof(float) * 9));

    glDrawArrays(GL_TRIANGLES, 0, 3 * o.numTriangles);
    CheckErrors("drawarrays");
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // draw wireframe
  if (g_show_wire) {
    glDisable(GL_POLYGON_OFFSET_FILL);
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);

    glColor3f(0.0f, 0.0f, 0.4f);
    for (size_t i = 0; i < drawObjects.size(); i++) {
      DrawObject o = drawObjects[i];
      if (o.vb_id < 1) {
        continue;
      }

      glBindBuffer(GL_ARRAY_BUFFER, o.vb_id);
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_NORMAL_ARRAY);
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
      glVertexPointer(3, GL_FLOAT, stride, (const void*)0);
      glNormalPointer(GL_FLOAT, stride, (const void*)(sizeof(float) * 3));
      glColorPointer(3, GL_FLOAT, stride, (const void*)(sizeof(float) * 6));
      glTexCoordPointer(2, GL_FLOAT, stride, (const void*)(sizeof(float) * 9));

      glDrawArrays(GL_TRIANGLES, 0, 3 * o.numTriangles);
      CheckErrors("drawarrays");
    }
  }
}

static void Init() {
  trackball(curr_quat, 0, 0, 0, 0);

  eye[0] = 0.0f;
  eye[1] = 0.0f;
  eye[2] = 3.0f;

  lookat[0] = 0.0f;
  lookat[1] = 0.0f;
  lookat[2] = 0.0f;

  up[0] = 0.0f;
  up[1] = 1.0f;
  up[2] = 0.0f;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Needs input.obj\n" << std::endl;
    return 0;
  }

  Init();

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW." << std::endl;
    return -1;
  }

  window = glfwCreateWindow(width, height, "Obj viewer", NULL, NULL);
  if (window == NULL) {
    std::cerr << "Failed to open GLFW window. " << std::endl;
    glfwTerminate();
    return 1;
  }

  std::cout << "W : Toggle wireframe\n";
  std::cout << "C : Toggle face culling\n";
  // std::cout << "K, J, H, L, P, N : Move camera\n";
  std::cout << "Q, Esc : quit\n";

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // Callback
  glfwSetWindowSizeCallback(window, reshapeFunc);
  glfwSetKeyCallback(window, keyboardFunc);
  glfwSetMouseButtonCallback(window, clickFunc);
  glfwSetCursorPosCallback(window, motionFunc);

  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW." << std::endl;
    return -1;
  }

  reshapeFunc(window, width, height);

  // float bmin[3] = {-71.892532, 0.000000, -47.928356};
  // float bmax[3] = {82.293999, 79.006561, 47.928356};
  glm::vec3 bmin;
  glm::vec3 bmax;
  std::vector<tinyobj::material_t> materials;
  std::map<std::string, GLuint> textures;
  if (false == LoadObjAndConvert( bmin, bmax, &gDrawObjects, materials, textures,
                                 argv[1])) {
    return -1;
  }

  std::cout << "Textures:\n";
  for (const auto& pair : textures) {
    std::cout << pair.first << ", " << pair.second << "\n";
  }

  // MS: compute bounding box

  float maxExtent = 0.5f * (bmax[0] - bmin[0]);
  if (maxExtent < 0.5f * (bmax[1] - bmin[1])) {
    maxExtent = 0.5f * (bmax[1] - bmin[1]);
  }
  if (maxExtent < 0.5f * (bmax[2] - bmin[2])) {
    maxExtent = 0.5f * (bmax[2] - bmin[2]);
  }

  while (glfwWindowShouldClose(window) == GL_FALSE) {
    glfwPollEvents();
    glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    // camera & rotate
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat mat[4][4];
    gluLookAt(eye[0], eye[1], eye[2], lookat[0], lookat[1], lookat[2], up[0],
              up[1], up[2]);
    build_rotmatrix(mat, curr_quat);
    glMultMatrixf(&mat[0][0]);

    // Fit to -1, 1
    glScalef(1.0f / maxExtent, 1.0f / maxExtent, 1.0f / maxExtent);

    // Centerize object.
    glTranslatef(-0.5 * (bmax[0] + bmin[0]), -0.5 * (bmax[1] + bmin[1]),
                 -0.5 * (bmax[2] + bmin[2]));

    Draw(gDrawObjects, materials, textures);

    glfwSwapBuffers(window);
  }

  glfwTerminate();
}
