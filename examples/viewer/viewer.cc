//
// Simple .obj viewer(vertex only)
//
// #include <GL/glew.h>
#include <glad/gl.h>

// #include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
// #include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define GLM_FORCE_SWIZZLE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
// #include <glm/gtx/quaternion.hpp>
#include <glm/ext/matrix_relational.hpp>
#include <glm/gtx/string_cast.hpp>
#include <spdlog/spdlog.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <GLFW/glfw3.h>

#define TINYOBJLOADER_IMPLEMENTATION
// TINYOBJLOADER_USE_MAPBOX_EARCUT: Enable better triangulation. Requires C++11
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
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

typedef struct {
  GLuint vb_id;  // vertex buffer id
  int numTriangles;
  size_t material_id;
} DrawObject;

std::vector<DrawObject> gDrawObjects;

int width = 768;
int height = 768;

// double prevMouseX, prevMouseY;
glm::dvec2 prev_mouse;
bool mouseLeftPressed;
bool mouseMiddlePressed;
bool mouseRightPressed;
float curr_quat[4];
float prev_quat[4];
glm::quat curr_quat_;
glm::quat prev_quat_;
glm::vec3 eye, lookat, up;
// float eye[3], lookat[3], up[3];
bool g_show_wire = true;
bool g_cull_face = false;

GLFWwindow* window;

static void CheckErrors(std::string desc) {
  GLenum e = glGetError();
  if (e != GL_NO_ERROR) {
    fprintf(stderr, "OpenGL error in \"%s\": %d (%d)\n", desc.c_str(), e, e);
    exit(20);
  }
}

void CalcNormal(glm::vec3& normal, const glm::vec3& v0, const glm::vec3& v1,
                const glm::vec3& v2) {
  const glm::vec3 v10{v1 - v0};
  const glm::vec3 v20{v2 - v0};

  const glm::vec3 n{glm::cross(v10, v20)};
  normal = glm::normalize(n);
}


namespace  // Local utility functions
{

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
                             std::map<int, glm::vec3>& smoothVertexNormals) {
  smoothVertexNormals.clear();
  std::map<int, glm::vec3>::iterator iter;

  SPDLOG_INFO("computeSmoothingNormals");

  for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
    // Get the three indexes of the face (all faces are triangular)
    tinyobj::index_t idx0 = shape.mesh.indices[3 * f + 0];
    tinyobj::index_t idx1 = shape.mesh.indices[3 * f + 1];
    tinyobj::index_t idx2 = shape.mesh.indices[3 * f + 2];

    // Get the three vertex indexes and coordinates
    int vi[3];      // indexes
    // float v[3][3];  // coordinates
    glm::vec3 v[3];  // coordinates

    // for (int k = 0; k < 3; k++) {
    //   vi[0] = idx0.vertex_index;
    //   vi[1] = idx1.vertex_index;
    //   vi[2] = idx2.vertex_index;
    //   assert(vi[0] >= 0);
    //   assert(vi[1] >= 0);
    //   assert(vi[2] >= 0);

    //   v[0][k] = attrib.vertices[3 * vi[0] + k];
    //   v[1][k] = attrib.vertices[3 * vi[1] + k];
    //   v[2][k] = attrib.vertices[3 * vi[2] + k];
    // }

      vi[0] = idx0.vertex_index;
      vi[1] = idx1.vertex_index;
      vi[2] = idx2.vertex_index;
      assert(vi[0] >= 0);
      assert(vi[1] >= 0);
      assert(vi[2] >= 0);
      for (int k = 0; k < 3; k++) {
        v[k] = glm::vec3{
          attrib.vertices[3 * vi[k] + 0],
          attrib.vertices[3 * vi[k] + 1],
          attrib.vertices[3 * vi[k] + 2]
        };
      }

    // Compute the normal of the face
    glm::vec3 normal;
    CalcNormal(normal, v[0], v[1], v[2]);

    // Add the normal to the three vertexes
    for (size_t i = 0; i < 3; ++i) {
      iter = smoothVertexNormals.find(vi[i]);
      if (iter != smoothVertexNormals.end()) {
        // add
        iter->second += normal;
      } else {
        smoothVertexNormals[vi[i]] = normal;
      }
    }

  }  // for faces

  // Normalize the normals, that is, make them unit vectors
  // for (iter = smoothVertexNormals.begin(); iter != smoothVertexNormals.end();
  //      iter++) {
  //   normalizeVector(iter->second);
  // }
  for (auto& pair : smoothVertexNormals) {
    pair.second = glm::normalize(pair.second);
  }

}  // computeSmoothingNormals

static void computeAllSmoothingNormals(tinyobj::attrib_t& attrib,
                                       std::vector<tinyobj::shape_t>& shapes) {
  glm::vec3 p[3];
  for (size_t s = 0, slen = shapes.size(); s < slen; ++s) {
    const tinyobj::shape_t& shape(shapes[s]);
    size_t facecount = shape.mesh.num_face_vertices.size();
    assert(shape.mesh.smoothing_group_ids.size());

    for (size_t f = 0, flen = facecount; f < flen; ++f) {
      for (unsigned int v = 0; v < 3; ++v) {
        tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
        assert(idx.vertex_index != -1);
        p[v] = glm::vec3{
          attrib.vertices[3 * idx.vertex_index + 0],
          attrib.vertices[3 * idx.vertex_index + 1],
          attrib.vertices[3 * idx.vertex_index + 2]
        };
      }

      // cross(p[1] - p[0], p[2] - p[0])

      glm::vec3 p10 = p[1] - p[0];
      glm::vec3 p20 = p[2] - p[0];
      glm::vec3 n = glm::cross(p10, p20);

      // Don't normalize here.
      for (unsigned int v = 0; v < 3; ++v) {
        tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
        attrib.normals[3 * idx.normal_index] += n.x;
        attrib.normals[3 * idx.normal_index + 1] += n.y;
        attrib.normals[3 * idx.normal_index + 2] += n.z;
      }
    }
  }

  assert(attrib.normals.size() % 3 == 0);
  for (size_t i = 0, nlen = attrib.normals.size() / 3; i < nlen; ++i) {
    tinyobj::real_t& nx = attrib.normals[3 * i + 0];
    tinyobj::real_t& ny = attrib.normals[3 * i + 1];
    tinyobj::real_t& nz = attrib.normals[3 * i + 2];
    tinyobj::real_t len = std::sqrtf(nx * nx + ny * ny + nz * nz);
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

static void computeSmoothingShapes(
    const tinyobj::attrib_t& inattrib,
    const std::vector<tinyobj::shape_t>& inshapes,
    std::vector<tinyobj::shape_t>& outshapes, tinyobj::attrib_t& outattrib) {
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

class Extent {
public:
  void Update(glm::vec3 vec) {
    min_extent = glm::min(min_extent, vec);
    max_extent = glm::max(max_extent, vec);
  }
    
  float MaxMidpoint() const {
    float max_midpoint{0.5f * (max_extent.x - min_extent.x)};
    float tmp_y{0.5f * (max_extent.y - min_extent.y)};
    if (max_midpoint < tmp_y) {
      max_midpoint = tmp_y;
    }
    float tmp_z{0.5f * (max_extent.z - min_extent.z)};
    if (max_midpoint < tmp_z) {
      max_midpoint = tmp_z;
    }
    return max_midpoint;
  }

  glm::vec3 CenterOffset() {
    const glm::vec3 center{(max_extent + min_extent) * -0.5f};
    return center;
  }

  std::string ToString() {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "\nMinExtent: (" << min_extent.x << ", " << min_extent.y << ", "
        << min_extent.z << ")\n";
    oss << "MaxExtent: (" << max_extent.x << ", " << max_extent.y << ", "
        << max_extent.z << ")";
    return oss.str();
  }

private:
  glm::vec3 min_extent{std::numeric_limits<tinyobj::real_t>::max()};
  glm::vec3 max_extent{std::numeric_limits<tinyobj::real_t>::min()};
};

void LoadDiffuseTexture(const tinyobj::material_t* mp,
                        const std::filesystem::path& base_dir,
                        std::map<std::string, GLuint>& textures) {
  if (mp->diffuse_texname.length() > 0) {
    // Only load the texture if it is not already loaded
    if (textures.find(mp->diffuse_texname) == textures.end()) {
      GLuint texture_id;
      int w, h;
      int comp;

      const std::string texture_filename{mp->diffuse_texname};
      const std::filesystem::path texture_path{base_dir / texture_filename};
      SPDLOG_INFO("Working on texture filename: {}", texture_path.string());

      if (!std::filesystem::exists(texture_path)) {
          SPDLOG_ERROR("Unable to find file: {}", texture_path.string());
          exit(1);
      }

      unsigned char* image =
          stbi_load(texture_path.c_str(), &w, &h, &comp, STBI_default);
      if (!image) {
        SPDLOG_ERROR("Unable to load texture: {}", texture_path.string());
        exit(1);
      }
      SPDLOG_INFO("Loaded texture: {}, w = {}, h = {}, comp = {}", texture_filename, w, h, comp);

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

void WriteBufferTest(const std::string& filename,
                     const std::vector<tinyobj::real_t>& buffer) {
  std::ofstream fh{filename};
  if (!fh) {
    SPDLOG_ERROR("Failed to open {}", filename);
    exit(1);
  }
  fh << std::fixed << std::setprecision(3);
  int i{0};
  for (const auto& val : buffer) {
    fh << i++ << ": " << val << "\n";
  }
  fh.close();
}


bool HasNan(const glm::vec3& vec) {
  // Check if any component of vec is NAN
  glm::bvec3 nan_components{glm::isnan(vec)};
  bool has_nan{glm::any(nan_components)};
  return has_nan;
}

// bool LoadObjAndConvert(glm::vec3& bmin, glm::vec3& bmax,
bool LoadObjAndConvert(Extent& box,
                       std::vector<DrawObject>* drawObjects,
                       std::vector<tinyobj::material_t>& _materials,
                       std::map<std::string, GLuint>& textures,
                       const char* inputfile) {
  const std::filesystem::path inputfile_path{inputfile};
  auto base_dir{inputfile_path.parent_path()};

  if (! base_dir.has_root_path()) {
    base_dir = std::filesystem::path{"."} / base_dir;
  }
  SPDLOG_INFO("Base directory: {}", base_dir.string());

  // defaults to triangulate(true), triangulation_method("simple"),
  // vertex_color(true)
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = base_dir;  // Path to material files

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
      SPDLOG_ERROR("TinyObjReader: {}", reader.Error());
    }
    return false;
  }
  assert(reader.Valid());
  if (!reader.Warning().empty()) {
    SPDLOG_WARN("TinyObjReader: {}", reader.Warning());
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  SPDLOG_INFO("# of vertices  = {}", (int)(attrib.vertices.size()) / 3);
  SPDLOG_INFO("# of normals   = {}", (int)(attrib.normals.size()) / 3);
  SPDLOG_INFO("# of texcoords = {}", (int)(attrib.texcoords.size()) / 2);
  SPDLOG_INFO("# of materials = {}", (int)materials.size());
  SPDLOG_INFO("# of shapes    = {}", (int)shapes.size());

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
  // bmin = glm::vec3{std::numeric_limits<tinyobj::real_t>::max()};
  // bmax = glm::vec3{std::numeric_limits<tinyobj::real_t>::min()};

  // Set to true to always regen normals
  bool regen_all_normals = attrib.normals.size() == 0;

  tinyobj::attrib_t outattrib;
  std::vector<tinyobj::shape_t> outshapes;

  if (regen_all_normals) {
    SPDLOG_INFO("Regenerate all normals!");
    computeSmoothingShapes(attrib, shapes, outshapes, outattrib);
    computeAllSmoothingNormals(outattrib, outshapes);
  } else {
    outshapes = shapes;
    outattrib = attrib;
  }

  if (regen_all_normals) {
    SPDLOG_INFO("After normal regeneration.");
    SPDLOG_INFO("# of vertices  = {}", (int)(outattrib.vertices.size()) / 3);
    SPDLOG_INFO("# of normals   = {}", (int)(outattrib.normals.size()) / 3);
    SPDLOG_INFO("# of texcoords = {}", (int)(outattrib.texcoords.size()) / 2);
    SPDLOG_INFO("# of materials = {}", (int)materials.size());
    SPDLOG_INFO("# of shapes    = {}", (int)outshapes.size());
  }

  int buffer_name_count{0};
  // Loop over shapes
  for (size_t s = 0; s < outshapes.size(); s++) {
    DrawObject draw_object;
    // pos(3float), normal(3float), color(3float)
    std::vector<tinyobj::real_t> buffer;

    // Check for smoothing group and compute smoothing normals
    std::map<int, glm::vec3> smoothVertexNormals;
    if (!regen_all_normals && (hasSmoothingGroup(outshapes[s]) > 0)) {
      SPDLOG_INFO("Compute smoothingNormal for shape [{}]", s);
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
        tinyobj::index_t idx =
            outshapes.at(s).mesh.indices.at(index_offset + v);

        // tinyobj::real_t vx =
        //     outattrib.vertices.at(3 * size_t(idx.vertex_index) + 0);
        // tinyobj::real_t vy =
        //     outattrib.vertices.at(3 * size_t(idx.vertex_index) + 1);
        // tinyobj::real_t vz =
        //     outattrib.vertices.at(3 * size_t(idx.vertex_index) + 2);

        const glm::vec3 vertex{
          outattrib.vertices.at(3 * size_t(idx.vertex_index) + 0),
          outattrib.vertices.at(3 * size_t(idx.vertex_index) + 1),
          outattrib.vertices.at(3 * size_t(idx.vertex_index) + 2)
        };

        // Check if any component of vertex is NAN
        bool has_nan{HasNan(vertex)};
        assert(not has_nan);
        // assert(vx != NAN && vy != NAN && vz != NAN);

        // vertices[v] = glm::vec3{vx, vy, vz};
        vertices[v] = vertex;

        box.Update(vertex);
        // bmin.x = glm::min(bmin.x, vertex.x);
        // bmin.y = glm::min(bmin.y, vertex.y);
        // bmin.z = glm::min(bmin.z, vertex.z);

        // bmax.x = glm::max(bmax.x, vertex.x);
        // bmax.y = glm::max(bmax.y, vertex.y);
        // bmax.z = glm::max(bmax.z, vertex.z);

        // Check if `normal_index` is zero or positive. negative = no normal
        // data
        if (idx.normal_index >= 0) {
          // tinyobj::real_t nx =
          //     outattrib.normals.at(3 * size_t(idx.normal_index) + 0);
          // tinyobj::real_t ny =
          //     outattrib.normals.at(3 * size_t(idx.normal_index) + 1);
          // tinyobj::real_t nz =
          //     outattrib.normals.at(3 * size_t(idx.normal_index) + 2);
          const glm::vec3 normal{
            outattrib.normals.at(3 * size_t(idx.normal_index) + 0),
            outattrib.normals.at(3 * size_t(idx.normal_index) + 1),
            outattrib.normals.at(3 * size_t(idx.normal_index) + 2)
          };
          // assert(nx != NAN && ny != NAN && nz != NAN);
          // Check if any component of vertex is NAN
          bool has_nan{HasNan(normal)};
          assert(not has_nan);

          // normals[v] = glm::vec3{nx, ny, nz};
          normals[v] = normal;
        }

        // Check if `texcoord_index` is zero or positive. negative = no texcoord
        // data
        if (idx.texcoord_index >= 0) {
          bool flip_y_coord{true};
          if (flip_y_coord) {
            const glm::vec2 tex_coord {
              outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 0),
              1.0f - outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 1)
            };
            texcoords[v] = tex_coord;
          } else {
            const glm::vec2 tex_coord{
              outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 0),
              outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 1)
            };
            texcoords[v] = tex_coord;
          }
          // tinyobj::real_t tx =
          //     outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 0);
          // tinyobj::real_t ty =
          //     outattrib.texcoords.at(2 * size_t(idx.texcoord_index) + 1);

          // Flip Y coordinate
          // tinyobj::real_t tx = outattrib.texcoords.at(2 * idx.texcoord_index);
          // tinyobj::real_t ty =
          //     1.0f - outattrib.texcoords.at(2 * idx.texcoord_index + 1);

          // texcoords[v] = glm::vec2{tx, ty};
        }

        // Optional: vertex colors
        // tinyobj::real_t red   =
        // outattrib.colors[3*size_t(idx.vertex_index)+0]; tinyobj::real_t green
        // = outattrib.colors[3*size_t(idx.vertex_index)+1]; tinyobj::real_t
        // blue  = outattrib.colors[3*size_t(idx.vertex_index)+2];

      }  // end loop over vertices in the face
      // index_offset counts up by 3 but fv could be something other than 3
      // there is an assert to make sure it increases by 3, triangulation
      // is on by default
      index_offset += fv;

      // per-face material
      // shapes[s].mesh.material_ids[f];
      const int current_material_id{outshapes.at(s).mesh.material_ids.at(f)};
      if ((current_material_id < 0) ||
          (current_material_id >= static_cast<int>(materials.size()))) {
        SPDLOG_ERROR("Current shape {} missing a material.", s);
        return false;
      }

      const glm::vec3 diffuse{
          materials.at(current_material_id).diffuse[0],
          materials.at(current_material_id).diffuse[1],
          materials.at(current_material_id).diffuse[2],
      };

      const float normal_factor = 0.2;
      const float diffuse_factor = 1 - normal_factor;
      glm::vec3 colors[3];

      for (int k = 0; k < 3; k++) {
        const glm::vec3 color{
            normals[k].x * normal_factor + diffuse.r * diffuse_factor,
            normals[k].y * normal_factor + diffuse.g * diffuse_factor,
            normals[k].z * normal_factor + diffuse.b * diffuse_factor};
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

    }  // end Loop over faces(polygon) 3 at a time

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

      SPDLOG_INFO("shape[{}] # of triangles = {}", static_cast<int>(s),
             draw_object.numTriangles);
    }

    if (false) {
      std::ostringstream file_name_stream;
      file_name_stream << "buffer-view-" << buffer_name_count++ << ".txt";
      WriteBufferTest(file_name_stream.str(), buffer);
    }

    drawObjects->push_back(draw_object);
  }  // end for every shape

  _materials = materials;

  // printf("bmin = %f, %f, %f\n", bmin.x, bmin.y, bmin.z);
  // printf("bmax = %f, %f, %f\n", bmax.x, bmax.y, bmax.z);
  SPDLOG_INFO("{}", box.ToString());
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
      trackball_(prev_quat_, glm::vec2{0, 0}, glm::vec2{0, 0});

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

  const glm::dvec2 mouse{mouse_x, mouse_y};

  if (mouseLeftPressed) {
    trackball(prev_quat, rotScale * (2.0f * prev_mouse.x - width) / (float)width,
              rotScale * (height - 2.0f * prev_mouse.y) / (float)height,
              rotScale * (2.0f * mouse.x - width) / (float)width,
              rotScale * (height - 2.0f * mouse.y) / (float)height);
    glm::vec2 p1{
      rotScale * (2.0f * prev_mouse.x - width) / (float)width,
      rotScale * (height - 2.0f * prev_mouse.y) / (float)height
    };
    glm::vec2 p2{
      rotScale * (2.0f * mouse.x - width) / (float)width,
      rotScale * (height - 2.0f * mouse.y) / (float)height
    };
    trackball_(prev_quat_, p1, p2);

    add_quats(prev_quat, curr_quat, curr_quat);
    curr_quat_ = glm::normalize(prev_quat_ * curr_quat_);

  } else if (mouseMiddlePressed) {
    // eye[0] -= transScale * (mouse_x - prevMouseX) / (float)width;
    // lookat[0] -= transScale * (mouse_x - prevMouseX) / (float)width;
    // eye[1] += transScale * (mouse_y - prevMouseY) / (float)height;
    // lookat[1] += transScale * (mouse_y - prevMouseY) / (float)height;

    eye.x -= transScale * (mouse.x - prev_mouse.x) / (float)width;
    lookat.x -= transScale * (mouse.x - prev_mouse.x) / (float)width;

    eye.y += transScale * (mouse.y - prev_mouse.y) / (float)height;
    lookat.y += transScale * (mouse.y - prev_mouse.y) / (float)height;
  } else if (mouseRightPressed) {
    // eye[2] += transScale * (mouse_y - prevMouseY) / (float)height;
    // lookat[2] += transScale * (mouse_y - prevMouseY) / (float)height;

    eye.z += transScale * (mouse.y - prev_mouse.y) / (float)height;
    lookat.z += transScale * (mouse.y - prev_mouse.y) / (float)height;
  }

  // Update mouse point
  // prevMouseX = mouse_x;
  // prevMouseY = mouse_y;

  prev_mouse = mouse;
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
  trackball_(curr_quat_, glm::vec2{0, 0}, glm::vec2{0, 0});

  eye = glm::vec3{0.0f, 0.0f, 3.0f};
  // eye[0] = 0.0f;
  // eye[1] = 0.0f;
  // eye[2] = 3.0f;

  lookat = glm::vec3{0.0f, 0.0f, 0.0f};
  // lookat[0] = 0.0f;
  // lookat[1] = 0.0f;
  // lookat[2] = 0.0f;

  up = glm::vec3{0.0f, 1.0f, 0.0f};
  // up[0] = 0.0f;
  // up[1] = 1.0f;
  // up[2] = 0.0f;
}

void GLVersion(bool do_dump_extensions, std::ostream& out) {
  const char* gl_vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
  const char* gl_renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
  const char* gl_version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
  const char* gl_shading_language_version = reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION));
  spdlog::info("Vendor: {}", gl_vendor);
  spdlog::info("Renderer: {}", gl_renderer);
  spdlog::info("OpenGL Version: {}", gl_version);
  spdlog::info("GLSL Version: {}", gl_shading_language_version);

  GLint major{0};
  GLint minor{0};
  GLint samples{0};
  GLint sample_buffers{0};
  // glGetIntegerv(GL_MAJOR_VERSION, &major);
  // glGetIntegerv(GL_MINOR_VERSION, &minor);
  glGetIntegerv(GL_SAMPLES, &samples);
  glGetIntegerv(GL_SAMPLE_BUFFERS, &sample_buffers);

  spdlog::info("-----------------------------------------------");
  // spdlog::info("GL Version   : {}.{}", major, minor);
  spdlog::info("MSAA samples : {}", samples);
  spdlog::info("MSAA buffers : {}", sample_buffers);
  spdlog::info("-----------------------------------------------");

  if (do_dump_extensions) {
    // GLint num_extensions{0};
    // glGetIntegerv(GL_NUM_EXTENSIONS, &num_extensions);
    // for (int i = 0; i < num_extensions; i++) {
    //   out << glGetStringi(GL_EXTENSIONS, i) << "\n";
    //   // fprintf(stderr, "%s\n", glGetStringi(GL_EXTENSIONS, i));
    // }
  }
}

int main(int argc, char** argv) {

  spdlog::set_level(spdlog::level::debug);

  if (argc < 2) {
    SPDLOG_ERROR("Needs input.obj");
    return 0;
  }

  Init();


  if (!glfwInit()) {
    SPDLOG_ERROR("Failed to initialize GLFW.");
    return -1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);

  window = glfwCreateWindow(width, height, "Obj viewer", NULL, NULL);
  if (window == NULL) {
    SPDLOG_ERROR("Failed to open GLFW window. ");
    glfwTerminate();
    return 1;
  }


  SPDLOG_INFO("W : Toggle wireframe");
  SPDLOG_INFO("C : Toggle face culling");
  // std::cout << "K, J, H, L, P, N : Move camera\n";
  SPDLOG_INFO("Q, Esc : quit");

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // Callback
  glfwSetWindowSizeCallback(window, reshapeFunc);
  glfwSetKeyCallback(window, keyboardFunc);
  glfwSetMouseButtonCallback(window, clickFunc);
  glfwSetCursorPosCallback(window, motionFunc);

  // glewExperimental = true;
  // if (glewInit() != GLEW_OK) {
  //   SPDLOG_ERROR("Failed to initialize GLEW.");
  //   return -1;
  // }

  int version_ = gladLoadGL(glfwGetProcAddress);
  if (version_ == 0) {
    SPDLOG_ERROR("Could not load OpenGL functions.");
    return -1;
  }


  GLVersion(false, std::cerr);

  reshapeFunc(window, width, height);

  // float bmin[3] = {-71.892532, 0.000000, -47.928356};
  // float bmax[3] = {82.293999, 79.006561, 47.928356};
  // glm::vec3 bmin;
  // glm::vec3 bmax;
  Extent box;
  std::vector<tinyobj::material_t> materials;
  std::map<std::string, GLuint> textures;
  // if (false == LoadObjAndConvert(bmin, bmax, &gDrawObjects, materials, textures,
  //                                argv[1])) {
  //   return -1;
  // }

  if (false == LoadObjAndConvert(box, &gDrawObjects, materials, textures,
                                 argv[1])) {
    return -1;
  }

  SPDLOG_INFO("Textures:");
  for (const auto& pair : textures) {
    SPDLOG_INFO("{}, {}", pair.first, pair.second);
  }

  // MS: compute bounding box

  float maxExtent{box.MaxMidpoint()};
  // float maxExtent = 0.5f * (bmax[0] - bmin[0]);
  // if (maxExtent < 0.5f * (bmax[1] - bmin[1])) {
  //   maxExtent = 0.5f * (bmax[1] - bmin[1]);
  // }
  // if (maxExtent < 0.5f * (bmax[2] - bmin[2])) {
  //   maxExtent = 0.5f * (bmax[2] - bmin[2]);
  // }

  while (glfwWindowShouldClose(window) == GL_FALSE) {
    glfwPollEvents();
    glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    // camera & rotate
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // GLfloat mat[4][4];
    glm::mat4 modelview{1.0f};

    const auto lookat_matrix = glm::lookAt(eye, lookat, up);
    // gluLookAt(eye[0], eye[1], eye[2], lookat[0], lookat[1], lookat[2], up[0],
              // up[1], up[2]);
    // glMultMatrixf(glm::value_ptr(lookat_matrix));
    
    // build_rotmatrix(mat, curr_quat);
    // glMultMatrixf(&mat[0][0]);
    // glm::mat4 rotation_matrix = glm::make_mat4(&mat[0][0]);

    // requires glm/gtx/quaternion.hpp
    // const auto rotation_matrix_ = glm::toMat4(curr_quat_);
    const auto rotation_matrix_ = glm::mat4_cast(curr_quat_);

    // std::cerr << "Main SGI Quat: " << curr_quat[0] << " " << curr_quat[1] << " " << curr_quat[2] << " " << curr_quat[3] << "\n";
    // std::cerr << "Main GLM Quat: " << curr_quat_.x << " " << curr_quat_.y << " " << curr_quat_.z << " " << curr_quat_.w << "\n";

    // if (glm::all(glm::equal(modelview, rotation_matrix, 0.000001f))) {
    //   std::cerr << "They are the same\n";
    // } else {
    //   std::cerr << "They are different\n";
    //   std::cerr << "SGI\n" << glm::to_string(rotation_matrix) << "\n\nGLM\n" <<
    //   glm::to_string(rotation_matrix_) << "\n";
    //   // exit(1);
    // }

    // Fit to -1, 1
    // glScalef(1.0f / maxExtent, 1.0f / maxExtent, 1.0f / maxExtent);
    auto scale_matrix{glm::scale(glm::vec3{(1.0f / maxExtent)})};
    // glMultMatrixf(value_ptr(scale_matrix));

    // Centerize object.
    const glm::vec3 center_offset{box.CenterOffset()};
    const auto translate_matrix{glm::translate(center_offset)};
    // glMultMatrixf(value_ptr(translate_matrix));
    // glTranslatef(-0.5 * (bmax[0] + bmin[0]), -0.5 * (bmax[1] + bmin[1]),
    //              -0.5 * (bmax[2] + bmin[2]));

    modelview = lookat_matrix * rotation_matrix_ * scale_matrix * translate_matrix;
    // glMultMatrixf(glm::value_ptr(modelview));
    glLoadMatrixf(glm::value_ptr(modelview));

    Draw(gDrawObjects, materials, textures);

    glfwSwapBuffers(window);
  }

  glfwTerminate();
}
