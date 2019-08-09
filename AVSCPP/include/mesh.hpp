#pragma once

// System Headers
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <glad/glad.h>
#include <glm/glm.hpp>

// Standard Headers
#include <map>
#include <memory>
#include <vector>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Define Namespace
namespace AVSCPP
{
    // Vertex Format
    struct Vertex {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 uv;
    };

    class Mesh
    {
    public:

        // Implement Default Constructor and Destructor
         Mesh() { glGenVertexArrays(1, & mVertexArray); }
        ~Mesh() { glDeleteVertexArrays(1, & mVertexArray); }

        // Implement Custom Constructors
        Mesh(std::string const & filename);
        Mesh(std::vector<Vertex> const & vertices,
             std::vector<GLuint> const & indices,
             std::map<GLuint, std::string> const & textures);

        // Public Member Functions
        void draw(GLuint shader);
        void setModelMatrix(glm::mat4 mat) {ModelMatrix = mat;}
        glm::mat4 getModelMatrix() {return ModelMatrix;}

        GLuint getSize() { // Recursively compute size
            GLuint size = mIndices.size();
            for (auto &i : mSubMeshes) {
                size = size + i->getSize();
            }
            return size;
        }

        std::vector<GLfloat> getBoundingBox(std::vector<GLfloat> bounds);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() {return pointCloud;}

    private:

        // Disable Copying and Assignment
        // Mesh(Mesh const &) = delete;
        // Mesh & operator=(Mesh const &) = delete;

        // Private Member Functions
        void parse(std::string const & path50, aiNode const * node, aiScene const * scene);
        void parse(std::string const & path, aiMesh const * mesh, aiScene const * scene);
        std::map<GLuint, std::string> process(std::string const & path,
                                              aiMaterial * material,
                                              aiTextureType type);

        // Private Member Containers
        std::vector<std::unique_ptr<Mesh>> mSubMeshes;
        std::vector<GLuint> mIndices;
        std::vector<Vertex> mVertices;
        std::map<GLuint, std::string> mTextures;

        // Private Member Variables
        GLuint mVertexArray;
        GLuint mVertexBuffer;
        GLuint mElementBuffer;

        glm::mat4 ModelMatrix;

        // PCL variant of data
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    };
}
