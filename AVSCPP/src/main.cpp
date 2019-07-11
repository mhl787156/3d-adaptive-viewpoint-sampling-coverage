// Local Headers
#include "avscpp.hpp"

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>

// Local Library Headers
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"

int main(int argc, char * argv[]) {

    // Load GLFW and Create a Window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    auto mWindow = glfwCreateWindow(mWidth, mHeight, "OpenGL", nullptr, nullptr);

    // Check for Valid Context
    if (mWindow == nullptr) {
        fprintf(stderr, "Failed to Create OpenGL Context");
        return EXIT_FAILURE;
    }

    // Create Context and Load OpenGL Functions
    glfwMakeContextCurrent(mWindow);
    gladLoadGL();
    fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

    // Read in .obj filec
    std::string const modelPath = PROJECT_SOURCE_DIR "/resources/models/cube.obj";
    AVSCPP::Mesh modelPoints(modelPath);

    // Initialise VAO
    GLuint VertexArrayObject;
	glGenVertexArrays(1, &VertexArrayObject);
	glBindVertexArray(VertexArrayObject);

    // Enable depth test
	// glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	// glDepthFunc(GL_LESS); 
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // Create and compile our GLSL program from the shaders
    std::string shaderPath = PROJECT_SOURCE_DIR "/AVSCPP/shaders/";
    AVSCPP::Shader shader;
    shader.attach(shaderPath+"simplevertexshader.vert")
          .attach(shaderPath+"simplegeometryshader.geom")
          .attach(shaderPath+"simplefragmentshader.frag")
          .link();

    // Create Model-View-Projection Matrix:
    AVSCPP::CameraControl camera(mWindow);

    GLuint modelSize = modelPoints.getSize();

    // Initialise Shader Storage Buffer Object
    glm::vec4 shader_data[modelSize];
    GLuint ssbo;
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, modelSize*sizeof(shader_data), shader_data, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo);
    // Read from sso
    GLuint block_index = 0;
    block_index = glGetProgramResourceIndex(shader.get(), GL_SHADER_STORAGE_BLOCK, "shader_data");
    GLuint ssbo_binding_point_index = 3;
    glShaderStorageBlockBinding(shader.get(), block_index, ssbo_binding_point_index);
    
    camera.setViewMatrix(glm::lookAt(
                glm::vec3(50, 50, 50), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0)
            ));

    // Rendering Loop
    while (glfwWindowShouldClose(mWindow) == false) {
        if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(mWindow, true);

        if (glfwGetKey(mWindow, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera.toggleControl();

        if (glfwGetKey(mWindow, GLFW_KEY_A) == GLFW_PRESS)
            camera.resetView();

        if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS)
            camera.setViewMatrix(glm::lookAt(
                glm::vec3(50, 50, 50), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0)
            ));

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Activate Shader
        shader.activate(); // glUseProgram

        // Compute the MVP matrix from keyboard and mouse input
		camera.computeMatricesFromInputs();
		glm::mat4 ModelMatrix = glm::mat4(1.0);
		glm::mat4 MVP = camera.getVPMatrix() * ModelMatrix;

        // Activate shader and bind MVP uniform
        shader.bind("MVP", MVP);

        // Draw Model
        modelPoints.draw(shader.get());

        // Flip Buffers and Draw
        glfwSwapBuffers(mWindow);
        glfwPollEvents();

        break;
    }   
    
    
    for (unsigned int i = 0; i < modelSize; i++) {
        glm::vec4 out;
        glNamedBufferSubData(ssbo, i * sizeof(glm::vec4), sizeof(glm::vec4), &out);
        printf("%s\n", glm::to_string(out).c_str());

    }


    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // unbind sso



    // glfwTerminate(); // Causes segfault on my machine? 

    return EXIT_SUCCESS;
}
