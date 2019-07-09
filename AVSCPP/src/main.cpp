// Local Headers
#include "avscpp.hpp"

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>

// Local Library Headers
#include "shader.hpp"
#include "mesh.hpp"

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
    std::string const modelPath = PROJECT_SOURCE_DIR "/resources/models/aeroplane.obj";
    AVSCPP::Mesh modelPoints(modelPath);

    // Initialise VAO
    GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

    // Enable depth test
	// glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	// glDepthFunc(GL_LESS); 
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

    // Create and compile our GLSL program from the shaders
    std::string shaderPath = PROJECT_SOURCE_DIR "/AVSCPP/shaders/";
    AVSCPP::Shader shader;
    shader.attach(shaderPath+"simplevertexshader.vert")
          .attach(shaderPath+"simplefragmentshader.frag")
          .link();

    // Activate Shader
    shader.activate(); // glUseProgram

    // Create Model-View-Projection Matrix:
    glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    glm::mat4 View       = glm::lookAt(
								glm::vec3(0,50,50), // Camera is at (4,3,3), in World Space
								glm::vec3(0,0,0), // and looks at the origin
								glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
						   );
    glm::mat4 Model      = glm::mat4(1.0f);
    glm::mat4 MVP        = Projection * View * Model; // Remember, matrix multiplication is the other way around

    // Rendering Loop
    while (glfwWindowShouldClose(mWindow) == false) {
        if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(mWindow, true);

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Activate shader and bind MVP uniform
        shader.bind("MVP", MVP);

        // Draw Model
        modelPoints.draw(shader.get());

        // Flip Buffers and Draw
        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }   

    // glfwTerminate();

    return EXIT_SUCCESS;
}
