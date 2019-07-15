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

void processInput(GLFWwindow *window);
GLfloat pixelIndex(GLfloat *b, int w, int h, int c);


// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

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
    GLuint modelSize = modelPoints.getSize();

    // Create Model-View-Projection Matrix:
    AVSCPP::CameraControl camera(mWindow);

    // Initialise VAO
    GLuint VertexArrayObject;
	glGenVertexArrays(1, &VertexArrayObject);
	glBindVertexArray(VertexArrayObject);

    // screen quad VAO
    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };
    unsigned int quadVAO, quadVBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));


    // Create and compile our GLSL program from the shaders
    std::string shaderPath = PROJECT_SOURCE_DIR "/AVSCPP/shaders/";
    AVSCPP::Shader shader;
    shader.attach(shaderPath+"simplevertexshader.vert")
        //   .attach(shaderPath+"simplegeometryshader.geom")
          .attach(shaderPath+"simplefragmentshader.frag")
          .link();

    AVSCPP::Shader texshader;
    texshader.attach(shaderPath+"2dtexvertexshader.vert")
             .attach(shaderPath+"2dtexfragshader.frag")
             .link();
    texshader.activate();
    texshader.bind("screenTexture", 0);

    AVSCPP::Shader projshader;
    projshader.attach(shaderPath+"2dbackprojectvert.vert")
              .attach(shaderPath+"2dbackprojectfrag.frag")
              .link();
    projshader.activate();
    projshader.bind("screenTexture", 0);
    projshader.bind("depthrange", glm::vec2(0.0, 1.0));
    projshader.bind("persMatrix", camera.getProjectionMatrix());
    glm::vec2 hsnp = glm::vec2(glm::tan(camera.getfov()/2.0)*camera.getAspect(), glm::tan(camera.getfov()/2.0));
    projshader.bind("halfSizeNearPlane", hsnp);

    // Create frame buffer
    GLuint framebuffer1;
    glGenFramebuffers(1, &framebuffer1);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1);    

    // generate texture
    GLuint texColorBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mWidth, mHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    GLuint texDepthBuffer;
    glGenTextures(1, &texDepthBuffer);
    glBindTexture(GL_TEXTURE_2D, texDepthBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    // attach it to currently bound framebuffer object
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepthBuffer, 0);

    // Check framebuffer is correctly set up then unbind the framebuffer
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  

    // Create another framebuffer for back projection
    GLuint framebuffer2;
    glGenFramebuffers(1, &framebuffer2);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
    GLuint projtexColorBuffer;
    glGenTextures(1, &projtexColorBuffer);
    glBindTexture(GL_TEXTURE_2D, projtexColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mWidth, mHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, projtexColorBuffer, 0);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  


    GLfloat * pixelArray = new GLfloat[mHeight * mWidth * 4];
   
    while (!glfwWindowShouldClose(mWindow))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(mWindow);

        // First pass into framebuffer with original shaders
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1);
        glViewport(0, 0, mWidth, mHeight);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
    //     // camera.setViewMatrix(glm::lookAt(
    //     //             glm::vec3(25, -25, 25), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0)
    //     //         ));

        // Activate Shader
        shader.activate(); // glUseProgram

        // Compute the MVP matrix from keyboard and mouse input
        camera.computeMatricesFromInputs();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        glm::mat4 MVP = camera.getVPMatrix() * ModelMatrix;
        shader.bind("MVP", MVP);

        // Draw Model
        modelPoints.draw(shader.get());

        // 2nd pass backprojection
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
        glViewport(0, 0, mWidth, mHeight);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
        projshader.activate();
        glBindVertexArray(quadVAO);
        glBindTexture(GL_TEXTURE_2D, texDepthBuffer);	// use the depth attachment texture as the texture of the quad plane
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
        glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_FLOAT, pixelArray);

        for (int i = 0; i < 4; i++) {
            printf("%f ", pixelIndex(pixelArray, mWidth/2, mHeight/2, i));        
        }
        printf("\n");

        // third pass (render screen)
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // back to default framebuffer
        glViewport(0, 0, mWidth, mHeight);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f); 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);

        texshader.activate(); // activate 2d texture shader
        glBindVertexArray(quadVAO);
        // glBindTexture(GL_TEXTURE_2D, texColorBuffer);	// use the color attachment texture as the texture of the quad plane
        glBindTexture(GL_TEXTURE_2D, texDepthBuffer);	// use the depth attachment texture as the texture of the quad plane
        glDrawArrays(GL_TRIANGLES, 0, 6);


        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &quadVBO);

    free(pixelArray);

    // glfwTerminate(); // Causes segfault on my machine? 

    return EXIT_SUCCESS;
}


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

}

GLfloat pixelIndex(GLfloat *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}