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
#include <utility> 

// Local Library Headers
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"

void processInput(GLFWwindow *window);
GLfloat pixelIndex(GLfloat *b, int w, int h, int c);
GLint pixelIndex(GLint *b, int w, int h, int c);


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

    std::string const floorPath = PROJECT_SOURCE_DIR "/resources/models/plane.obj";
    AVSCPP::Mesh floorMesh(floorPath);

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
    glm::vec2 hsnp = glm::vec2(glm::tan(camera.getfov()/2.0)*camera.getAspect(), glm::tan(camera.getfov()/2.0));
    projshader.bind("halfSizeNearPlane", hsnp);
    projshader.bind("screenTexture", 0);
    projshader.bind("depthrange", glm::vec2(0.0, 1.0));

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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32I, mWidth, mHeight, 0, GL_RGBA_INTEGER, GL_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, projtexColorBuffer, 0);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  

    // SSBO for storing backprojection data per pixel
    
    // struct shader_data_t
    // {
        // float lol;
        // glm::vec4 pixelLocs[mWidth][mHeight];
    // };

    // std::shared_ptr<shader_data_t> shader_data(new shader_data_t());
    
    // Create SSBO
    // projshader.activate();
    // glBindVertexArray(quadVAO);
    // GLuint ssbo;
    // GLuint ssbo_binding_point_index = 2;
    // GLuint ssbo_block_index = 0;
    // glGenBuffers(1, &ssbo);
    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    // glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(shader_data_t), nullptr, GL_DYNAMIC_COPY);

    // Connect SSBO to shader program
    // ssbo_block_index = glGetProgramResourceIndex(projshader.get(), GL_SHADER_STORAGE_BLOCK, "PixelLocs");
    // if (ssbo_block_index == GL_INVALID_INDEX) {
    //     printf("Shader log could not be connected to the shader.\n");
    // }
    // // glBindBufferBase(GL_SHADER_STORAGE_BUFFER, ssbo_block_index, ssbo_binding_point_index);
    // glShaderStorageBlockBinding(projshader.get(), ssbo_block_index, ssbo_binding_point_index);
    // printf("%u %u\n", ssbo_block_index, ssbo_binding_point_index);

    // Create image texture for storing back projection
    // GLuint backProjText;
    // glGenTextures(1, &backProjText);
    // glBindTexture(GL_TEXTURE_2D, backProjText);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);        
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);    
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA32F, mWidth, mHeight);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, mWidth, mHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
    // glBindTexture(GL_TEXTURE_2D, 0);
    // glBindImageTexture(5, backProjText, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);


    // // Pixel Buffer Object
    // GLuint pbo_unpack;
    // glGenBuffers(1, &pbo_unpack);
    // glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_unpack);
    // GLuint pbo_pack;
    // glGenBuffers(1, &pbo_pack);
    // glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_pack);
    // glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);


    GLint * pixelArray = new GLint[mHeight * mWidth * 4];
    GLint * pixelArray2 = new GLint[mHeight * mWidth * 4];
   
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
        floorMesh.draw(shader.get());
        modelPoints.draw(shader.get());
        

        // 2nd pass backprojection
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
        glViewport(0, 0, mWidth, mHeight);
 

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
        
        projshader.activate();
        // projshader.bind("persMatrix", camera.getProjectionMatrix());
        projshader.bind("invViewMatrix", camera.getInverseViewMatrix());
        // projshader.bind("invPersMatrix", camera.getInverseProjectionMatrix());

        glBindVertexArray(quadVAO);
        glBindTexture(GL_TEXTURE_2D, texDepthBuffer);	// use the depth attachment texture as the texture of the quad plane
        // glBindImageTexture(5, backProjText, 0, GL_FALSE, 0,  GL_READ_ONLY, GL_RGBA32F);

        // glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);

        glDrawArrays(GL_TRIANGLES, 0, 6);

        // glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
        // glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
        // glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);

        glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_INT, pixelArray);

        for (int i = 0; i < 4; i++) {
            // printf("%f ", pixelIndex(pixelArray, mWidth/2, mHeight/2, i));        
            printf("%i ", pixelArray[i]);
        }
        
        printf("|");

        // glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_pack);
        // glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_FLOAT, pixelArray);
        // glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

        // for (int i = 0; i < 4; i++) {
        //     printf("%f ", pixelIndex(pixelArray, mWidth/2, mHeight/2, i));        
        //     // printf("%f ", pixelArray[i]);
        // }
        
        // printf("|");

        // // And then copy data to CPU memory
        // glBindBuffer( GL_PIXEL_PACK_BUFFER, pbo_pack );
        // GLfloat* ptr = (GLfloat*) glMapBuffer( GL_PIXEL_PACK_BUFFER, GL_READ_ONLY );
        // if (ptr == NULL) {
        //     printf("AKJLSGDLIASGIDGIA");
        // } else {
        //     memcpy( pixelArray, ptr, mWidth * mHeight * 4 * sizeof(GLfloat) );
        // }
        // glUnmapBuffer( GL_PIXEL_PACK_BUFFER );
        // glBindBuffer( GL_PIXEL_PACK_BUFFER, 0 );

        // for (int i = 0; i < 4; i++) {
        //     // printf("%f ", pixelArray[i]);
        //     printf("%f ", pixelIndex(pixelArray, mWidth/2, mHeight/2, i));        
        // }
        
        // printf("|");

        glBindTexture(GL_TEXTURE_2D, projtexColorBuffer);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        glGetTexImage(GL_TEXTURE_2D, 0,  GL_RGBA_INTEGER, GL_INT, pixelArray2);
        // glGetTextureImage(backProjText, 0, GL_RGBA, GL_FLOAT, mHeight * mWidth * 4, pixelArray);
        for (int i = 0; i < 4; i++) {
            // printf("%i ", pixelArray2[i]/1000);
            printf("%i ", pixelIndex(pixelArray2, mWidth/2, mHeight/2, i));        
        }




        // glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // Ensure ssbo writes are complete (waits until completion before continuing)
        // // Do SSBO Read here
        
        // GLvoid* mapped_ssbo = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
        // if (mapped_ssbo == NULL) {
        //     printf("Could not map shader log into client's memory ");
        // } else {
        //     memcpy(shader_data.get(), mapped_ssbo, sizeof(shader_data_t));
        // }
        // glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        // printf("|");
        // printf("%f ", shader_data->lol);
        // glm::vec4 t = shader_data->pixelLocs[int(mWidth/2)][int(mHeight/2)];
        // glm::vec4 t = shader_data->pixelLocs[0][0];
        // for (int i = 0; i < 4; i++) {
        //     printf("%f ", t[i]);
            // printf("%f ", pixelIndex((GLfloat *) &shader_data->pixelLocs, mWidth/2, mHeight/2, i));        
            // printf("%f ", pixelIndex((GLfloat *) mapped_ssbo, mWidth/2, mHeight/2, i));        
        //     printf("%f ", pixelIndex((GLfloat *) mapped_ssbo, 0, 0, i));        
        // }

        // glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(shader_data_t), shader_data.get());

        // printf("|");
        // printf("%f ", shader_data->lol);

        // glm::vec4 r = shader_data->pixelLocs[0][0];
        // for (int i = 0; i < 4; i++) {
        //     printf("%f ", r[i]);
        //     // printf("%f", shader_data->pixelLocs[int(mWidth/2)][int(mHeight/2)][i]);
        //     // printf("%f ", pixelIndex((GLfloat *) shader_data->pixelLocs, mWidth/2, mHeight/2, i));        
        //     // printf("%f ", pixelIndex((GLfloat *) mapped_ssbo, 0, 0, i));        
        // }

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        
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
        glBindTexture(GL_TEXTURE_2D, texColorBuffer);	// use the depth attachment texture as the texture of the quad plane
        // glBindTexture(GL_TEXTURE_2D, texDepthBuffer);	// use the depth attachment texture as the texture of the quad plane
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

GLint pixelIndex(GLint *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}