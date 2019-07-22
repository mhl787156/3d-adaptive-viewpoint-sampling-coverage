#include "render.hpp"

using namespace AVSCPP;

Renderer::Renderer(GLint _mWidth, GLint _mHeight) {

    mWidth = _mWidth;
    mHeight = _mHeight;

     // Load GLFW and Create a Window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    mWindow = glfwCreateWindow(mWidth, mHeight, "OpenGL", nullptr, nullptr);

    // Check for Valid Context
    if (mWindow == nullptr) {
        fprintf(stderr, "Failed to Create OpenGL Context");
    }

    // Create Context and Load OpenGL Functions
    glfwMakeContextCurrent(mWindow);
    gladLoadGL();
    fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

    pixelArray = new GLint[mHeight * mWidth * 4];
}

Renderer::~Renderer() {

    glDeleteVertexArrays(1, &VertexArrayObject);
    glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &quadVBO);

    free(pixelArray);
}

void Renderer::initVertexArraysandBuffers() {
    // Initialise VAO
	glGenVertexArrays(1, &VertexArrayObject);
	glBindVertexArray(VertexArrayObject);

    // VBO for drawing collected points
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, mWidth * mHeight * 4 * sizeof(GLint), NULL, GL_DYNAMIC_DRAW);

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
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
}

void Renderer::setShaders(AVSCPP::Shader &_normalShader,
                        AVSCPP::Shader &_backprojectionShader,
                        AVSCPP::Shader &_integerdisplayShader,
                        AVSCPP::CameraControl *camera) {
    normalShader = &_normalShader;
    backprojectionShader = &_backprojectionShader;
    integerdisplayShader = &_integerdisplayShader;
    
    backprojectionShader->activate();
    float half_y_near_plane = glm::tan(camera->getfov() / 2.0);
    glm::vec2 hsnp = glm::vec2(half_y_near_plane , half_y_near_plane / camera->getAspect()); //  / camera.getAspect();
    backprojectionShader->bind("halfSizeNearPlane", hsnp);
    backprojectionShader->bind("screenTexture", 0);
    backprojectionShader->bind("scaleFactor", positionShaderScaler);
    backprojectionShader->bind("cameraNearFarPlane", camera->getDisplayRange());
}


void Renderer::initFrameBuffers() {
    // Create frame buffer
    glGenFramebuffers(1, &framebuffer1);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1);    

    // generate texture
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mWidth, mHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenTextures(1, &texDepthBuffer);
    glBindTexture(GL_TEXTURE_2D, texDepthBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    // attach it to currently bound framebuffer object
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepthBuffer, 0);

    // Check framebuffer is correctly set up then unbind the framebuffer
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer1 is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  

    // Create another framebuffer for back projection
    glGenFramebuffers(1, &framebuffer2);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
    glGenTextures(1, &projtexColorBuffer);
    glBindTexture(GL_TEXTURE_2D, projtexColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32I, mWidth, mHeight, 0, GL_RGBA_INTEGER, GL_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, projtexColorBuffer, 0);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer2 is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  
}

int* Renderer::getRenderedPositions(AVSCPP::CameraControl* camera, std::vector<AVSCPP::Mesh*> meshes) {

    if(!renderToScreen) {
        glfwHideWindow(mWindow);
    }

    // per-frame time logic
    // --------------------
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    printf("%f: ", currentFrame);

    glm::vec3 camPos = camera->getPosition();
    for (int i = 0; i < 3; i++) {
        printf("%f ", camPos[i]);
    }

    // input
    // -----
    processInput(mWindow, camera);

    // First pass into framebuffer with original shaders
    // ------
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1);
    glViewport(0, 0, mWidth, mHeight);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Activate Shader
    normalShader->activate(); // glUseProgram

    // Compute the MVP matrix from keyboard and mouse input
    camera->computeMatricesFromInputs();
    glm::mat4 ModelMatrix = glm::mat4(1.0);
    glm::mat4 MVP = camera->getVPMatrix() * ModelMatrix;
    normalShader->bind("MVP", MVP);

    // Draw Meshes
    for(AVSCPP::Mesh* m: meshes){
        m->draw(normalShader->get());
    } 
    
    // 2nd pass backprojection
    // -----------
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
    glViewport(0, 0, mWidth, mHeight);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
    
    backprojectionShader->activate();
    backprojectionShader->bind("invViewMatrix", camera->getInverseViewMatrix());

    glBindVertexArray(quadVAO);
    glBindTexture(GL_TEXTURE_2D, texDepthBuffer);	// use the depth attachment texture as the texture of the quad plane
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindTexture(GL_TEXTURE_2D, projtexColorBuffer);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glGetTexImage(GL_TEXTURE_2D, 0,  GL_RGBA_INTEGER, GL_INT, pixelArray);
    
    GLfloat k = (float) pixelIndex(pixelArray, mWidth/2, mHeight/2, 3);
    for (int i = 0; i < 3; i++) {
        GLfloat v = (float) pixelIndex(pixelArray, mWidth/2, mHeight/2, i);
        printf("%f ", v/k);        
    }

    glBindVertexArray(VertexArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,  mWidth * mHeight * 4 * sizeof(GLint), pixelArray);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if(renderToScreen) {
        // third pass (render screen)
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // back to default framebuffer
        glViewport(0, 0, mWidth, mHeight);
        glClearColor(0.0, 0.0, 0.5, 0.0); 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_CULL_FACE);

        if (glfwGetKey(mWindow, GLFW_KEY_D) != GLFW_PRESS) {
            normalShader->activate(); // glUseProgram
            normalShader->bind("MVP", MVP);
                        // Draw Meshes
            for(AVSCPP::Mesh* m: meshes){
                m->draw(normalShader->get());
            } 
        }

        integerdisplayShader->activate();
        integerdisplayShader->bind("MVP", MVP);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glBindVertexArray(VertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_INT, GL_FALSE, 0, (void*)0);
        glEnableVertexAttribArray(0);  
        
        glDrawArrays(GL_POINTS, 0, mWidth * mHeight);
    }

    printf("\n");

    glfwSwapBuffers(mWindow);
    glfwPollEvents();

    return pixelArray;
}


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window, AVSCPP::CameraControl* c)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        c->resetView();

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        c->toggleControl();
}

GLfloat pixelIndex(GLfloat *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}

GLint pixelIndex(GLint *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}