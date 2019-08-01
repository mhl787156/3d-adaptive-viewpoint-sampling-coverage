#include "render.hpp"

#include <glm/gtx/euler_angles.hpp>

using namespace AVSCPP;

Renderer::Renderer(GLint _mWidth, GLint _mHeight) {

    mWidth = _mWidth;
    mHeight = _mHeight;
    numPixels = mWidth * mHeight;

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

    pixelArray = new GLint[numPixels * 4];
    floatPixelArray = new GLfloat[numPixels * 4];
}

Renderer::~Renderer() {

    glDeleteVertexArrays(1, &VertexArrayObject);
    glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &quadVBO);

    free(pixelArray);
    free(floatPixelArray);
}

void Renderer::initVertexArraysandBuffers() {
    // Initialise VAO
	glGenVertexArrays(1, &VertexArrayObject);
	glBindVertexArray(VertexArrayObject);

    // VBO for drawing collected points
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, numPixels * 4 * sizeof(GLint), NULL, GL_DYNAMIC_DRAW);

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

void Renderer::setShaders(AVSCPP::Shader *_normalShader,
                        AVSCPP::Shader *_backprojectionShader,
                        AVSCPP::Shader *_integerdisplayShader,
                        AVSCPP::CameraControl &camera) {
    normalShader = _normalShader;
    backprojectionShader = _backprojectionShader;
    integerdisplayShader = _integerdisplayShader;

    backprojectionShader->activate();
    float half_y_near_plane = glm::tan(camera.getfov() / 2.0);
    glm::vec2 hsnp = glm::vec2(half_y_near_plane , half_y_near_plane / camera.getAspect()); //  / camera.getAspect();
    backprojectionShader->bind("halfSizeNearPlane", hsnp);
    backprojectionShader->bind("scaleFactor", positionShaderScaler);
    backprojectionShader->bind("cameraNearFarPlane", camera.getDisplayRange());

    GLuint t1Location = glGetUniformLocation(backprojectionShader->get(), "texDepthWithCull");
    GLuint t2Location = glGetUniformLocation(backprojectionShader->get(), "texDepthNoCull");
    glUniform1i(t1Location, 0);
    glUniform1i(t2Location, 1);
}


void Renderer::initFrameBuffers() {
    // Create frame buffer for with cull
    glGenFramebuffers(1, &framebuffer1a);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1a);    
    glGenTextures(1, &texDepthBuffer);
    glBindTexture(GL_TEXTURE_2D, texDepthBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepthBuffer, 0);
    // Check framebuffer is correctly set up then unbind the framebuffer
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	    printf("ERROR::FRAMEBUFFER:: Framebuffer1 is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  

    // Create frame buffer for no cull
    glGenFramebuffers(1, &framebuffer1b);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1b);  
    glGenTextures(1, &texDepthBufferNoCull);
    glBindTexture(GL_TEXTURE_2D, texDepthBufferNoCull);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0); 
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepthBufferNoCull, 0);
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

GLfloat* Renderer::getRenderedPositions(AVSCPP::CameraControl &camera, std::vector<AVSCPP::Mesh*> meshes) {

    if(!renderToScreen) {
        glfwHideWindow(mWindow);
    }

    // per-frame time logic
    // --------------------
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if(debug) {
        printf("%f (%.2f fps)| Camera Loc (xyz): ", currentFrame, 1/deltaTime);

        glm::vec3 camPos = camera.getPosition();
        for (int i = 0; i < 3; i++) {
            printf("%f ", camPos[i]);
        }
    }

    // input
    // -----
    processInput(camera);
    // Compute the MVP matrix from keyboard and mouse input
    camera.computeMatricesFromInputs();

    // First pass into of depth into framebuffer with original shaders and face culling
    // ------
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1a);
    glViewport(0, 0, mWidth, mHeight);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); // we're not using the stencil buffer now
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // First pass with cull

    // Activate Shader
    normalShader->activate(); // glUseProgram
    normalShader->bind("in_colour", glm::vec4(0.0, 0.0, 0.0, 1.0));
    
    // Draw Meshes
    for(AVSCPP::Mesh* m: meshes){
        glm::mat4 MVP = camera.getVPMatrix() * m->getModelMatrix();
        normalShader->bind("MVP", MVP);
        m->draw(normalShader->get());
    }

    // Second pass of depth into framebuffer with original shaders and no face culling
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer1b);
    glViewport(0, 0, mWidth, mHeight);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); // we're not using the stencil buffer now
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); // Second pass without cull

    // Activate Shader
    normalShader->activate(); // Write into alpha channel as negative on backface

    // Draw Mesh 2nd time
    for(AVSCPP::Mesh* m: meshes){
        glm::mat4 MVP = camera.getVPMatrix() * m->getModelMatrix();
        normalShader->bind("MVP", MVP);
        normalShader->bind("in_colour", glm::vec4(0.0, 0.0, 0.0, 1.0));
        m->draw(normalShader->get());
    }

    // 3rd pass backprojection
    // -----------
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer2);
    glViewport(0, 0, mWidth, mHeight);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
    
    backprojectionShader->activate();
    backprojectionShader->bind("invViewMatrix", camera.getInverseViewMatrix());

    glBindVertexArray(quadVAO);
    glActiveTexture(GL_TEXTURE0 ); // Texture unit 0
    glBindTexture(GL_TEXTURE_2D, texDepthBuffer); // use the depth attachment texture as the texture of the quad plane
    glActiveTexture(GL_TEXTURE0+1 ); // Texture unit 1
    glBindTexture(GL_TEXTURE_2D, texDepthBufferNoCull);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindTexture(GL_TEXTURE_2D, projtexColorBuffer);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glGetTexImage(GL_TEXTURE_2D, 0,  GL_RGBA_INTEGER, GL_INT, pixelArray);

    if(debug) {
        printf("| Center Pixel (xyzd): ");
        for (int i = 0; i < 4; i++) {
            GLfloat v = (float) pixelIndex(pixelArray, mWidth/2, mHeight/2, i);
            printf("%f ", v/positionShaderScaler);        
        }
    }

    if (glfwGetKey(mWindow, GLFW_KEY_Z) != GLFW_PRESS) {
        glBindVertexArray(VertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,  numPixels * 4 * sizeof(GLint), pixelArray);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    if(renderToScreen) {
        // third pass (render screen)
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // back to default framebuffer
        glViewport(0, 0, mWidth, mHeight);
        glClearColor(0.0, 0.0, 0.5, 0.0); 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        if (glfwGetKey(mWindow, GLFW_KEY_X) != GLFW_PRESS) {
            normalShader->activate();
            // Draw Meshes
            for(AVSCPP::Mesh* m: meshes){
                glm::mat4 MVP = camera.getVPMatrix() * m->getModelMatrix();
                normalShader->bind("MVP", MVP);
                normalShader->bind("in_colour", glm::vec4(1.0, 0.0, 0.0, 1.0));
                m->draw(normalShader->get());
            } 
        }

        integerdisplayShader->activate();
        integerdisplayShader->bind("MVP", camera.getVPMatrix());
        integerdisplayShader->bind("scale", positionShaderScaler);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glBindVertexArray(VertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_INT, GL_FALSE, 0, (void*)0);
        glEnableVertexAttribArray(0);  
        glDrawArrays(GL_POINTS, 0, numPixels);

        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    if(debug) {
        printf("\n");
    }
    
    for(int i = 0; i < numPixels * 4; i++) {
        floatPixelArray[i] = float(pixelArray[i]) / positionShaderScaler;
    }

    // Note depth value of pixelArray may return negative if backface.
    return floatPixelArray;
}


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void Renderer::processInput(AVSCPP::CameraControl &c)
{
    if (glfwGetKey(mWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(mWindow, true);

    if (glfwGetKey(mWindow, GLFW_KEY_SPACE) == GLFW_PRESS)
        c.resetView();

    if (glfwGetKey(mWindow, GLFW_KEY_C) == GLFW_PRESS)
        c.toggleControl();
}

GLfloat Renderer::pixelIndex(GLfloat *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}

GLint Renderer::pixelIndex(GLint *b, int w, int h, int c) {
    return b[int(w) * 4 + int(h) * mWidth * 4 + c];
}

void Renderer::displayViewpoints(AVSCPP::CameraControl &camera, 
                                 std::vector<glm::mat4> &viewpoints,
                                 std::vector<GLint>& trajectory,
                                 std::vector<glm::vec3> &seenPoints,
                                 std::vector<AVSCPP::Mesh*> meshes) {
    
    glfwShowWindow(mWindow);
    glfwFocusWindow(mWindow);
    camera.enableControl();    

    float axisLength = camera.getDisplayRange()[1];
    std::vector<glm::vec3> axis;
    axis.push_back(glm::vec3(0.0, 0.0, 0.0));
    axis.push_back(glm::vec3(axisLength, 0.0, 0.0));
    axis.push_back(glm::vec3(0.0, 0.0, 0.0));
    axis.push_back(glm::vec3(0.0, axisLength, 0.0));
    axis.push_back(glm::vec3(0.0, 0.0, 0.0));
    axis.push_back(glm::vec3(0.0, 0.0, axisLength));

    std::vector<glm::vec3> vps;
    std::vector<glm::vec3> lines;
    std::vector<glm::vec3> xylines;
    for(glm::mat4 vp: viewpoints) {
        glm::vec3 cameraLoc = glm::vec3(vp[3]);

        float yaw = atan2(-vp[0][2], sqrt( pow(vp[1][2], 2.0) + pow(vp[2][2], 2.0) ) );

        glm::vec4 unitVectorLoc = vp * glm::vec4(0.0, 0.0, 0.5, 1.0);
        lines.push_back(cameraLoc);
        lines.push_back(glm::vec3(unitVectorLoc));

        unitVectorLoc = vp * glm::vec4(0.0, 0.5, 0.0, 1.0);
        xylines.push_back(cameraLoc);
        xylines.push_back(glm::vec3(unitVectorLoc));

        unitVectorLoc = vp * glm::vec4(0.5, 0.0, 0.0, 1.0);
        xylines.push_back(cameraLoc);
        xylines.push_back(glm::vec3(unitVectorLoc));

        vps.push_back(glm::vec3(vp[3]));
    }

    std::vector<glm::vec3> trajpoints;
    for(GLint traj_idx: trajectory) {
        trajpoints.push_back(vps[traj_idx-1]);
    }

    // Green Z lines for each viewpoint
    GLuint vpVAO1, vpVAO2, vpVBO1, vpVBO2, axisVBO;
    glGenVertexArrays(1, &vpVAO1);
    glBindVertexArray(vpVAO1);
    glGenBuffers(1, &vpVBO1);
    glBindBuffer(GL_ARRAY_BUFFER, vpVBO1);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * lines.size(), &lines[0], GL_STATIC_DRAW);

    // Viewpoint X, Y axis
    glGenVertexArrays(1, &vpVAO2);
    glBindVertexArray(vpVAO2);
    glGenBuffers(1, &vpVBO2);
    glBindBuffer(GL_ARRAY_BUFFER, vpVBO2);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * xylines.size(), &xylines[0], GL_STATIC_DRAW);

    // Main Axis x, y, z
    glBindVertexArray(axisVBO);
    glGenBuffers(1, &axisVBO);
    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * axis.size(), &axis[0], GL_STATIC_DRAW);

    // Seen Points
    GLuint spVAO, spVBO;
    glGenVertexArrays(1, &spVAO);
    glBindVertexArray(spVAO);
    glGenBuffers(1, &spVBO);
    glBindBuffer(GL_ARRAY_BUFFER, spVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * seenPoints.size(), &seenPoints[0], GL_STATIC_DRAW);

    // Trajectory Line Strip
    GLuint tVAO, tVBO;
    glGenVertexArrays(1, &tVAO);
    glBindVertexArray(tVAO);
    glGenBuffers(1, &tVBO);
    glBindBuffer(GL_ARRAY_BUFFER, tVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * trajpoints.size(), &trajpoints[0], GL_STATIC_DRAW);

    glClearColor(0.0, 0.0, 0.5, 0.0); 
    
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_CULL_FACE); 

    while(canRender()){
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(camera);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

        // Compute the MVP matrix from keyboard and mouse input
        camera.computeMatricesFromInputs();

        normalShader->activate(); // glUseProgram

        glm::mat4 VP = camera.getVPMatrix();

        // Draw Meshes
        normalShader->bind("in_colour", glm::vec4(1.0, 0.0, 0.0, 1.0));
        for(AVSCPP::Mesh* m: meshes){
            glm::mat4 MVP = camera.getVPMatrix() * m->getModelMatrix();
            normalShader->bind("MVP", MVP);
            if (glfwGetKey(mWindow, GLFW_KEY_X) != GLFW_PRESS) {
                m->draw(normalShader->get());
            }
        } 

        // Draw Seen Points
        normalShader->bind("MVP", VP);
        normalShader->bind("in_colour", glm::vec4(1.0, 1.0, 0.0, 1.0));
        glBindVertexArray(spVAO);
        glBindBuffer(GL_ARRAY_BUFFER, spVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_POINTS, 0, seenPoints.size());

        // Draw trajectory lines
        normalShader->bind("MVP", VP);
        normalShader->bind("in_colour", glm::vec4(1.0, 0.5, 1.0, 1.0));
        glBindVertexArray(tVAO);
        glBindBuffer(GL_ARRAY_BUFFER, tVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINE_STRIP, 0, trajpoints.size());

        // Draw viewpoint axis
        normalShader->bind("MVP", VP);
        normalShader->bind("in_colour", glm::vec4(0.0, 1.0, 0.0, 1.0));
        glLineWidth(10.0);
        glBindVertexArray(vpVAO1);
        glBindBuffer(GL_ARRAY_BUFFER, vpVBO1);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINES, 0, lines.size());

        normalShader->bind("in_colour", glm::vec4(0.5, 0.5, 0.5, 1.0));
        glLineWidth(1.0);
        glBindVertexArray(vpVAO2);
        glBindBuffer(GL_ARRAY_BUFFER, vpVBO2);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINES, 0, xylines.size());

        glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glDrawArrays(GL_LINES, 0, axis.size());

        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &vpVAO1);
    glDeleteVertexArrays(1, &vpVAO2);
    glDeleteBuffers(1, &vpVBO1);
    glDeleteBuffers(1, &vpVBO2);
}