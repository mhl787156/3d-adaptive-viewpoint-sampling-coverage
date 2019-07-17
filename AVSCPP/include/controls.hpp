#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <GLFW/glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace AVSCPP {

    class CameraControl
    {
        public:
            // Constructor and Destructor
            CameraControl(GLFWwindow *_window) {
                window = _window;
                glfwGetWindowSize(_window, &width, &height);
                aspect = (float) width / (float) height;
            }
            void computeMatricesFromInputs();
            glm::mat4 getViewMatrix() {return ViewMatrix;}
            glm::mat4 getInverseViewMatrix() {return glm::inverse(ViewMatrix);}
            glm::mat4 getProjectionMatrix() {return ProjectionMatrix;}
            glm::mat4 getInverseProjectionMatrix() {return glm::inverse(ProjectionMatrix);}
            glm::mat4 getVPMatrix() {return ProjectionMatrix * ViewMatrix;}
            glm::mat4 getInverseVPMatrix() {return glm::inverse(ProjectionMatrix * ViewMatrix);}
            void toggleControl();
            void setViewMatrix(glm::mat4 vm);
            void resetView();
            glm::vec3 getPosition() {return position;}
            float getAspect() {return aspect;}
            float getfov() {return initialFoV;}
            glm::vec2 getDisplayRange(){return displayRange;}

        private:

            GLFWwindow *window;
            int width;
            int height;

            glm::mat4 ViewMatrix;
            glm::mat4 ProjectionMatrix;
            bool mouseControl = true;
            // Initial position : on +Z
            glm::vec3 position = glm::vec3( 5, 5, 5); 
            glm::vec3 direction = glm::vec3(-5, -5, -5);
            glm::vec3 up = glm::vec3(1, 0, 0);
            glm::vec3 right = glm::vec3(0.0);
            glm::vec3 lookat = glm::vec3(0.0);

            glm::vec2 displayRange = glm::vec2(0.1, 100.0);

            // Initial horizontal angle : toward -Z
            float horizontalAngle = 3.14f;
            // Initial vertical angle : none
            float verticalAngle = 0.0f;
            // Initial Field of View
            float initialFoV = 45.0f;
            // Aspect Ratio
            float aspect = 4.0f / 3.0f;

            double xpos, ypos;

            float speed = 10.0f; // 3 units / second
            float mouseSpeed = 0.005f;
    };
    

}


#endif