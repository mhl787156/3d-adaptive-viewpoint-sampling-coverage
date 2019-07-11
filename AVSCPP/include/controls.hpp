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
            CameraControl(GLFWwindow *_window) {window = _window;}
            void computeMatricesFromInputs();
            glm::mat4 getViewMatrix() {return ViewMatrix;}
            glm::mat4 getProjectionMatrix() {return ProjectionMatrix;}
            glm::mat4 getVPMatrix() {return ProjectionMatrix * ViewMatrix;}
            void toggleControl();
            void setViewMatrix(glm::mat4 vm);
            void resetView();

        private:

            GLFWwindow *window;
            glm::mat4 ViewMatrix;
            glm::mat4 ProjectionMatrix;
            bool mouseControl = true;
            // Initial position : on +Z
            glm::vec3 position = glm::vec3( 0, 0, 5 ); 
            // Initial horizontal angle : toward -Z
            float horizontalAngle = 3.14f;
            // Initial vertical angle : none
            float verticalAngle = 0.0f;
            // Initial Field of View
            float initialFoV = 45.0f;

            float speed = 3.0f; // 3 units / second
            float mouseSpeed = 0.005f;
    };
    

}


#endif