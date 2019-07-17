#include "controls.hpp"

#include <cstdio>

namespace AVSCPP {

	void CameraControl::toggleControl(){
		mouseControl = !mouseControl;
		std::printf("Mouse control toggled to %u\n", mouseControl);
	}

	void CameraControl::setViewMatrix(glm::mat4 vm) {
		ViewMatrix = vm;
		mouseControl = false;
	}

	void CameraControl::resetView() {
		position = glm::vec3(0, 0, 5);
		lookat = glm::vec3(0.0, 0.0, 0.0);
		up = glm::vec3(1, 0, 0);
		mouseControl = false;
	}

	void CameraControl::computeMatricesFromInputs(){


		// glfwGetTime is called only once, the first time this function is called
		static double lastTime = glfwGetTime();

		// Compute time difference between current and last frame
		double currentTime = glfwGetTime();
		float deltaTime = float(currentTime - lastTime);

		if (mouseControl) {

			// Get mouse position
			glfwGetCursorPos(window, &xpos, &ypos);

			// Reset mouse position for next frame
			glfwSetCursorPos(window, (int)width/2, (int)height/2);

			// Compute new orientation
			horizontalAngle += mouseSpeed * float(width/2 - xpos );
			verticalAngle   += mouseSpeed * float(height/2 - ypos );

			// Direction : Spherical coordinates to Cartesian coordinates conversion
			direction = glm::vec3(
				cos(verticalAngle) * sin(horizontalAngle), 
				sin(verticalAngle),
				cos(verticalAngle) * cos(horizontalAngle)
			);
			
			// Right vector
			right = glm::vec3(
				sin(horizontalAngle - 3.14f/2.0f), 
				0,
				cos(horizontalAngle - 3.14f/2.0f)
			);

			// Move forward
			if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
				position += direction * deltaTime * speed;
			}
			// Move backward
			if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
				position -= direction * deltaTime * speed;
			}
			// Strafe right
			if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
				position += right * deltaTime * speed;
			}
			// Strafe left
			if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
				position -= right * deltaTime * speed;
			}

			// Up vector
			up = glm::cross( right, direction );
			lookat = position + direction;
		}

		float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.
		
		// Projection matrix : 45� Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
		ProjectionMatrix = glm::perspective(glm::radians(FoV), aspect, displayRange[0], displayRange[1]);

		// Camera matrix
		ViewMatrix       = glm::lookAt(
									position,           // Camera is here
									lookat, 			// and looks here : at the same position, plus "direction"
									up                  // Head is up (set to 0,-1,0 to look upside-down)
							);
		
		// For the next frame, the "last time" will be "now"
		lastTime = currentTime;
	}
}