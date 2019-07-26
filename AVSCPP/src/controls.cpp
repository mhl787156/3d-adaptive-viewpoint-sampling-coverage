#include "controls.hpp"

#include <cstdio>
#include <cmath>

namespace AVSCPP {

	void CameraControl::toggleControl(){
		mouseControl = !mouseControl;
		std::printf("Mouse control toggled to %u\n", mouseControl);
	}

	void CameraControl::setViewMatrix(glm::mat4 vm) {
		ViewMatrix = vm;
		mouseControl = false;
	}

	void CameraControl::setViewMatrixFromPoseMatrix(glm::mat4 pose) {

		glm::vec3 pos = pose[3];
		
		glm::vec4 unitmZ = glm::vec4(0.0, 0.0, 1.0, 1.0);
		glm::vec3 direction = glm::vec3(pose * unitmZ - pose[3]);
		direction = glm::normalize(direction);

		glm::vec4 unitX = glm::vec4(-1.0, 0.0, 0.0, 1.0);
		glm::vec3 left = glm::vec3(pose * unitX);

		glm::vec3 up = glm::cross( left, direction );

		setViewMatrix(glm::lookAt(
			pos, pos+direction, glm::vec3(0.0, 1.0, 0.0)
		));
	}

	void CameraControl::setViewMatrixFromPositionAngles(glm::vec3 position, glm::vec3 hvangles) {
            // Calculate lookAt transformation based on position and yaw            
			glm::vec3 direction = glm::vec3(
				cos(hvangles[1]) * sin(hvangles[0]), 
				sin(hvangles[1]),
				cos(hvangles[1]) * cos(hvangles[0])
			);
			
			// Right vector
			glm::vec3 right  = glm::vec3(
				sin(hvangles[2] + hvangles[0] - 3.14f/2.0f), 
				0,
				cos(hvangles[2] + hvangles[0] - 3.14f/2.0f)
			);
            glm::vec3 up = glm::cross( right, direction);
            
            glm::mat4 cameraView = glm::lookAt(
                position,
                position + direction,
                up
            );

			setViewMatrix(cameraView);
	}

	void CameraControl::resetView() {
		position = glm::vec3(10, 10, 10);
		lookat = glm::vec3(0.0, 0.0, 0.0);
		glm::vec3 pl = lookat - position; // position - lookat;
		horizontalAngle = atan2(pl[1], pl[0] );
		verticalAngle = -atan2(pl[2], pl[0] * cos(horizontalAngle));
	}

	void CameraControl::computeMatricesFromInputs(){


		// glfwGetTime is called only once, the first time this function is called
		static double lastTime = glfwGetTime();

		// Compute time difference between current and last frame
		double currentTime = glfwGetTime();
		float deltaTime = float(currentTime - lastTime);

		if (mouseControl) {
			
			if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS){
				verticalAngle -= deltaTime * speed * 0.05;
			}
			// Move backward
			if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS){
				verticalAngle += deltaTime * speed * 0.05;
			}
			// Strafe right
			if (glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS){
				horizontalAngle += deltaTime * speed * 0.05;
			}
			// Strafe left
			if (glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS){
				horizontalAngle -= deltaTime * speed * 0.05;
			}

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

		if(mouseControl) {
			// Camera matrix
			ViewMatrix       = glm::lookAt(
										position,           // Camera is here
										lookat, 			// and looks here : at the same position, plus "direction"
										up                  // Head is up (set to 0,-1,0 to look upside-down)
								);
		}

		// For the next frame, the "last time" will be "now"
		lastTime = currentTime;
	}
}