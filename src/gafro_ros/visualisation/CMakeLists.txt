target_sources(${PROJECT_NAME}
	PRIVATE 
		${CMAKE_CURRENT_LIST_DIR}/VectorField.cpp

		${CMAKE_CURRENT_LIST_DIR}/Point.cpp
		${CMAKE_CURRENT_LIST_DIR}/PointVector.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/Line.cpp
		${CMAKE_CURRENT_LIST_DIR}/LineVector.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/Plane.cpp
		${CMAKE_CURRENT_LIST_DIR}/PlaneVector.cpp

		${CMAKE_CURRENT_LIST_DIR}/Circle.cpp
		${CMAKE_CURRENT_LIST_DIR}/CircleVector.cpp

		${CMAKE_CURRENT_LIST_DIR}/PointPair.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
		${CMAKE_CURRENT_LIST_DIR}/SphereVector.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/Cube.cpp
		${CMAKE_CURRENT_LIST_DIR}/Ellipse.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/Motor.cpp
		${CMAKE_CURRENT_LIST_DIR}/MotorVector.cpp

		${CMAKE_CURRENT_LIST_DIR}/Twist.cpp
		${CMAKE_CURRENT_LIST_DIR}/Wrench.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/SubscriberTwist.cpp
		${CMAKE_CURRENT_LIST_DIR}/SubscriberWrench.cpp
		
		${CMAKE_CURRENT_LIST_DIR}/System.cpp
		)
