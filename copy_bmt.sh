#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						bmt:/home/devel/catkin_ws/src/imu_positioning/src/
rsync -avzh ./include/imu_positioning/*.hpp 	bmt:/home/devel/catkin_ws/src/imu_positioning/include/imu_positioning/
rsync -avzh CMakeLists.txt 						bmt:/home/devel/catkin_ws/src/imu_positioning/
rsync -avzh *.xml 								bmt:/home/devel/catkin_ws/src/imu_positioning/
rsync -avzh *.launch	 						bmt:/home/devel/catkin_ws/src/imu_positioning/
rsync -avzh *.yaml		 						bmt:/home/devel/catkin_ws/src/imu_positioning/
rsync -avzh *.md								bmt:/home/devel/catkin_ws/src/imu_positioning/
rsync -avzh ./urdf/*.urdf						bmt:/home/devel/catkin_ws/src/imu_positioning/urdf/
rsync  -avzh -e ssh ./msg/*.msg					bmt:/home/devel/catkin_ws/src/imu_positioning/msg/


echo "All done, Good Success!"