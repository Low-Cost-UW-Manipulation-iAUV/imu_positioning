#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/src/
rsync -avzh ./include/imu_positioning/*.hpp 	BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/include/imu_positioning/
rsync -avzh CMakeLists.txt 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/
rsync -avzh *.xml 								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/
rsync -avzh ./launch/*.launch	 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/launch/
rsync -avzh *.yaml		 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/
rsync -avzh *.md								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/
rsync -avzh ./urdf/*.urdf						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/urdf/
rsync  -avzh -e ssh ./msg/*.msg					BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/imu_positioning/msg/


echo "All done, Good Success!"