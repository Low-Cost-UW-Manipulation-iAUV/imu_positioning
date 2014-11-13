#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync  -avzh -e ssh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/src/
rsync  -avzh -e ssh ./include/imu_positioning/*.hpp 			eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/include/imu_positioning/
rsync  -avzh -e ssh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/
rsync  -avzh -e ssh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/
rsync  -avzh -e ssh *.launch	 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/
rsync  -avzh -e ssh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/
rsync  -avzh -e ssh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/
rsync  -avzh -e ssh ./urdf/*.urdf						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/urdf/
rsync  -avzh -e ssh ./msg/*.msg						eurathlon_vm:/home/euratlhon/uwesub_msc/src/imu_positioning/msg/


echo "All done, Good Success!"