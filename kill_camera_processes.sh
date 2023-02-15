# kills any processes that may use the camera, so custom applications can be run
ps aux | grep rosnode | awk ' {print $2}' | xargs  kill -9
ps aux | grep live_human_pose | awk ' {print $2}' | xargs  kill -9
ps aux | grep point_cloud_node | awk ' {print $2}' | xargs  kill -9
ps aux | grep mqttControlNode | awk ' {print $2}' | xargs  kill -9