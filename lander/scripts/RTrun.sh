tmux new-session \;\
  send-keys 'roslaunch fdilink_ahrs AllImus_driver.launch' C-m \;\
  split-window -h -p 50 \;\
  send-keys 'sleep 2s' C-m \;\
  send-keys 'rosrun lander lander_plan' C-m \;\
  split-window -v -t 0 -p 50 \;\
  send-keys 'sleep 3s' C-m \;\
  send-keys 'rosrun lander muti_receiver.py' C-m \;\
  split-window -v -t 1 -p 50 \;\
  send-keys 'sleep 4s' C-m \;\
  send-keys 'rosrun lander Sub_StateData.py' C-m \;\
  split-window -v -t 3 -p 50 \;\
  send-keys 'sleep 4s' C-m \;\
  send-keys 'rqt_plot /JointStates/position[3]:velocity[3]:effort[3]:position[4]:velocity[4]:effort[4]' C-m \;\
