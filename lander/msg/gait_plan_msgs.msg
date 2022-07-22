# 客户端请求
# 0:getpos; 1:init; 2:planfoot; 3:planmotion; -1:reset
int32 command_index
# For planfoot
int32 leg_index
float64[3] foot1_motion
float64[3] foot2_motion
float64[3] foot3_motion
float64[3] foot4_motion
# For planmotion
int32 data_num
float64[] foot1_trace_x
float64[] foot1_trace_y
float64[] foot1_trace_z
float64[] foot2_trace_x
float64[] foot2_trace_y
float64[] foot2_trace_z
float64[] foot3_trace_x
float64[] foot3_trace_y
float64[] foot3_trace_z
float64[] foot4_trace_x
float64[] foot4_trace_y
float64[] foot4_trace_z 