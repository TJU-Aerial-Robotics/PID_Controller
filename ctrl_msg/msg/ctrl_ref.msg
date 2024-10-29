# enum mask
uint8 POS_CTRL_VALIED = 1
uint8 VEL_CTRL_VALIED = 2
uint8 ACC_CTRL_VALIED = 4

# 
Header     header
float32[3] pos_ref
float32[3] vel_ref
float32[3] acc_ref
float32    yaw_ref
uint8      ref_mask