# the pose from which to grasp the object prior to translation
geometry_msgs/PoseStamped pre_grasp_pose

# the pose from which to grasp the object after translation
geometry_msgs/PoseStamped grasp_pose

# estimated probability of success for this grasp
float64 grasp_quality

# grasp_type = 0: wrap grasp with Preshape = 0; Horizontal; example: mug #1 
# grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 
# grasp_type = 2: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball
# grasp type 
uint8 grasp_type

# the id of the possible grasp poses
string grasp_id

# the id of the grasped object
string object_id