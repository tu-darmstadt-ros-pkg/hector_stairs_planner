# hector_sbpl_stairs_detection/refindePath
# This message represents a ar segemnt of a path, and the flipper positions at the end

#path segement
nav_msgs/Path segment
#flipper angle front
float32 flipperFront
#flipper angle rear
float32 flipperRear
#flipper angle has to be interpolated to the robots position
bool interpolate
