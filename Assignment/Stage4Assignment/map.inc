define floorplan model
(
	
	color "gray30"		# sombre, sensible, artistic
	boundary 1			# most maps will need a bounding box
	gui_nose 1
	gui_grid 1
	gui_move 0
	gui_outline 0
	fiducial_return 0
	gripper_return 0
	ranger_return 1
)

define landmark model
(
 size [0.1 0.1 0.1]
 ranger_return 1             	# reflects rangers
 fiducial_return 1           	# Default seen as fiducial id#1

 block
  (
	points 4
	point[0] [1 0]
	point[1] [1 1]
	point[2] [0 1]
	point[3] [0 0]
	z [0 1]
   )
 color "DarkBlue"
)
