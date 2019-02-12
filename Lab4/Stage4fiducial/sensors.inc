# A multi-point range finder with configurable range and field of view
define topurg ranger
(
	sensor
	  ( 			
	    range [ 0.0  2.0 ]
	    fov 30
	    samples 1
	  )
  color "black"				# generic model properties
  size [ 0.05 0.05 0.1 ]
)

# A camera with configurable resolution, field of view and pan tilt angle
define mycamera camera
(
	range [ 0.2 5.0 ]
	resolution [ 1000 1000 ]
	fov [ 120 120 ]
	pantilt [ 0 0 ]
	alwayson 1
	color "red"
  	size [ 0.05 0.05 0.05 ]
  	obstacle_return 1                                 # Can hit things.
)

define feature_detector fiducial
(
  # fiducial properties
  range_min 0.0				# minimum range of sensor
  range_max 2.0				# maximum range of sensor
  range_max_id 2.0			# max range that fiducial ID detected
  fov 180					# field of view in degrees
  ignore_zloc 1				# ignore the z component of fiducial

  # model properties
  size [ 0.1 0.1 0.1 ]		# in metres
)