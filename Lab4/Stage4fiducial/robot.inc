define cleanerbot position
(
  size [0.2 0.24 0.2]                               # [x, y, z] in metres
  origin [0 0 0 0]                                  # centre of rotation
  drive "diff"                                      # Differential steering model.
  localisation "gps"                                # return ground truth pose
  velocity_bounds [ -5 5 0 0 0 0 -90.0 90.0 ]       # Kinematic constraint [ xmin xmax ymin ymax zmin zmax amin amax ]  
  acceleration_bounds [ -10 10 0 0 0 0 -90.0 90.0 ] # Kinematic constraint [ xmin xmax ymin ymax zmin zmax amin amax ]   
  mass 20.0                                         # Mass in KG

  obstacle_return 0                                 # Can hit things.
  ranger_return 1                                   # reflects sonar beams
  fiducial_return 0                                 # Seen by fiducial sensors (0=not, else id)

                                                    # body shape defined as 2.5D object
  block                                       
    (
      points 16
      point[15] [0.7071 0.7071]
      point[14] [0.4540 0.8910]
      point[13] [0.1564 0.9877]
      point[12] [-0.1564 0.9877]
      point[11] [-0.4540 0.8910]
      point[10] [-0.7071 0.7071]
      point[9] [-0.8910 0.4540]
      point[8] [-0.9877 0.1564]
      point[7] [-0.9877 -0.1564]
      point[6] [-0.8910 -0.4540]
      point[5] [-0.7071 -0.7071]
      point[4] [-0.4540 -0.8910]
      point[3] [-0.1564 -0.9877]
      point[2] [0.1564 -0.9877]
      point[1] [0.4540 -0.8910]
      point[0] [0.7071 -0.7071]
      z [0 0.2]
    )
)
