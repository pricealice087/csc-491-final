from localization_utils import motion_update, measurement_update

pose = (0, 0, 0)

# simulate movement
pose = motion_update(pose, (1.0, 0.0, 0.1))

# fake landmarks + distances
landmarks = [(5,5), (2,3)]
measurements = [6.5, 3.0]

pose = measurement_update(pose, landmarks, measurements)

print("Estimated pose:", pose)