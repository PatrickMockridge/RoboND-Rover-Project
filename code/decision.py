import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                ##############################################################################################
                # Determining where to go:
                # 1) Find the steering angle pointing the direction of the most navigable terrain.
                angle = np.mean(Rover.nav_angles * 180 / np.pi)
                # 2) For angles +/- 45 degrees from the angle defined above, find the steering angle the will
                # lead the robot to point in the world that has been least visited according to the sum of the
                # visit counts in Rover.worldmap_visited.
                visit_count = []
                for delta in np.arange(-45, 45, step=5):
                    # Given a candidate direction, project where the robot will be if that angle is selected.
                    new_x = 70 * np.cos((angle+delta)*np.pi/180.)
                    new_y = 70 * np.cos((angle+delta)*np.pi/180.)
                    new_x, new_y = perception.pix_to_world(
                        new_x, new_y, Rover.pos[0], Rover.pos[1], Rover.yaw,
                        Rover.worldmap.shape[0], 10)
                    # If the new direction lands the robot in an obstacle then we skip it, otherwise we
                    # count the number of times that area has been visited.
                    if not np.all(Rover.worldmap[new_y, new_x] == [255, 0, 0]):
                        count = np.sum(Rover.worldmap_visited[new_y-5:new_y+5, new_x-5:new_x+5].ravel())
                        visit_count.append((delta, count))
                # 3) If we have found more than one possible direction, then we select the one with the least visit.
                if len(visit_count) > 0:
                    best_delta = sorted(visit_count, key=lambda record: record[1])[0][0]
                    Rover.steer = angle + best_delta
                else:
                # 4) Otherwise simply steer the robot in the direction of the most navigable terrain.
                    Rover.steer = np.clip(angle, -15, 15)
                #################################################################################################
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
