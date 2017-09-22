[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project

## Project Video

[![Udacity RoboND Project 1: Autonomous Rover Recording](http://img.img.youtube.com/vi/K9vMkEMIb4o)](https://youtu.be/K9vMkEMIb4o)

 The rover maps 50% of the terrain with 70% fidelity and successfully spots one rock on the journey, thus completing the basic project rubric.

## Jupyter Notebook Analysis

In order to identify the ground pixels over the wall and obstacle pixels the camera view was divided between light and dark with an RGB grey threshold of 160/255. This worked well enough for the purposes of this project, but could be optimised in future to achieve better fidelity.

The function from the Jupyter notebook is located in cell 10:

```python
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
```
In order to create a worldmap the code in cell 11 of the Jupyter notebook was used. The first function `rover_coords` converts the coordinates of the image into coordinates relvative to the position of the rover. The function `to_polar_coords` converts these x and y coordinates of the rover in the world plane into polar coordinates. The `rotate_pix` function rotates the picture according to the yaw of the rover in world space, while `translate_pix` scales and translates them. These two functions comabine in the `pix_to_world` function to turn the camera image from the rover into an image on the world map.

```python
# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world
```
The decision step function for the Rover is designed to steer the rover in the direction of the most navigable terrain. If there is no navigable terrain the Rover will stop and turn. The steering angle is clipped between +/- 15 degrees.

```python

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

```

    The perception step function from perception.py uses many of the same functions from the Jupyter notebook to map the navigable terrain and find rocks. Looking for the rocks just utilises a separate colour thresholding function looking in the red and green channels.

```python

    def perception_step(Rover):
        # Perform perception steps to update Rover()
        # TODO:
        dst_size = 5
        # Set a bottom offset to account for the fact that the bottom of the image
        # is not the position of the rover but a bit in front of it
        # this is just a rough guess, feel free to change it!
        bottom_offset = 6
        image = Rover.img
        # 1) Define source and destination points for perspective transform
        source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
        destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])
        # 2) Apply perspective transform
        warped, mask = perspect_transform(Rover.img, source, destination)
        threshed = color_thresh(warped)


        # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
        obs_map = np.absolute(np.float32(threshed) - 1) * mask



        # 4) Update Rover.vision_image (this will be displayed on left side of screen)
            # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
            #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
            #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
        Rover.vision_image[:,:,2] = threshed * 255
        Rover.vision_image[:,:,0] = obs_map * 255

        # 5) Convert map image pixel values to rover-centric coords

        xpix, ypix = rover_coords(threshed)

        world_size = Rover.worldmap.shape[0]

        scale = 2 * dst_size

        # 6) Convert rover-centric pixel values to world coordinates

        x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1],
                                        Rover.yaw, world_size, scale)

        obsxpix, obsypix = rover_coords(obs_map)

        obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1],
                                        Rover.yaw, world_size, scale)

        # 7) Update Rover worldmap (to be displayed on right side of screen)
            # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
            #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
            #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

        Rover.worldmap[y_world, x_world, 2] += 10
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1

        # 8) Convert rover-centric pixel positions to polar coordinates
            # Update Rover pixel distances and angles
            # Rover.nav_dists = rover_centric_pixel_distances
            # Rover.nav_angles = rover_centric_angles

        dist, angles = to_polar_coords(xpix, ypix)

        Rover.nav_angles = angles

        rock_map = find_rocks(warped, levels=(110, 110, 50))

        if rock_map.any():

            rock_x, rock_y = rover_coords(rock_map)

            rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0],
                                                        Rover.pos[1], Rover.yaw, world_size, scale)
            rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)

            rock_idx = np.argmin(rock_dist)

            rock_xcen = rock_x_world[rock_idx]

            rock_ycen = rock_x_world[rock_idx]

            Rover.worldmap[rock_ycen, rock_xcen, 1] * 255

            Rover.vision_image[:, :, 1] = rock_map * 255

        else:

            Rover.vision_image[:, :, 1] = 0

        return Rover
````
