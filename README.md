# vision_pkg

## Introduction
The code that actually runs on the plane is entirely contained in the `sniper_cam` package. Everything else is legacy code from before the 2017 crash.

## Image detection
### Plane side
The camera on the plane takes pictures at the rate of 15 Hz. The camera we are using, the Pointgrey Chameleon 3, has drivers provided by the manufacturer in the `pointgrey_camera_driver` package. These images are published raw and uncompressed on the `image_raw` topic. `image_stamper.py` subscribes to this topic and the `state` topic, which provides current state information of the plane (location, heading, etc). It then creates a message with both the raw image and relevant state information and publishes it to the `state_image` topic at the rate of 1 Hz.

This whole system begins by running `roslaunch sniper_cam onboard_camera.launch` in the correct workspace on the Odroid. For this to work, the following environment variables need to be set on the Odroid:
* `ROS_IP=Odroid's IP:1311`
* `ROS_HOSTNAME=Odroid's IP`
* `ROS_MASTER_URI=Odriod's IP`

Settings to ajust the white balance and color profiles is also in this launchfile.

### Ground side
To run the ground image detection software:
1. Ensure that your local environment variables are set to the following:
* `ROS_IP=Your Computer's IP`
* `ROS_HOSTNAME=Your Computer's IP`
* `ROS_MASTER_URI=Odroid's IP:1311`

An easy way to do this is to run `source .rosrc`, where `.rosrc` is the script in the root folder of this project. Make sure to edit this script first to use the proper IP addresses.

2. Ensure that you are connected to the same network as the Odroid.
3. If this is your first time running this, the proper folders need to be set up to hold images. This can be donw by running the following script from the root folder of this project: `.setup_imaging_gs`.
4. Run the command `roslaunch sniper_cam manual_imaging.launch`.

A GUI should open up showing an image count in the upper left hand corner incrementing as new images are saved. Here's what's going on:
* `state_image_writer.py` subscribes to the `state_image` topic. It takes the image portion of the message and saves it to `~/Desktop/vision_files/all_images` with a filename consisting of the timestamp.jpg. The state portion of the message is written the `~/Desktop/vision_files/all_state_files` folder with a filename of the same timestamp. The format is CSV.
* `sniper_geo_locator.py` reads images from the `all_images` folder. When you left click a spot on the image, it parses the state file and figures out the GPS location of the click point, copying the image to a `target_[number]` directory, and appends the located point to a file in `target_locations_sorted` directory.
* You cycle to another target with a right mouse click. 

