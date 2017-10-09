# vision_pkg

## Introduction
The code that actually runs on the plane is entirely contained in the `sniper_cam` package. Everything else is legacy code from before the 2017 crash.

## Image detection
The camera on the plane takes pictures at the rate of 15 Hz. The camera we are using, the Pointgrey Chameleon 3, has drivers provided by the manufacturer in the `pointgrey_camera_driver` package. These images are published raw and uncompressed on the `image_raw` topic. `image_stamper.py` subscribes to this topic and the `state` topic, which provides current state information of the plane (location, heading, etc). It then creates a message with both the raw image and relevant state information and publishes it to the `state_image` topic at the rate of 1 Hz.

This whole system begins by running `roslaunch sniper_cam onboard_camera.launch` in the correct workspace on the Odroid. For this to work, the following environment variables need to be set on the Odroid:
* `ROS_IP=Odroid's IP`
* `ROS_HOSTNAME=Odroid's IP`
* `ROS_MASTER_URI=Odriod's IP`
