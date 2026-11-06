"""g1_hand_camera: palm cameras of the Unitree G1 U6 Inspire hands as ROS 2 images.

Publishes ``/g1/hand/{left,right}/camera/{image_raw,image_raw/compressed,
camera_info,available}`` from the two small UVC cameras that look at the palm
of each hand. Runs on the Jetson Orin NX (the cameras are USB devices of the
Jetson hub); fully functional in mock mode without a camera or OpenCV.
"""
