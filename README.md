# Vision-based-state-estimation-of-the-quadrotor

## Overview
The goal of the project is to implemented vision-based 3D pose estimator, OpticalFlow and RANSAC for finding the position, and orientation
of the quadrotor along with linear and angular velocity of the camera.

## Installations
Requires MATLAB to run the files in the code folder.

## Data Format

### Environment and Calibration

* The data for this phase was collected using a Nano+ quadrotor which contains **AprilTags, each of which has a unique ID, intrinsic camera calibration matrix and the transformation between the camera and the robot center** can be found in
  ```bash
  ├── code 
        ├── parameters.txt
        
* Two photos **(top_view.jpg and side_view.jpg)** are included to visualize the camera-robot transform. You will need to transform your camera-based pose estimate from the camera frame to the robot frame so that you can compare it against the ground truth Vicon data.

  ```bash
  ├── code
        ├── top_view.jpg
        ├── side_view.jpg

### Pose Estimation
* Data is present in
    ```bash
     ├── code
          ├── data
              ├── studentdata1.mat
              ├── studentdata4.mat
  
* The data for each trial is provided in a mat file. The file contains a struct array of image data called data, which holds all of the data necessary to do pose estimation. The format of the image data struct is as follows:
  
<div align="center">
  <img src="https://github.com/Naveench7/Vision-based-state-estimation-of-the-quadrotor/assets/100085132/fb79db5e-6715-4176-9319-a3dba597e212" width="200" height="200">
</div>


  
  
      - Time stamp (t) in seconds.
      - ID of every AprilTag that is observed in the image (id).
      - The center (p0) and four corners of every AprilTag in the image. The corners are given in the order
        bottom left (p1), bottom right (p2), top right (p3), and top left (p4). The ith column
        in each of these fields corresponds to the ith tag in the ID field and the values are expressed in image
        coordinates.
      - Rectified image (img).

* The mat file also contains Vicon data taken at 100 Hz, which will serve as our ground truth measurements.The Vicon data is stored in two matrix variables, time and vicon. The time variable contains the timestamp while the vicon variable contains the Vicon data.

## Results

<table>
  <tr>
    <td>
      <img src="https://github.com/Naveench7/Vision-based-state-estimation-of-the-quadrotor/assets/100085132/baf06f56-56d9-44d1-99f7-f295f0675412" width="500" height="500">
    </td>
    <td>
      <img src="https://github.com/Naveench7/Vision-based-state-estimation-of-the-quadrotor/assets/100085132/062998b0-6d5e-49c7-a422-d136e566456e" width="500" height="500">
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/Naveench7/Vision-based-state-estimation-of-the-quadrotor/assets/100085132/159f3464-6545-4e4f-9154-d47ae512f142" width="500" height="500">
    </td>
    <td>
      <img src="https://github.com/Naveench7/Vision-based-state-estimation-of-the-quadrotor/assets/100085132/77bdb9bb-7507-4a2f-bb4c-14e8df736cc0" width="500" height="500">
    </td>
  </tr>
</table>




      




  
  




