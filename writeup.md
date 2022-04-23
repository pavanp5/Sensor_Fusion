# Writeup: Track 3D-Objects Over Time

Link to github repo: https://github.com/pavanp5/Sensor_Fusion


**Results**
Step 1
<img src="img/output/step 1_rmse.PNG"/>
<img src="img/output/step 1_tracking.PNG"/>

Step 2
<img src="img/output/step 2_rmse.PNG"/>
<img src="img/output/step 2_tracking.PNG"/>

Step 3
<img src="img/output/step 3_rmse.PNG"/>
<img src="img/output/step 3_tracking.PNG"/>

Step 4
<img src="img/output/step 4_rmse.PNG"/>
<img src="img/output/step 4_tracking.PNG"/>

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

**filter.py**

The filter.py has the predict and the update functions.

Upon receiving a new measurement, predict() executes, subsequently update() executes.

Based on time elapsed between measurements received the predict computes returns the state x- and estimation error covariance P-.

The update uses the inputs from predict along with the new measurement from sensor and computes Kalman gain and subsequently returns the updated state x+ and updated estimation error covariance P+.

**Detailed explaination on predict and update:**
The filter.py has the state transition funtion F(), the process noise covariance matrix function Q(), the predict(), and update() fuctions.

The state x has the position and velocity information.
On receiving a new measurement. The time elapsed between receiving previous meaurement and current measurement is computed.

The predict function predicts the state as x- and the estimation error P- using the state transition function and the process noise covariance Q.

Once the predict is executed in this way the Update() function is executed. 

It takes the predicted state x- & predicted estimation error covariance P- and new sensor measurement Z as inputs.
It calculates the gamma i.e. the measurement error. To find the measurement error the state and the sensor need to be in same coordinate system. This is achieved using the transform H i.e (x-)*H.

This gamma is the estimation error is calculated as Z - (x-)*H.
We assume the measurement noise covariance R based on our detections in 3d object detection project and its standard deviation in box center coordinates of detections.

We calculate the residual covariance S, using the estimation error covariance P and the measurement noise covariance R.

Subsequently we calculate the Kalman gain K using estimation error covariance P- and the residual covariance S. 

Using the Kalman gain K calculated and the gamma we perform the update of state x- to get new state x+ and new estimation error covariance P+


**track management**
The track_management.py has the track and trackmanagement classes. 
The manage_tracks loops through unassigned tracks and deletes the confirmed tracks if score is below threshold or if the estimation error covariace is too high. If the score is above the threshold it reduces the score. New tracks are initialized for unassigned measurements.

The handle_updated_tracks increases the track score and updates the track status to confirmed based on score and threshold defined.

When new track is initialized the measurement is assigned to the state. The estimation error covariance is assigned a predefined value from parameters.

**association.py**
The association.py has
associate, associate_and_update, get_closest_track_and_meas, MHD, and gating functions.

The associate function returns the association matrix of tracks with measurements and mahalnobis distances between measurement and tracks based on MHD() function and gating() function.

The get_closest_track_and_meas() associates a track to the closest measurement. 

The associate_and_update() function updates only tracks in field of view. It calls get_closest_track_and_meas() to associate measurement to a track and there by uses kalman filter to update the track. 

Subsequently it calls manage_tracks() function to handle unassigned tracks and measurements. New tracks are created for unassigned measurements and track score is decreased for unassigned tracks that are in field of view.

Finally the handle_updated_track() is called to update track score of all tracks that are updated. Based on track score threshold the tracks are either deleted, set to tentative or confirmed.

**camera fusion**
The Kalman filter works for distributions that are gaussian. For the Vehicle tracking we use state x that has position and velocity information. While for the lidar the transform from state to sensor coordinate space is linear model, for camera transform we have a non linear function and so, is not a gaussian distribution. In order to use kalman filter for camera fusion we take the first derivative of the camera transform. The first derivative of hx is the Jocobian Hj a 2*6 matrix.It is a linear function and hence a gaussian. This lets use kalman filter for camera fusion.The first derivate of transform is computed in get_H() function when sensor.name='camera'.



### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
The Lidar & Camera measurements each provide a specific detail. The Lidar has good depth resolution but poor  resolution in tangential direction. The Camera data has good angular resolution but no radial or depth resolution information. So, combining both predictions to update the state gives more accurate results.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
In a real life sensor fusion system, the sensors need to have a synchronized clock, this would be achived using an appropriate hardware. In addition there could be challenges such a vibration of the sensors. The vibration of sensor in tangential direction distorts the lidar points captured over a period of 100ms per say which is the camera frame rate. Hence it would yeild blurred objects in the point cloud due to the vibration of the sensor. 


### 4. Can you think of ways to improve your tracking results in the future?
Yang, Wen et al. work reccomends the use of an APX inertial system or lidar odometry to perform ego correction on distortion caused due to sensor vibration. They suggest unified coordinate and calibrations so all sensors are in the same coordinate system and time synchronization is done at the hardware level.   

[Yang, Wen and Gong, Zheng et al. "Lidar with Velocity: Motion Distortion Correction of Point Clouds from Oscillating Scanning Lidars"  2021](https://arxiv.org/abs/2111.09497)



