

/*DEFINE: high-level sensor structs, state estimation time step, turn-on bias time step
Initialize attitude struct with zero values.
/


//SETUP
/*
Wait for Xbee signal to start the sensor calibration

1. Determine sensor turn-on bias.
    a. Initialize arrays of 1000 elements
    b. Run a while loop - condition is while arrays are not yet filled
        c. Fill arrays with latest sensor measurements
    d. Once while loop terminates, take average of each array. This is the turn-on bias.
    e. Add the turn-on bias for each sensor measurement to each sensor measurement's struct.
    f. Log the global time at which the turn-on bias was determined.
2. Determine sensor run-on bias.
    a. Initialize arrays to store values of average slope of sensor measurement error with respect to time.
    b. while loop for one minute (condition is that current time is one minute beyond time of turn-on bias determination)
    c. Read each sensor.
    d. Determine instantaneous slope of sensor measurements with respect to time.
    e. Store slope value from (c) in the circular buffer for each sensor measurement.
    f. At intervals in the while loop, append the current average circular buffer slope to the arrays declared in (a).
    g. Once while loop has terminated, take average value of arrays from (a) and take this as de/dt for each sensor measurement.
    h. Store the de/dt of each sensor measurement in the struct for that sensor.
3. Determine initial attitude.
    a. Accelerometer gives roll and pitch, magnetometer gives yaw. Covnert this to initial quaternion.
    b. Store initial attitude quaternion in attitude struct.

Ground calibration is now complete. Send Xbee signal to ground station that calibration is complete and rocket is ready to be launched.
*/

//LOOP
/*
Awaiting Launch
Powered Ascent
Unpowered Ascent
*/


