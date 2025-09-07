#include "ahrs.hpp"



AHRS::AHRS(TwoWire& wireRef) : wire(wireRef), myIMU() {}

void AHRS::begin() {
    wire.begin();
    if (!myIMU.begin(BNO08X_ADDR, wire, BNO08X_INT, BNO08X_RST)) {
        Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1);
    }
    Serial.println("BNO08x found!");
    wire.setClock(400000); // 400kHz I2C
    setReports();
    Serial.println("Reading events");
    delay(100);
}

void AHRS::setReports() 
{
    Serial.println("Setting desired reports");
    /*
    Serial.println("Enabling GyroIntegratedRotationVector");
    if (myIMU.enableGyroIntegratedRotationVector(1)) {
        Serial.println(F("Gryo Integrated Rotation vector enabled"));
        Serial.println(F("Output in form i, j, k, real, gyroX, gyroY, gyroZ"));
    } else {
        Serial.println("Could not enable gyro integrated rotation vector");
    }*/

    Serial.println("Enabling RotationVector");
    
    //if (myIMU.enableRotationVector(3) == true) {
    if (myIMU.enableReport(SH2_ROTATION_VECTOR, 2500) == true){
        Serial.println(F("Rotation vector enabled"));
        Serial.println(F("Output in form i, j, k, real, accuracy"));
    } else {
        Serial.println("Could not enable rotation vector");
    }


    //if (myIMU.enableGyro(3) == true) {
    if (myIMU.enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, 2500) == true) {
        Serial.println(F("Gyro enabled"));
        Serial.println(F("Output in form x, y, z, in radians per second"));
    } else {
        Serial.println("Could not enable gyro");
    }
}

EulerAngles_t AHRS::quaternionToEuler(Quaternion_t q) 
{
    // Formulas from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // Using the convention: Roll (X), Pitch (Y), Yaw (Z)
    EulerAngles_t angles;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1)
        angles.pitch = copysignf(PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return angles; // returns angles in radians
}


void AHRS::getGyroData() 
{
    /*
    float x = myIMU.getGyroX();
    float y = myIMU.getGyroY();
    float z = myIMU.getGyroZ();

    Serial.print("GYRO: ");
    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);

    Serial.println();    
    */

    gyroX = myIMU.getGyroX();
    gyroY = myIMU.getGyroY();
    gyroZ = myIMU.getGyroZ();
}

void AHRS::getRotationVectorData() 
{
    /*
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();     


    Serial.print("ROT_VEC: ");
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    //Serial.print(F(","));
    //Serial.print(quatRadianAccuracy, 2);

    Serial.println();
    */

    Quaternion_t q;
    q.w = myIMU.getQuatReal();
    q.x = myIMU.getQuatI();
    q.y = myIMU.getQuatJ();
    q.z = myIMU.getQuatK();
    latestAttitudeEuler_rad = quaternionToEuler(q); // Convert quaternion to Euler angles
}



/*
One issue is that the reports come in separately, so we need to make sure we get both the gyroscope and rotation vector data,
and also that they are both not too old (within a tolerance).
*/
bool AHRS::update() {
    if (myIMU.wasReset()) {
        Serial.println("*** IMU Sensor was reset ***");
        setReports();
    }
    if (DBG_AHRS) Serial.print("A");

    bool gotGyro = false, gotRotVec = false;
    uint32_t now = _micros();

    // Process all available sensor events
    while (myIMU.getSensorEvent()) {
        uint8_t id = myIMU.getSensorEventID();
        if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
            getGyroData();
            lastGyroTimestamp_us = now;
            gotGyro = true;
            if (DBG_AHRS) Serial.print("G");
        } else if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
            getRotationVectorData();
            lastRotVecTimestamp_us = now;
            gotRotVec = true;
            if (DBG_AHRS) Serial.print("R");
        }
    }

    // Check if both samples are recent enough
    bool gyroFresh = (now - lastGyroTimestamp_us) <= maxSampleAge_us;
    bool rotVecFresh = (now - lastRotVecTimestamp_us) <= maxSampleAge_us;

    if (DBG_AHRS) Serial.print(gyroFresh && rotVecFresh);
    return gyroFresh && rotVecFresh;
}




/*
One issue is that the reports come in separately, so we need to make sure we get both the gyroscope and rotation vector data,
and also that they are from the same time frame.
*/
bool AHRS::update_old() { 
    if (myIMU.wasReset())
    {
        Serial.print("sensor was reset ");
        setReports();
    }

    // If no new event, return
    bool newEventAvailable = myIMU.getSensorEvent();
    if (newEventAvailable == false)
    {   
        if (DBG_AHRS) Serial.print("a");
        return false;
    }
    if (DBG_AHRS) Serial.print("b");
    /*
    if (myIMU.getSensorEventID() != SENSOR_REPORTID_GYROSCOPE_CALIBRATED && 
        myIMU.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) 
    {Serial.print("c");
        // Not the data we want, return false
        return false;
    }
    Serial.print("d");*/
    unsigned long startTime_us = _micros();
    bool success_gy = false;
    bool success_rv = false;

    do
    {Serial.print("e");        

        if (newEventAvailable == true) 
        {
            switch (myIMU.getSensorEventID()) {
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                Serial.print("f");
                // Get gyroscope data
                getGyroData();
                success_gy = true;
                break;

            case SENSOR_REPORTID_ROTATION_VECTOR:
                Serial.print("h");
                // Get rotation vector data
                getRotationVectorData();
                success_rv = true;
                break;
            }
        }



        // Check if we have both data
        if (success_gy && success_rv)
        {
            Serial.print("i");
            break;
        }
        else
        {
            // Check if we have a new event
            delayMicroseconds(100);
            Serial.print("j");
            newEventAvailable = myIMU.getSensorEvent();
        }
        
    } while (_micros() - startTime_us < PARAM_MAX_INTERVAL_BW_TIME_CORRELATED_SENSOR_REPORTS_US); 
    Serial.print("k");
    Serial.print(success_gy && success_rv);
    return success_gy && success_rv; // Return true if both data were successfully updated

}






bool AHRS::update_old2() 
{ 
    if (myIMU.wasReset())
    {
        Serial.print("sensor was reset ");
        setReports();
    }

    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true)
    {
        // is it the correct sensor data we want?
        uint8_t sensorEventId = myIMU.getSensorEventID();
        
        if ( myIMU.getSensorEventID() == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR)
        {

            gyroX = myIMU.getGyroIntegratedRVangVelX();
            gyroY = myIMU.getGyroIntegratedRVangVelY();
            gyroZ = myIMU.getGyroIntegratedRVangVelZ();
            float quatRadianAccuracy = myIMU.getQuatRadianAccuracy(); // Optional: get accuracy

            Quaternion_t q;
            q.w = myIMU.getGyroIntegratedRVReal();
            q.x = myIMU.getGyroIntegratedRVI();
            q.y = myIMU.getGyroIntegratedRVJ();
            q.z = myIMU.getGyroIntegratedRVK();
            latestAttitudeEuler_rad = quaternionToEuler(q); // Convert quaternion to Euler angles


            Serial.print("GYRO_INT_RV: ");
            /*
            Serial.print(RVI, 2);
            Serial.print(F(","));
            Serial.print(RVJ, 2);
            Serial.print(F(","));
            Serial.print(RVK, 2);
            Serial.print(F(","));
            Serial.print(RVReal, 2);
            Serial.print(F(","));
            Serial.print(gyroX, 2);
            Serial.print(F(","));
            Serial.print(gyroY, 2);
            Serial.print(F(","));
            Serial.print(gyroZ, 2);
            */

            // --- Quaternion to Euler Conversion ---
            // Formulas from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            // Using the convention: Roll (X), Pitch (Y), Yaw (Z)
            /*
            float RVI = myIMU.getGyroIntegratedRVI();
            float RVJ = myIMU.getGyroIntegratedRVJ();
            float RVK = myIMU.getGyroIntegratedRVK();
            float RVReal = myIMU.getGyroIntegratedRVReal();

            float quatI = RVI;
            float quatJ = RVJ;
            float quatK = RVK;
            float quatReal = RVReal;

            // Roll (x-axis rotation)
            float sinr_cosp = 2 * (quatReal * quatI + quatJ * quatK);
            float cosr_cosp = 1 - 2 * (quatI * quatI + quatJ * quatJ);
            rollQuat = atan2(sinr_cosp, cosr_cosp);

            // Pitch (y-axis rotation)
            float sinp = 2 * (quatReal * quatJ - quatK * quatI);
            if (abs(sinp) >= 1)
                pitchQuat = copysign(PI / 2, sinp); // Use 90 degrees if out of range
            else
                pitchQuat = asin(sinp);

            // Yaw (z-axis rotation)
            float siny_cosp = 2 * (quatReal * quatK + quatI * quatJ);
            float cosy_cosp = 1 - 2 * (quatJ * quatJ + quatK * quatK);
            yawQuat = atan2(siny_cosp, cosy_cosp);
            */
            // --- End of Conversion ---
            /*
            // Print Quaternion data
            Serial.print("Quaternion: i=");
            Serial.print(quatI, 4); // Print with 4 decimal places
            Serial.print(", j=");
            Serial.print(quatJ, 4);
            Serial.print(", k=");
            Serial.print(quatK, 4);
            Serial.print(", real=");
            Serial.print(quatReal, 4);
            // Serial.print(", accuracy="); Serial.print(quatRadianAccuracy, 4); // Optional accuracy
            Serial.println();
            
            return true; // Data was successfully updated
        } */

        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) 
        {
            float quatI = myIMU.getQuatI();
            float quatJ = myIMU.getQuatJ();
            float quatK = myIMU.getQuatK();
            float quatReal = myIMU.getQuatReal();
            float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

            Serial.print("ROT_VEC: ");
            Serial.print(quatI, 2);
            Serial.print(F(","));
            Serial.print(quatJ, 2);
            Serial.print(F(","));
            Serial.print(quatK, 2);
            Serial.print(F(","));
            Serial.print(quatReal, 2);
            //Serial.print(F(","));
            //Serial.print(quatRadianAccuracy, 2);

            Serial.println();

            return true; // Data was successfully updated
        }

        if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

            float x = myIMU.getGyroX();
            float y = myIMU.getGyroY();
            float z = myIMU.getGyroZ();

            Serial.print("GYRO: ");
            Serial.print(x, 2);
            Serial.print(F(","));
            Serial.print(y, 2);
            Serial.print(F(","));
            Serial.print(z, 2);

            Serial.println();

            return true; // Data was successfully updated
        }

    }
    return false; // No relevant event received
}
}
