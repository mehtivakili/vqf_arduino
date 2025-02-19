#include <Wire.h>
#include <MPU9250.h>  // Ensure you have the correct IMU library installed

MPU9250 IMU(Wire, 0x68);

// Define the desired loop interval in microseconds (5 ms = 5000 µs)
const unsigned long LOOP_INTERVAL = 5000;  // 5ms in microseconds

// Variable to store the last loop time
unsigned long lastLoopTime = 0;

// Quaternion variables
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Initial quaternion (no rotation)
float gyro[3] = {0.0f, 0.0f, 0.0f};    // Gyroscope data
float accel[3] = {0.0f, 0.0f, 0.0f};   // Accelerometer data

// Filter coefficients for Butterworth filter
float gyro_b[3], gyro_a[2], accel_b[3], accel_a[2];
float gyro_state[2][3] = {{0, 0, 0}, {0, 0, 0}};  // For gyro filtering (state)
float accel_state[2][3] = {{0, 0, 0}, {0, 0, 0}};  // For accel filtering (state)

void butterworthCoeffs(float tau, float Ts, float* b, float* a) {
  // Calculate the cutoff frequency for Butterworth filter
  float fc = sqrt(2) / (2.0 * M_PI * tau);
  float C = tan(M_PI * fc * Ts);
  float D = C * C + sqrt(2) * C + 1;
  
  // Numerator coefficients (b0, b1, b2)
  b[0] = C * C / D;
  b[1] = 2 * b[0];
  b[2] = b[0];

  // Denominator coefficients (a1, a2) (excluding a0 which is 1)
  a[0] = 2 * (C * C - 1) / D;
  a[1] = (1 - sqrt(2) * C + C * C) / D;
}

void filterData(float* data, float* b, float* a, float state[2][3]) {
  for (int i = 0; i < 3; i++) {
    // Apply the filter to each axis (x, y, z)
    float filtered = b[0] * data[i] + state[0][i];
    state[0][i] = b[1] * data[i] - a[0] * filtered + state[1][i];
    state[1][i] = b[2] * data[i] - a[1] * filtered;
    
    // Store the filtered value back to data
    data[i] = filtered;
  }
}

void normalize(float* vec) {
  float norm = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  if (norm > 0.0f) {
    vec[0] /= norm;
    vec[1] /= norm;
    vec[2] /= norm;
  }
}

void updateQuaternion(float* gyro, float* accel, float* q, float dt, float tau) {
  float qDot[4];
  qDot[0] = 0.5f * (-q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2]);
  qDot[1] = 0.5f * (q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1]);
  qDot[2] = 0.5f * (q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0]);
  qDot[3] = 0.5f * (q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0]);

  // Integrate to get new quaternion
  q[0] += qDot[0] * dt;
  q[1] += qDot[1] * dt;
  q[2] += qDot[2] * dt;
  q[3] += qDot[3] * dt;

  // Normalize quaternion to avoid drift
  normalize(q);

  // Accelerometer correction (simplified)
  float ax = accel[0];
  float ay = accel[1];
  float az = accel[2];
  
  float accRef[4] = {0.0f, ax, ay, az};
  float beta = 0.1f;  // Sensitivity to accelerometer data
  float normAccRef = sqrt(accRef[1] * accRef[1] + accRef[2] * accRef[2] + accRef[3] * accRef[3]);
  if (normAccRef > 0.0f) {
    accRef[1] /= normAccRef;
    accRef[2] /= normAccRef;
    accRef[3] /= normAccRef;
  }

  float error[3] = {q[1] - accRef[1], q[2] - accRef[2], q[3] - accRef[3]};
  q[0] -= beta * error[0];
  q[1] -= beta * error[1];
  q[2] -= beta * error[2];

  // Normalize quaternion after correction
  normalize(q);
}

void quaternionToEuler(float* q, float* pitch, float* roll, float* yaw) {
  *pitch = asin(2.0f * (q[0] * q[1] - q[2] * q[3])) * 180.0f / M_PI;  // Convert to degrees
  *roll  = atan2(2.0f * (q[0] * q[2] + q[1] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 180.0f / M_PI;
  *yaw   = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 180.0f / M_PI;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize the I2C communication
  Wire.begin();
  
  // Initialize the MPU9250
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1); // Halt execution
  }

  // Configure IMU settings
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);          // +/-4G
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);       // +/-1000 deg/s
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ); // 10 Hz
  IMU.setSrd(4); // Sample Rate Divider: (1 + SRD) = 5 -> 200 Hz if internal rate is 1000 Hz

  // Calculate the filter coefficients
  butterworthCoeffs(0.1f, 0.005f, accel_b, accel_a);  // For accelerometer
  butterworthCoeffs(0.1f, 0.005f, gyro_b, gyro_a);    // For gyroscope
}

void loop() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {  // Time comparison in microseconds
    lastLoopTime += LOOP_INTERVAL; // Schedule next loop

    // Read sensor data from the IMU
    IMU.readSensor();

    // Get sensor data
    accel[0] = IMU.getAccelX_mss();
    accel[1] = IMU.getAccelY_mss();
    accel[2] = IMU.getAccelZ_mss();

    gyro[0] = IMU.getGyroX_rads();
    gyro[1] = IMU.getGyroY_rads();
    gyro[2] = IMU.getGyroZ_rads();

    // Apply Butterworth low-pass filter to the accelerometer data
    filterData(accel, accel_b, accel_a, accel_state);

    // Apply Butterworth low-pass filter to the gyroscope data
    filterData(gyro, gyro_b, gyro_a, gyro_state);

    // Normalize accelerometer data to get unit vector
    normalize(accel);

    // Perform the sensor fusion to update the quaternion
    updateQuaternion(gyro, accel, q, 0.005f, 0.1f);

    // Convert quaternion to Euler angles (pitch, roll, yaw)
    float pitch, roll, yaw;
    quaternionToEuler(q, &pitch, &roll, &yaw);

    // Output Euler angles (in degrees)
    Serial.print("Pitch: ");
    Serial.print(pitch, 6);
    Serial.print("\tRoll: ");
    Serial.print(roll, 6);
    Serial.print("\tYaw: ");
    Serial.println(yaw, 6);
  }
}
