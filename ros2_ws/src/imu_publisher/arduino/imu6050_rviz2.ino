#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object
MPU6050 mpu;

void setup() {
  // Start I2C communication
  Wire.begin();
  // Initialize MPU6050
  mpu.initialize();

  // Check if the MPU6050 is connected
  if (mpu.testConnection()) {
    Serial.begin(115200);  // Set baud rate to match ROS 2 serial read
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // Variables to hold raw sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Read raw accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print data in a format that can be easily parsed
  Serial.print(ax); Serial.print(","); 
  Serial.print(ay); Serial.print(","); 
  Serial.print(az); Serial.print(","); 
  Serial.print(gx); Serial.print(","); 
  Serial.print(gy); Serial.print(","); 
  Serial.println(gz);

  // Delay between readings
  delay(100);
}
