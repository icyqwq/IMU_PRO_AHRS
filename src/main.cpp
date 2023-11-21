#include <M5Unified.h>
#include <deque>
#include <MadgwickAHRS.h>
#include <cmath>
#include <vector>
#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>
#include "m5backend.h"

// Define to use Reefwing AHRS (Attitude and Heading Reference System)
#define USE_REEFAHRS 1

// Initialization of AHRS and sensor objects
ReefwingAHRS ahrs;
Madgwick filter;
SensorData data;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
const long displayPeriod = 100;
unsigned long previousMillis = 0;
uint32_t diff;

LGFX_Sprite sprite(&M5.Display);

// Queues to store roll, pitch, and yaw data
std::deque<float> record_roll(320);
std::deque<float> record_pitch(320);
std::deque<float> record_yaw(320);

// Function to draw graphs for roll, pitch, and yaw
void drawGraph(const std::deque<float>& data, int startX, int startY, int W, int H, int color, float minRange, float maxRange) {
    for (size_t i = 1; i < data.size() && i < W; ++i) {
        float y1 = startY + H - ((data[i - 1] - minRange) / (maxRange - minRange) * H);
        float y2 = startY + H - ((data[i] - minRange) / (maxRange - minRange) * H);
        sprite.drawLine(startX + i - 1, y1, startX + i, y2, color);
    }
}

// Function to update and display sensor graphs
void updateGraph() {
    char buf[50];
    sprite.fillSprite(BLACK);

    // Set text size and color for display
    sprite.setTextSize(2);
    sprite.setTextDatum(TL_DATUM);

    // Display roll, pitch, and yaw values
    sprite.setTextColor(RED);
    sprintf(buf, "Roll  %.1f", record_roll.back());
    sprite.drawString(buf, 165, 0);
    sprite.setTextColor(GREEN);
    sprintf(buf, "Pitch %.1f", record_pitch.back());
    sprite.drawString(buf, 165, 20);
    sprite.setTextColor(BLUE);
    sprintf(buf, "Yaw   %.1f", record_yaw.back());
    sprite.drawString(buf, 165, 40);

    // Display accelerometer, gyroscope, and magnetometer data
    sprite.setTextSize(1);
    sprite.setTextColor(RED);
    sprintf(buf, "AX %.1f", data.ax);
    sprite.drawString(buf, 165, 60);
    sprintf(buf, "AY %.1f", data.ay);
    sprite.drawString(buf, 215, 60);
    sprintf(buf, "AZ %.1f", data.az);
    sprite.drawString(buf, 265, 60);

    sprite.setTextColor(GREEN);
    sprintf(buf, "GX %.1f", data.gx);
    sprite.drawString(buf, 165, 80);
    sprintf(buf, "GY %.1f", data.gy);
    sprite.drawString(buf, 215, 80);
    sprintf(buf, "GZ %.1f", data.gz);
    sprite.drawString(buf, 265, 80);

    sprite.setTextColor(BLUE);
    sprintf(buf, "MX %.1f", data.mx);
    sprite.drawString(buf, 165, 100);
    sprintf(buf, "MY %.1f", data.my);
    sprite.drawString(buf, 215, 100);
    sprintf(buf, "MZ %.1f", data.mz);
    sprite.drawString(buf, 265, 100);

    // Draw the graphs for roll, pitch, and yaw
    drawGraph(record_roll, 0, 160, 320, 80, RED, -180, 180);
    drawGraph(record_pitch, 0, 160, 320, 80, GREEN, -180, 180);
    drawGraph(record_yaw, 0, 160, 320, 80, BLUE, 0, 360);
}

// Task to update sensor data
void updateTask(void *args) {
    uint32_t t = 0;
    while (1) {
        float roll, pitch, yaw;

        // Read data from gyroscope, accelerometer, and magnetometer
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(data.gx, data.gy, data.gz);
        }
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(data.ax, data.ay, data.az);
        }
        if (IMU.magneticFieldAvailable()) {
            IMU.readMagneticField(data.mx, data.my, data.mz);
        }

        // Update the AHRS filter and compute orientation
        if (micros() - microsPrevious < microsPerReading) {
            taskYIELD();
            continue;
        }

        #if USE_REEFAHRS
            ahrs.setData(data);
            ahrs.update();
            roll = ahrs.angles.roll;
            pitch = ahrs.angles.pitch;
            yaw = ahrs.angles.yaw;
        #else
            filter.update(data.gx, data.gy, data.gz, data.ax, data.ay, data.az, data.mx, -data.my, data.mz);
            roll = filter.getRoll();
            pitch = filter.getPitch();
            yaw = filter.getYaw();
        #endif

        // Update the record queues
        record_roll.push_back(roll);
        record_pitch.push_back(pitch);
        record_yaw.push_back(yaw);
        record_roll.pop_front();
        record_pitch.pop_front();
        record_yaw.pop_front();

        diff = micros() - microsPrevious;
        microsPrevious = micros();
    }
}

// Function to calculate rotation matrix
void getRotationMatrix(float yaw, float pitch, float roll, Mat4 *matrix) {
    // Compute elements of the rotation matrix
    float cy = cos(yaw);
    float sy = sin(yaw);
    float cp = cos(pitch);
    float sp = sin(pitch);
    float cr = cos(roll);
    float sr = sin(roll);

    // Combine rotations in Z-Y-X order
    matrix->elements[0]  = cy * cr + sy * sp * sr;
    matrix->elements[1]  = -cy * sr + sy * sp * cr;
    matrix->elements[2]  = sy * cp;
    matrix->elements[3]  = 0.0;

    matrix->elements[4]  = cp * sr;
    matrix->elements[5]  = cp * cr;
    matrix->elements[6]  = -sp;
    matrix->elements[7]  = 0.0;

    matrix->elements[8]  = -sy * cr + cy * sp * sr;
    matrix->elements[9]  = sr * sy + cy * cr * sp;
    matrix->elements[10] = cy * cp;
    matrix->elements[11] = 0.0;

    // Homogeneous coordinate row
    matrix->elements[12] = 0.0;
    matrix->elements[13] = 0.0;
    matrix->elements[14] = 0.0;
    matrix->elements[15] = 1.0;
}

void setup() {
    // Initialize M5Stack
    M5.begin();

    // Create and display a black sprite on the screen
    sprite.createSprite(320, 240);
    sprite.fillSprite(BLACK);
    sprite.pushSprite(0, 0);

    // Initialize AHRS
    ahrs.begin();
    ahrs.setImuType(ImuType::BMI270_BMM150);
    ahrs.setDOF(DOF::DOF_9);
    ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    ahrs.setDeclination(-3.22); // Set magnetic declination for Shenzhen, China

    filter.begin(100);

    // Initialize 3D graphics
    Vec2i size = { 160, 160 };
    M5STACKBackend mB;
    M5STACKBackendInit(&mB, size);

    // Create and initialize renderer
    Renderer renderer;
    rendererInit(&renderer, size, (BackEnd*) &mB);
    Scene s;
    sceneInit(&s);
    rendererSetScene(&renderer, &s);

    // Create and add a teapot object to the scene
    Object tea;
    tea.mesh = &mesh_teapot;
    sceneAddRenderable(&s, object_as_renderable(&tea));
    tea.material = 0;

    float phi = 0; // Unused variable
    Mat4 t;

    // Initialize IMU (Inertial Measurement Unit)
    Wire.begin(2, 1, 400000);
    if (IMU.begin()) {
        // Output IMU connection status and sensor rates
        printf("BMI270 & BMM150 IMUs Connected.\nGyroscope sample rate = %f Hz\n\nGyroscope in degrees/second\nAccelerometer sample rate = %f Hz\n\nAcceleration in G's\nMagnetic field sample rate = %f Hz\n\nMagnetic Field in uT\n", IMU.gyroscopeSampleRate(), IMU.accelerationSampleRate(), IMU.magneticFieldSampleRate());
    } else {
        printf("BMI270 & BMM150 IMUs Not Detected.\n");
        while (1); // Infinite loop if IMU is not detected
    }

    // Set up timing for sensor readings
    microsPerReading = 1000000 / 100;
    microsPrevious = micros();

    // Create a task for continuous updates
    xTaskCreatePinnedToCore(updateTask, "updateTask", 8192, NULL, 1, NULL, 0);

    while (1) {
        // Update the graph with new data
        updateGraph();

        // Calculate roll, pitch, and yaw from recorded data
        float roll, pitch, yaw;
        roll = record_roll.back() * (PI / 180.0);
        pitch = record_pitch.back() * (PI / 180.0);
        yaw = (360-record_yaw.back()-90) * (PI / 180.0);

        // Set the projection matrix for the renderer
        renderer.camera_projection = mat4Perspective(2.0, 16.0, (float)size.x / (float)size.y, 20.0);

        // Set the view matrix for the renderer
        Mat4 v = mat4Translate((Vec3f){ 0, 0, -9 });
        Mat4 rotateDown = mat4RotateX(0.40);
        renderer.camera_view = mat4MultiplyM(&rotateDown, &v);

        // Set the transformation for the teapot
        tea.transform = mat4RotateZ(3.142128);
        t = mat4Translate((Vec3f){ 0, -0.5, 0 });
        tea.transform = mat4MultiplyM(&tea.transform, &t);

        // Update scene transformation based on IMU data
        Mat4 tr;
        getRotationMatrix(yaw, pitch, -roll, &tr);
        s.transform = tr;

        // Render the scene
        rendererSetCamera(&renderer, (Vec4i){ 0, 0, size.x, size.y });
        rendererRender(&renderer);

        // Update the display with the rendered image
        sprite.pushSprite(0, 0);
    }
}


void loop()
{
    
}
