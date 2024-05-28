# XIAO nRF52840 Brake Light

This project is an experimental, tested demo utilizing the XIAO nRF52840 Sense by Seeed, which includes a built-in 6 DOF IMU (LSM6DS3TR).

With the LED matrix, you can create customized lighting effects to match your unique style. If you're a cyberpunk enthusiast, the RGB matrix provides endless possibilities for dynamic lighting.

> In essence, this project functions by parsing data from the IMU to control the LED matrix, enabling precise motion tracking and responsive lighting effects.

### Project Implementation

There are two primary approaches to building this project:
1. **Traditional Attitude Analysis**: Using conventional methods to process and interpret IMU data.
2. **Machine Learning Processing**: Leveraging machine learning techniques for advanced data analysis, potentially using platforms like Edge Impulse to develop and deploy models.

For this demo, I have implemented the project using the traditional method.

## Issues

In my project, there appears to be an issue with data drifting, likely due to my limited expertise in using IMUs. This drifting affects the stability and accuracy of the sensor readings over time.

### Potential Solutions(though I tried this.):

- **Calibration**: Regularly calibrate the IMU to minimize drift.
- **Sensor Fusion Algorithms**: Implement sensor fusion algorithms such as the Kalman Filter or Complementary Filter to improve stability.
- **Software Filtering**: Apply software filtering techniques to smooth out the noise and reduce drift.

Hoping someone with more experience in IMUs can help make this project more stable and reliable.!~ (It's too expensive to buy a bike braker on Amazon!)

## demostration videos

[video 1 - YouTube Shors](https://www.youtube.com/shorts/8ZCEmZu_6Wk)
[Video 2 - X](https://x.com/seeedstudio/status/1793634720548606059)

#### Minimum Hardware used
- [Seeed XIAO nRF52840 Sense](https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html)
- [Seeed Studio 6x10 RGB MATRIX for XIAO](https://www.seeedstudio.com/6x10-RGB-MATRIX-for-XIAO-p-5771.html)