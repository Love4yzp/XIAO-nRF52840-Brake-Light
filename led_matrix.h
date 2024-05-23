
#pragma once
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

/********** Patterns ***********/
const uint8_t pixels_right[6][10] = {
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
    {1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
};
const uint8_t pixels_left[6][10] = {
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0}
};
const uint8_t pixels_down[6][10] = {
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const uint8_t pixels_up[6][10] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
};

#define DELAYVAL       250 
#define SOFT_BRIGHTNESS_MAX 8
#define BRIGHTNESS_MAX 125

#define LED_PIN   A0
#define NUMPIXELS 60

#define HEX_RED_COLOR 0xFF0000
#define HEX_COLOR     HEX_RED_COLOR
#define ROWS          6
#define COLS          10

typedef enum LED_DIRECTIONS {
    LEFT  = 0,
    RIGHT = 1,
    UP    = 2,
    DOWN  = 3,
    SOFT_FULL,
    FULL,
} LED_DIRECTIONS;

uint8_t buf_pixels[ROWS][COLS] = {0};

class led_matrix : public Adafruit_NeoPixel {
  private:
    LED_DIRECTIONS cur_dir;
    LED_DIRECTIONS prev_dir;
    int shift_index        = 0;
    const int max_num_steps = 10;

  public:
    led_matrix() : Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800) {}
    uint16_t getPixelIndex(int row, int col) {
        return (COLS - 1 - col) * ROWS + row;
    }
    void begin() {
        Adafruit_NeoPixel::begin();  // Call the base class begin method
        setBrightness(SOFT_BRIGHTNESS_MAX);
        clear();  // Optional: Clear all pixels on startup
        show();   // Update the strip to turn off all LEDs
    }
    void draw_pixels_frame(void) {
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                if (buf_pixels[i][j]) {
                    uint16_t idx = getPixelIndex(i, j);
                    setPixelColor(idx, HEX_COLOR);
                }
            }
        }
    }

    void shift_frame_buffer(const uint8_t pixels[ROWS][COLS], LED_DIRECTIONS direction, int step = 1) {
        switch (direction) {
            case LED_DIRECTIONS::RIGHT:  // Right
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < COLS; j++) {
                        buf_pixels[i][(j + step) % COLS] = pixels[i][j];
                    }
                }
                break;
            case LED_DIRECTIONS::LEFT:  // Left
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < COLS; j++) {
                        buf_pixels[i][(COLS + j - step) % COLS] = pixels[i][j];
                    }
                }
                break;
            case LED_DIRECTIONS::DOWN:  // Down
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < COLS; j++) {
                        buf_pixels[(i + step) % ROWS][j] = pixels[i][j];
                    }
                }
                break;
            case LED_DIRECTIONS::UP:  // Up
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < COLS; j++) {
                        buf_pixels[(ROWS + i - step) % ROWS][j] = pixels[i][j];
                    }
                }
                break;
        }
    }
    // void animate_shift(uint8_t pixels[ROWS][COLS], LED_DIRECTIONS direction, int num_steps, int step_size = 1) {
    void animate_shift(LED_DIRECTIONS direction, int step_size = 1) {
        if (direction != cur_dir) {
            shift_index = 0;
            cur_dir     = direction;
        }
        clear();
        this->setBrightness(SOFT_BRIGHTNESS_MAX);
        switch (direction) {
            case LED_DIRECTIONS::LEFT:
                // Serial.println("LEFT - LED");
                shift_frame_buffer(pixels_left, direction, shift_index);
                break;
            case LED_DIRECTIONS::RIGHT:
                // Serial.println("RIGHT - LED");
                shift_frame_buffer(pixels_right, direction, shift_index);
                break;
            case LED_DIRECTIONS::UP:
                // Serial.println("UP - LED");
                shift_frame_buffer(pixels_up, direction, shift_index);
                break;
            case LED_DIRECTIONS::DOWN:
                // Serial.println("DOWN - LED");
                shift_frame_buffer(pixels_down, direction, shift_index);
                break;

            case LED_DIRECTIONS::SOFT_FULL:
                // if (shift_index % 2 == 0) {
                //     this->fill(HEX_COLOR);
                // } else {
                //     this->fill(0);
                // }
                this->fill(HEX_COLOR);
                goto end;
                break;
            case LED_DIRECTIONS::FULL:
                // Serial.println("FULL - LED");
                this->setBrightness(BRIGHTNESS_MAX);
                this->fill(HEX_COLOR);
                goto end;
                break;
            default:
                break;
        }
        draw_pixels_frame();
    end:
        show();
        shift_index = (shift_index + step_size) % max_num_steps;
    }
};

class NonBlockingDelay {
  private:
    unsigned long previousMillis;  // Last time
    unsigned long interval;        // interval to wait

  public:
    NonBlockingDelay(unsigned long interval)
        : interval(interval), previousMillis(0) {}

    /**
     * @brief if it is time to execute the function(similar to a timer)
     *
     * @return true
     * @return false
     */
    bool check() {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;  // update time
            return true;                     // exceute time
        }
        return false;  // not the time yet
    }

    /**
     * @brief Set the Interval time
     *
     * @param newInterval
     */
    void setInterval(unsigned long newInterval) { interval = newInterval; }
};

// void setup() {
//     led.begin();
// }

// void loop() {
//     // led.motion_right();
//     led.fill(HEX_COLOR);
//     led.show();
// }
