/*******************************************************************************
 * Project: RTES Gesture Unlock Embedded Challenge
 * File: main.cpp
 * 
 * Team Members:
 * - Shreyas Kootiganahalli Venugopal (sk12200)
 * - Regan Zhu (xz4089)
 * - Akasha Tigalappanavara (at5854)
 * 
 * Description:
 * This file contains the main program for the gesture-based unlocking system.
 ******************************************************************************/

#include <mbed.h>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <math.h>
#include "gyro.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"

// Definitions for event flags
#define KEY_FLAG 1
#define UNLOCK_FLAG 2
#define ERASE_FLAG 4
#define DATA_READY_FLAG 8

// Font size for LCD text
#define FONT_SIZE 16

// Threshold for unlocking; adjust to a lower value if unlocking is difficult (must be positive)
#define CORRELATION_THRESHOLD 0.005f

InterruptIn gyro_int2(PA_2, PullDown);
InterruptIn user_button(PC_13, PullDown);

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

LCD_DISCO_F429ZI lcd; // LCD instance
TS_DISCO_F429ZI ts; // Touchscreen instance

EventFlags flags; // Event flags for synchronization

Timer timer; // Timer instance

/*******************************************************************************
 * LCD and Touchscreen Function Prototypes
 ******************************************************************************/
void draw_button1(int x, int y, int width, int height, const char *label);
void draw_button2(int x, int y, int width, int height, const char *label);
bool is_touch_inside_button(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height);

/*******************************************************************************
 * Data Processing Function Prototypes
 ******************************************************************************/
float euclidean_distance(const array<float, 3> &a, const array<float, 3> &b);
float dtw(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t);
void trim_gyro_data(vector<array<float, 3>> &data);
float correlation(const vector<float> &a, const vector<float> &b);
array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2);

/*******************************************************************************
 * Thread Function Prototypes
 ******************************************************************************/
void gyroscope_thread();
void touch_screen_thread();

/*******************************************************************************
 * Flash Memory Function Prototypes
 ******************************************************************************/
bool storeGyroDataToFlash(vector<array<float, 3>> &gesture_key, uint32_t flash_address);
vector<array<float, 3>> readGyroDataFromFlash(uint32_t flash_address, size_t data_size);

/*******************************************************************************
 * Filter Function Prototypes
 ******************************************************************************/
// Moving average filter implementation declared later in the code
float movingAverageFilter(float input, float display_buffer[], size_t N, size_t &index, float &sum);

/*******************************************************************************
 * ISR Callback Functions
 ******************************************************************************/
void button_press() // Callback for button press interrupt
{
    flags.set(ERASE_FLAG);
}
void onGyroDataReady() // ISR for gyroscope data-ready signal
{
    flags.set(DATA_READY_FLAG);
}

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
vector<array<float, 3>> gesture_key; // Stores the reference gesture key
vector<array<float, 3>> unlocking_record; // Stores the unlocking attempt data

const int button1_x = 60;
const int button1_y = 80;
const int button1_width = 100;
const int button1_height = 50;
const char *button1_label = "Setup";
const int button2_x = 60;
const int button2_y = 180;
const int button2_width = 120;
const int button2_height = 50;
const char *button2_label = "Unlock";
const int message_x = 5;
const int message_y = 30;
const char *message = "GESTURE UNLOCK";
const int text_x = 5;
const int text_y = 270;
const char *text_0 = "NO KEY RECORDED";
const char *text_1 = "LOCKED";

int err = 0; // Error flag for correlation function

/*******************************************************************************
 * Main Function
 ******************************************************************************/
int main()
{
    lcd.Clear(LCD_COLOR_WHITE);
    
    // Render the first button
    draw_button1(button1_x, button1_y, button1_width, button1_height, button1_label);

    // Render the second button
    draw_button2(button2_x, button2_y, button2_width, button2_height, button2_label);

    // Display the welcome message on the LCD
    lcd.DisplayStringAt(message_x, message_y, (uint8_t *)message, CENTER_MODE);

    // Configure interrupts for button and gyroscope
    user_button.rise(&button_press);
    gyro_int2.rise(&onGyroDataReady);

    // Initialize LEDs based on gesture key state
    if (gesture_key.empty())
    {
        red_led = 0;
        green_led = 1;
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_0, CENTER_MODE);
    }
    else
    {
        red_led = 1;
        green_led = 0;
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_1, CENTER_MODE);
    }

    // Start the thread for handling gyroscope data
    Thread key_saving;
    key_saving.start(callback(gyroscope_thread));

    // Start the thread for touchscreen interaction
    Thread touch_thread;
    touch_thread.start(callback(touch_screen_thread));

    // Keep the main thread running indefinitely
    while (1)
    {
        ThisThread::sleep_for(100ms);
    }
}

/*******************************************************************************
 * Gyroscope Gesture Key Handling Thread
 ******************************************************************************/
void gyroscope_thread()
{
    // Set up parameters for initializing the gyroscope
    Gyroscope_Init_Parameters init_parameters;
    init_parameters.conf1 = ODR_200_CUTOFF_50;
    init_parameters.conf3 = INT2_DRDY;
    init_parameters.conf4 = FULL_SCALE_500;

    // Raw data structure for gyroscope readings
    Gyroscope_RawData raw_data;

    // Display buffer for messages shown on the LCD
    char display_buffer[50];

    // Account for potential interrupt signal already active from the gyroscope
    if (!(flags.get() & DATA_READY_FLAG) && (gyro_int2.read() == 1))
    {
        flags.set(DATA_READY_FLAG);
    }

    while (1)
    {
        vector<array<float, 3>> temp_key; // Temporary storage for recording gyroscope data

        // Wait for relevant event flags (key recording, unlocking, or erase)
        auto flag_check = flags.wait_any(KEY_FLAG | UNLOCK_FLAG | ERASE_FLAG);

        if (flag_check & ERASE_FLAG)
        {
            // Handle erasing the stored gesture key and unlocking record
            sprintf(display_buffer, "Erasing....");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  // Clear the text area
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   // Display erasing message
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            gesture_key.clear();
            
            // Clear the unlocking record
            sprintf(display_buffer, "Key Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  // Clear the text area
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);
            lcd.SetTextColor(LCD_COLOR_BLUE);                   // Display completion message
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            unlocking_record.clear();

            // Reset LED states and display completion message
            green_led = 1;
            red_led = 0;
            sprintf(display_buffer, "All Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        if (flag_check & (KEY_FLAG | UNLOCK_FLAG))
        {
            sprintf(display_buffer, "Hold On");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Calibrating...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            // Initialize the gyroscope sensor
            InitiateGyroscope(&init_parameters, &raw_data);

            // Begin gesture recording
            sprintf(display_buffer, "Recording in 3...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);
            sprintf(display_buffer, "Recording in 2...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);
            sprintf(display_buffer, "Recording in 1...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            ThisThread::sleep_for(1s);

            sprintf(display_buffer, "Recording...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            
            // Capture gyroscope data for 5 seconds
            timer.start();
            while (timer.elapsed_time() < 5s)
            {
                // Wait for gyroscope data availability
                flags.wait_all(DATA_READY_FLAG);
                // Retrieve calibrated raw data from the gyroscope
                GetCalibratedRawData();
                // Append converted data to the temporary key vector
                temp_key.push_back({ConvertToDPS(raw_data.x_raw), ConvertToDPS(raw_data.y_raw), ConvertToDPS(raw_data.z_raw)});
                ThisThread::sleep_for(50ms); // 20Hz sampling rate
            }
            timer.stop();  // Stop the timer
            timer.reset(); // Reset the timer

            // Remove zero entries from the recorded data
            trim_gyro_data(temp_key);

            sprintf(display_buffer, "Finished...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
        }

        // Handle key saving or unlocking based on the event flag
        if (flag_check & KEY_FLAG)
        {
            if (gesture_key.empty())
            {
                sprintf(display_buffer, "Saving Key...");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // Save the new gesture key
                gesture_key = temp_key;

                // Clear temporary storage
                temp_key.clear();

                // Update LED states
                red_led = 1;
                green_led = 0;

                sprintf(display_buffer, "Key saved...");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
            }
            else
            {
                sprintf(display_buffer, "Removing old key...");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                ThisThread::sleep_for(1s);
                
                // Discard the previous key
                gesture_key.clear();

                // Save the new gesture key
                gesture_key = temp_key;

                sprintf(display_buffer, "New key is saved.");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // Clear temporary storage
                temp_key.clear();

                // Update LED states
                red_led = 1;
                green_led = 0;
            }
        }
        else if (flag_check & UNLOCK_FLAG)
        {
            flags.clear(UNLOCK_FLAG);
            sprintf(display_buffer, "Unlocking...");
            lcd.SetTextColor(LCD_COLOR_WHITE);                  
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
            lcd.SetTextColor(LCD_COLOR_BLUE);                   
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

            // Save the unlocking attempt data
            unlocking_record = temp_key; 
            temp_key.clear(); // Clear temporary storage

            // Check if the gesture key is stored
            if (gesture_key.empty())
            {
                sprintf(display_buffer, "NO KEY SAVED.");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                // Clear the unlocking record
                unlocking_record.clear();

                // Update LED states
                green_led = 1;
                red_led = 0;
            }
            else // Compare the unlocking data with the gesture key
            {
                int unlock = 0; // Counter for coordinates meeting the threshold

                // Calculate correlation between the stored key and unlocking record
                array<float, 3> correlationResult = calculateCorrelationVectors(gesture_key, unlocking_record);

                if (err != 0)
                {
                    printf("Error calculating correlation: vectors have different sizes\n");
                }
                else
                {
                    printf("Correlation values: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
                    // Check if all correlation values exceed the threshold
                    for (size_t i = 0; i < correlationResult.size(); i++)
                    {
                        if (correlationResult[i] > CORRELATION_THRESHOLD)
                        {
                            unlock++;
                        }
                    }
                }

                if (unlock == 3) // Unlock condition; refine threshold if necessary
                {
                    sprintf(display_buffer, "UNLOCK: SUCCESS");
                    lcd.SetTextColor(LCD_COLOR_GREEN);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_BLACK);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                    
                    // Update LED states
                    green_led = 1;
                    red_led = 0;

                    // Clear the unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
                else
                {
                    sprintf(display_buffer, "UNLOCK: FAILED");
                    lcd.SetTextColor(LCD_COLOR_RED);                  
                    lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                    lcd.SetTextColor(LCD_COLOR_BLACK);                   
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);

                    // Update LED states
                    green_led = 0;
                    red_led = 1;

                    // Clear the unlocking record
                    unlocking_record.clear();
                    unlock = 0;
                }
            }
        }
        ThisThread::sleep_for(100ms);
    }
}

/*******************************************************************************
 * Touchscreen Interaction Handling Thread
 ******************************************************************************/
void touch_screen_thread()
{
    // Initialize and handle touchscreen interactions
    TS_StateTypeDef ts_state;

    if (ts.Init(lcd.GetXSize(), lcd.GetYSize()) != TS_OK)
    {
        printf("Failed to initialize the touch screen!\r\n");
        return;
    }

    // Display buffer for touchscreen status messages
    char display_buffer[50];

    while (1)
    {
        ts.GetState(&ts_state);
        if (ts_state.TouchDetected)
        {
            int touch_x = ts_state.X;
            int touch_y = ts_state.Y;

            // Check if the touch coordinates are within the recording button's area
            if (is_touch_inside_button(touch_x, touch_y, button2_x, button2_y, button1_width, button1_height))
            {
                sprintf(display_buffer, "Starting Recording...");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(KEY_FLAG);
            }

            // Check if the touch coordinates are within the unlock button's area
            if (is_touch_inside_button(touch_x, touch_y, button1_x, button1_y, button2_width, button2_height))
            {
                sprintf(display_buffer, "Unlocking Initiated...");
                lcd.SetTextColor(LCD_COLOR_WHITE);                  
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); 
                lcd.SetTextColor(LCD_COLOR_BLUE);                   
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
                ThisThread::sleep_for(1s);
                flags.set(UNLOCK_FLAG);
            }
        }
        ThisThread::sleep_for(10ms);
    }
}

/*******************************************************************************
 * Store Gyroscope Data to Flash Memory
 ******************************************************************************/
bool storeGyroDataToFlash(vector<array<float, 3>> &gesture_key, uint32_t flash_address)
{
    FlashIAP flash;
    flash.init();

    // Calculate total size of the data in bytes
    uint32_t data_size = gesture_key.size() * sizeof(array<float, 3>);

    // Erase flash memory at the specified address
    flash.erase(flash_address, data_size);

    // Write data to flash memory
    int write_result = flash.program(gesture_key.data(), flash_address, data_size);

    flash.deinit();

    return write_result == 0;
}

/*******************************************************************************
 * Read Gyroscope Data from Flash Memory
 ******************************************************************************/
vector<array<float, 3>> readGyroDataFromFlash(uint32_t flash_address, size_t data_size)
{
    vector<array<float, 3>> gesture_key(data_size);

    FlashIAP flash;
    flash.init();

    // Read data from flash memory
    flash.read(gesture_key.data(), flash_address, data_size * sizeof(array<float, 3>));

    flash.deinit();

    return gesture_key;
}

/*******************************************************************************
 * Draw a Button on the LCD
 ******************************************************************************/
void draw_button1(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(LCD_COLOR_LIGHTRED);
    lcd.FillRect(x, y, width, height);
    lcd.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

void draw_button2(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(LCD_COLOR_LIGHTBLUE);
    lcd.FillRect(x, y, width, height);
    lcd.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

/*******************************************************************************
 * Check if a Touch Point is Inside a Button
 ******************************************************************************/
bool is_touch_inside_button(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height)
{
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

/*******************************************************************************
 * Calculate the Euclidean Distance Between Two Vectors
 ******************************************************************************/
float euclidean_distance(const array<float, 3> &a, const array<float, 3> &b)
{
    float sum = 0;
    for (size_t i = 0; i < 3; ++i)
    {
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return sqrt(sum);
}

/*******************************************************************************
 * Compute the Dynamic Time Warping (DTW) Distance Between Two Sequences
 ******************************************************************************/
float dtw(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t)
{
    vector<vector<float>> dtw_matrix(s.size() + 1, vector<float>(t.size() + 1, numeric_limits<float>::infinity()));

    dtw_matrix[0][0] = 0;

    for (size_t i = 1; i <= s.size(); ++i)
    {
        for (size_t j = 1; j <= t.size(); ++j)
        {
            float cost = euclidean_distance(s[i - 1], t[j - 1]);
            dtw_matrix[i][j] = cost + min({dtw_matrix[i - 1][j], dtw_matrix[i][j - 1], dtw_matrix[i - 1][j - 1]});
        }
    }

    return dtw_matrix[s.size()][t.size()];
}

/*******************************************************************************
 * Trim Leading and Trailing Zeros from Gyroscope Data
 ******************************************************************************/
void trim_gyro_data(vector<array<float, 3>> &data)
{
    float threshold = 0.00001;
    auto ptr = data.begin();
    // Locate the first non-zero data entry
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr++;
    }
    if (ptr == data.end())
        return;      // All data entries are below the threshold
    auto lptr = ptr; // Record the left bound
    // Search for the last non-zero data entry
    ptr = data.end() - 1;
    while (abs((*ptr)[0]) <= threshold && abs((*ptr)[1]) <= threshold && abs((*ptr)[2]) <= threshold)
    {
        ptr--;
    }
    auto rptr = ptr; // Record the right bound
    // Move valid data entries to the front
    auto replace_ptr = data.begin();
    for (; replace_ptr != lptr && lptr <= rptr; replace_ptr++, lptr++)
    {
        *replace_ptr = *lptr;
    }
    // Remove trailing zeros
    if (lptr > rptr)
    {
        data.erase(replace_ptr, data.end());
    }
    else
    {
        data.erase(rptr + 1, data.end());
    }
}

/*******************************************************************************
 * Compute Correlation Between Two Vectors
 ******************************************************************************/
float correlation(const vector<float> &a, const vector<float> &b)
{
    // Verify that the two vectors have the same size
    if (a.size() != b.size())
    {
        err = -1;
        return 0.0f;
    }

    float sum_a = 0, sum_b = 0, sum_ab = 0, sq_sum_a = 0, sq_sum_b = 0;

    for (size_t i = 0; i < a.size(); ++i)
    {
        sum_a += a[i];
        sum_b += b[i];
        sum_ab += a[i] * b[i];
        sq_sum_a += a[i] * a[i];
        sq_sum_b += b[i] * b[i];
    }

    size_t n = a.size(); // Total number of elements

    float numerator = n * sum_ab - sum_a * sum_b; // Covariance
    
    float denominator = sqrt((n * sq_sum_a - sum_a * sum_a) * (n * sq_sum_b - sum_b * sum_b)); // Standard deviation

    return numerator / denominator;
}

/*******************************************************************************
 * Calculate Correlation Values for Each Dimension
 ******************************************************************************/
array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2) {
    array<float, 3> result;

    // Compute the correlation for each axis
    for (int i = 0; i < 3; i++) {
        vector<float> a;
        vector<float> b;

        // Populate vectors with the ith coordinate values
        for (const auto& arr : vec1) {
            a.push_back(arr[i]);
        }
        for (const auto& arr : vec2) {
            b.push_back(arr[i]);
        }

        // Adjust the size of vectors to be equal, if needed
        if (a.size() > b.size()) {
            a.resize(b.size(), 0);
        } else if (b.size() > a.size()) {
            b.resize(a.size(), 0);
        }

        // Compute the correlation and store it in the result
        result[i] = correlation(a, b);
    }

    return result;
}
