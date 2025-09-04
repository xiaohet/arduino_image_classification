#include <_Image_Capture_2_inferencing.h>
// #include <Xiaohe-project-1_inferencing.h>
#include "setup.h"
#include "ingestion-sdk-platform/nano-ble33/ei_at_handlers.h"
#include "ingestion-sdk-platform/nano-ble33/ei_flash_nano_ble33.h"
#include "ingestion-sdk-platform/nano-ble33/ei_device_nano_ble33.h"
#include "sensors/ei_microphone.h"
#include "sensors/ei_inertialsensor.h"
#include "sensors/ei_inertialsensor_rev2.h"
#include "sensors/ei_environmentsensor.h"
#include "sensors/ei_environmental_rev2.h"
#include "sensors/ei_interactionsensor.h"
#include "sensors/ei_camera.h"
#include "ingestion-sdk-c/ei_run_impulse.h"
#include "firmware-sdk/ei_device_info_lib.h"

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <Arduino_OV767X.h>
#include "firmware-sdk/at-server/ei_at_server.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"


static ATServer *at;

mbed::DigitalOut led(LED1);

#define CAMERA_WIDTH  EI_CLASSIFIER_INPUT_WIDTH
#define CAMERA_HEIGHT EI_CLASSIFIER_INPUT_HEIGHT

unsigned short features_read[CAMERA_WIDTH * CAMERA_HEIGHT];
unsigned short image[176 * 144];

void capture_image() {
    ei_printf("capture image\n");
    // ei_camera_capture(176, 144, image);
    ei_printf("capture image end\n");
  
    // Normalize to 0.0 ~ 1.0 float range:
    for (int i = 0; i < CAMERA_WIDTH * CAMERA_HEIGHT; i++) {
        features_read[i] = (float)image[i] / 255.0f;
    }
}

// This is required by Edge Impulse to access your features_read
int get_feature_data(size_t offset, size_t length, float *out_ptr) {
    // Example: assuming your feature array is float features_read[]
    memcpy(out_ptr, features_read + offset, length * sizeof(float));
    return 0;
}

void print_memory_info()
{
    // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) ei_malloc(cnt * sizeof(mbed_stats_stack_t));

    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        ei_printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
    }
    free(stats);

    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    ei_printf("Heap size: %lu / %lu bytes\r\n", heap_stats.current_size, heap_stats.reserved_size);
}

void ei_main_init(void)
{
    EiDeviceNanoBle33 *dev = static_cast<EiDeviceNanoBle33*>(EiDeviceInfo::get_device());
    ei_printf("Hello from Edge Impulse on Arduino Nano 33 BLE Sense\r\n"
              "Compiled on %s %s\r\n",
              __DATE__,
              __TIME__);

    // we cannot flash anymore after hitting a hard fault, so let's wait 10 seconds
    // so we can attach to the serial port
    // for (size_t ix = 0; ix < 10; ix++) {
    //     ei_printf("Waiting to start: %lu\n", ix);
    //     led = !led;
    //     wait_ms(1000);
    // }
    
    if (ei_inertial_init() == false) {        
        ei_inertial_rev2_init();
    }
    
    if (ei_environment_init() == false) {
        ei_environment_rev2_init();
    }
    
    ei_interaction_init();
    // ei_camera_init();
    ei_microphone_init();

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
    
    Serial.begin(9600);
    while (!Serial);

    Serial.println("OV767X Camera Capture");
    Serial.println();

    if (!Camera.begin(QCIF, RGB565, 1)) {
         Serial.println("Failed to initialize camera!");
         while (1);
    }

    Serial.println("Send the 'c' character to read a frame ...");
    Serial.println();
}

void ei_main()
{
    /* handle command comming from uart */

    /*
    char data = Serial.read();
    while (data != 0xFF) {
        at->handle(data);
        data = Serial.read();
    }
    */

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_feature_data;

    ei_impulse_result_t result;
    
    if (Serial.read() == 'c') {
        Serial.println("Reading frame");
        Serial.println();
        Camera.readFrame(image);

        Serial.print("image read");
        Serial.println();

        int feature_cnt = 0;
        int col_read, row_read, col_cls, row_cls, ptr_read;
        for (int feature_cnt = 0; feature_cnt < CAMERA_WIDTH * CAMERA_HEIGHT; feature_cnt++) {
            col_cls = feature_cnt % CAMERA_WIDTH;
            row_cls = feature_cnt / CAMERA_WIDTH;
            col_read = col_cls * 176 / CAMERA_WIDTH;
            row_read = row_cls * 144 / CAMERA_HEIGHT;
            ptr_read = row_read * 176 + col_cls;
            features_read[row_cls * CAMERA_WIDTH + col_cls] = (unsigned short)(image[ptr_read]); // / 65535.0f;
            Serial.print("0x");
            Serial.print(image[ptr_read], HEX);
            Serial.print(", ");
        }
        
        Serial.print("features read");
        Serial.println();


        // Assuming `features_read` holds the input data (e.g., image buffer):
        //ei_printf("Free memory before: %lu\n", xPortGetFreeHeapSize());
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        //ei_printf("Free memory after: %lu\n", xPortGetFreeHeapSize());
        
        if (res != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", res);
            return;
        }

        ei_printf("Predictions:\n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
    }
}
