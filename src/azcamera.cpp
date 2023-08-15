
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.h>

int main()
{
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        printf("No k4a devices attached!\n");
        return 1;
    }

    // Open the first plugged in Kinect device
    k4a_device_t device = NULL;
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
    {
        printf("Failed to open k4a device!\n");
        return 1;
    }

    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    printf("Opened device: %s\n", serial);
    free(serial);

    // Configure a stream
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps       = K4A_FRAMES_PER_SECOND_15;
    config.color_format     = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode 	    = K4A_DEPTH_MODE_OFF;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras!\n");
        k4a_device_close(device);
    }
    
    // Capture a depth frame
    k4a_capture_t capture = NULL;
switch (k4a_device_get_capture(device, &capture, 1000))
{
	case K4A_WAIT_RESULT_SUCCEEDED:
    		break;
	case K4A_WAIT_RESULT_TIMEOUT:
    		printf("Timed out waiting for a capture\n");
    		break;
	case K4A_WAIT_RESULT_FAILED:
    		printf("Failed to read a capture\n");
	}
	
	printf("Captured");

        // Probe for a depth image (only captures depth & ir images)
        k4a_image_t image;
        
        image = k4a_capture_get_color_image(capture);
        if (image)
        {
            printf(" | Color res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));
            k4a_image_release(image);
        }
        else
        {
            printf(" | Color None ");
        }
        
        image = k4a_capture_get_ir_image(capture);
        if (image != NULL)
        {
            printf(" | IR res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));
            k4a_image_release(image);
        }

    // Shut down the camera when finished with application logic
    k4a_capture_release(capture);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}
