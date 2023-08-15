#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
int main(int argc, char **argv)
{

    if (argc < 2)
    {
    	printf("Usage: k4a_custom_track output.mkv\n");
    	return 1;
    }
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        printf("No k4a devices attached!\n");
        return 1;
    }

    k4a_device_t device = NULL; //open connected device
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
    {
        printf("Failed to open k4a device!\n");
        return 1;
    }

    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size); //size of serial num

    char *serial = (char*)(malloc(serial_size)); //allocate mem for serial num
    k4a_device_get_serialnum(device, serial, &serial_size); //get that mem
    printf("Opened device: %s\n", serial);
    free(serial);

    // Configure a stream
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps       = K4A_FRAMES_PER_SECOND_15;
    config.color_format     = K4A_IMAGE_FORMAT_COLOR_MJPG;
    //config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode 	    = K4A_DEPTH_MODE_NFOV_UNBINNED;
    
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras!\n");
        k4a_device_close(device);
    }
    
    char *path = argv[1];
    k4a_record_t record = NULL;
    if (K4A_FAILED(k4a_record_create(path, device, config, &record))) 
    {
    	printf("Unable to create recording file: %s\n", path);
    	return 1;
    }
    
    k4a_calibration_t sensor_calibration;
    k4a_device_get_calibration(device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration);
    uint32_t depth_width = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_width;
    uint32_t depth_height = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_height;
    
    k4a_record_video_settings_t video_settings;
    video_settings.width = depth_width;
    video_settings.height = depth_height;
    video_settings.frame_rate = 15; //same as camera_fps
    
    for (int frame = 0; frame < 100; frame++) //read 100 depth frames from cam
    {
    	k4a_capture_t capture;
    	k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
    	if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) 
    	{
    		k4a_record_write_capture(record, capture); //write capture to built-in tracks
    		k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
    		if (depth_image)
    		{
    			uint8_t *depth_buffer = k4a_image_get_buffer(depth_image);
    			size_t depth_buffer_size = k4a_image_get_size(depth_image);
    			for (size_t i = 0; i < depth_buffer_size; i += 2) 
    			{
    				uint16_t depth = (uint16_t)(depth_buffer[i + 1] << 8 | depth_buffer[i]);
    				if (depth > 0x3FF)
    				{
    					depth_buffer[i] = 0xFF;
    				}
    				else
    				{
    					depth_buffer[i] = (uint8_t)(depth >> 2);
    				}
    				depth_buffer[i + 1] = 128;
    			}
    			k4a_image_release(depth_image);
    		}
    		k4a_capture_release(capture);
    	}
    	else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    	{
    		printf("k4a_device_get_capture() timed out\n");
    		k4a_device_close(device);
    	}
    	else
    	{
    		printf("k4a_device_get_capture() returned error: %d\n", get_capture_result);
    		k4a_device_close(device);
    	}
    }
    
    // Shut down the camera when finished with application logic
    k4a_device_stop_cameras(device);
    k4a_record_flush(record);
    k4a_record_close(record);
    k4a_device_close(device);
}
