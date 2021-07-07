/********************************************//**
* This example demonstrates streaming basic data from the sensor
* This is compatible with all sensors plugged in via USB or Bluetooth
* Will not work with the dongle or wireless sensor wirelessly see
* streaming_information_wireless for a wireless example.
***********************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "yost_3_space_api/threespace_api_export.h"
// #include "threespace_api_export.h"

int main()
{
	TSS_ComPort port;
#ifdef WIN32
	port.port_name = malloc(64);
#endif
	tss_device_id device_id;
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Device from Search====\n");
	// tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	// error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
	//if (error == TSS_NO_ERROR)
	{
		error = tss_createSensor(/*port.port_name*/ "/dev/ttyACM0", &device_id);

		if (error)
		{
			printf("Failed to create TSS Sensor on %s!\n", port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
			return 0;
		}
		else
		{

			// U32 set_axis_timestamp = 0;
            // tss_sensor_setAxisDirections(device_id, 0x01, &set_axis_timestamp);

            U8 axis_directions = 0;
            U32 get_axis_timestamp = 0;
            tss_sensor_getAxisDirections(device_id, &axis_directions, &get_axis_timestamp);

            U8 calibration_mode = 0;
            U32 get_calibration_mode_timestamp = 0;
            tss_sensor_getCalibrationMode(device_id, &calibration_mode, &get_calibration_mode_timestamp);

            U8 reference_vector_mode = 0;
            U32 get_reference_vector_mode_timestamp = 0;
            tss_sensor_getReferenceVectorMode(device_id, &reference_vector_mode, &get_reference_vector_mode_timestamp);

            U32 set_reference_vector_mode_timestamp = 0;
            tss_sensor_setReferenceVectorMode(device_id, 3, &set_reference_vector_mode_timestamp);

            printf("====Starting Streaming====\n");

            error = tss_sensor_startStreamingWired(device_id,
                                                   TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION |
                                                   TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES |
                                                   TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES |
                                                   TSS_STREAM_CORRECTED_ACCELEROMETER_DATA |
                                                   TSS_STREAM_CORRECTED_GYROSCOPE_DATA |
                                                   TSS_STREAM_CORRECTED_MAGNETOMETER_DATA,
                                                   1000, TSS_STREAM_DURATION_INFINITE, 0);
            (void) error;
			TSS_Stream_Packet packet;
			for (int i = 0; i < 10; i++)
			{
				printf("Press Enter to get next packet.\n");
				getchar();
				error = tss_sensor_getLastStreamingPacket(device_id, &packet);
				printf("Quaternion: (%.03f,%.03f,%.03f,%.03f)\n", packet.taredOrientQuat[0], packet.taredOrientQuat[1], packet.taredOrientQuat[2], packet.taredOrientQuat[3]);
                printf("Euler (tared): (%.03f,%.03f,%.03f)\n", packet.taredOrientEuler[0], packet.taredOrientEuler[1], packet.taredOrientEuler[2]);
                printf("Euler (untared): (%.03f,%.03f,%.03f)\n", packet.untaredOrientEuler[0], packet.untaredOrientEuler[1], packet.untaredOrientEuler[2]);
                printf("Gyro: (%.03f,%.03f,%.03f)\n", packet.correctedGyroscopeData[0], packet.correctedGyroscopeData[1], packet.correctedGyroscopeData[2]);
                printf("Accel: (%.03f,%.03f,%.03f)\n", packet.correctedAccelerometerData[0], packet.correctedAccelerometerData[1], packet.correctedAccelerometerData[2]);
                printf("Mag: (%.03f,%.03f,%.03f)\n", packet.correctedMagnetometerData[0], packet.correctedMagnetometerData[1], packet.correctedMagnetometerData[2]);

			}

			tss_sensor_stopStreamingWired(device_id);

			tss_removeSensor(device_id);
		}
	}
#if 0
	else
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		printf("Finished press Enter to continue");
		getchar();
		return 0;
	}
#endif
	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();
	return 1;
}