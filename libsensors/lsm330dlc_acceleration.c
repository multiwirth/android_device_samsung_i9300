/*
 * Copyright (C) 2013 Paul Kocialkowski <contact@paulk.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include <hardware/sensors.h>
#include <hardware/hardware.h>

#define LOG_TAG "smdk4x12_sensors"
#include <utils/Log.h>

#include "smdk4x12_sensors.h"
#include "lsm330dlc_accel.h"

struct lsm330dlc_acceleration_data {
	int64_t delay;
	int device_fd;

	pthread_t thread;
	pthread_mutex_t mutex;
	int thread_continue;
};

void *mAccel;

int lsm330dlc_acceleration_init(struct smdk4x12_sensors_handlers *handlers,
	struct smdk4x12_sensors_device *device)
{
	struct lsm330dlc_acceleration_data *data = NULL;
	pthread_attr_t thread_attr;
	int device_fd = -1;
	int uinput_fd = -1;
	int input_fd = -1;
	int rc;
	int i;

        mAccel = createAccelSensor();

	ALOGE("%s(%p, %p)", __func__, handlers, device);

	if (handlers == NULL || device == NULL)
		return -EINVAL;

	data = (struct lsm330dlc_acceleration_data *) calloc(1, sizeof(struct lsm330dlc_acceleration_data));

	device_fd = open("/dev/accelerometer", O_RDONLY);
	if (device_fd < 0) {
		ALOGE("%s: Unable to open device", __func__);
		goto error;
	}

	input_fd = accelGetDataFd(mAccel);
	if (input_fd < 0) {
		ALOGE("%s: Unable to open acceleration input", __func__);
		goto error;
	}

	data->thread_continue = 1;

	pthread_mutex_init(&data->mutex, NULL);
	pthread_mutex_lock(&data->mutex);

	pthread_attr_init(&thread_attr);
	pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

	data->device_fd = device_fd;
	handlers->poll_fd = input_fd;
	handlers->data = (void *) data;

	return 0;

error:
	if (data != NULL)
		free(data);

	if (input_fd >= 0)
		close(input_fd);

	if (device_fd >= 0)
		close(device_fd);

	handlers->poll_fd = -1;
	handlers->data = NULL;

	return -1;
}

int lsm330dlc_acceleration_deinit(struct smdk4x12_sensors_handlers *handlers)
{
	struct lsm330dlc_acceleration_data *data = NULL;

	deleteAccelSensor(mAccel);

	ALOGE("%s(%p)", __func__, handlers);

	if (handlers == NULL || handlers->data == NULL)
		return -EINVAL;

	data = (struct lsm330dlc_acceleration_data *) handlers->data;

	handlers->activated = 0;
	data->thread_continue = 0;
	pthread_mutex_unlock(&data->mutex);

	pthread_mutex_destroy(&data->mutex);

	if (data->device_fd >= 0)
		close(data->device_fd);
	data->device_fd = -1;

	free(handlers->data);
	handlers->data = NULL;

	return 0;
}

int lsm330dlc_acceleration_activate(struct smdk4x12_sensors_handlers *handlers)
{
	int handle = -1; // unused
	int enable = 1;

	accelEnable(mAccel, handle, enable);
	return 0;
}

int lsm330dlc_acceleration_deactivate(struct smdk4x12_sensors_handlers *handlers)
{
	int handle = -1; // unused
	int enable = 0;

	accelEnable(mAccel, handle, enable);
	return 0;
}

int lsm330dlc_acceleration_set_delay(struct smdk4x12_sensors_handlers *handlers, int64_t delay)
{
	int handle = -1; // unused
	accelSetDelay(mAccel, handle, delay);

	return 0;
}

float lsm330dlc_acceleration_convert(int value)
{
	return (float) (value) * (GRAVITY_EARTH / 1024.0f);
}

extern int mFlushed;

int lsm330dlc_acceleration_get_data(struct smdk4x12_sensors_handlers *handlers,
	struct sensors_event_t *event)
{
	struct lsm330dlc_acceleration_data *data;
	struct input_event input_event;
	int input_fd;
	int rc;
	int sensorId = SENSOR_TYPE_ACCELEROMETER;

//	ALOGE("%s(%p, %p)", __func__, handlers, event);

	if (handlers == NULL || handlers->data == NULL || event == NULL)
		return -EINVAL;

	if (mFlushed & (1 << sensorId)) { /* Send flush META_DATA_FLUSH_COMPLETE immediately */
		sensors_event_t sensor_event;
		memset(&sensor_event, 0, sizeof(sensor_event));
		sensor_event.version = META_DATA_VERSION;
		sensor_event.type = SENSOR_TYPE_META_DATA;
		sensor_event.meta_data.sensor = sensorId;
		sensor_event.meta_data.what = 0;
		*event++ = sensor_event;
		mFlushed &= ~(0x01 << sensorId);
		ALOGE("AkmSensor: %s Flushed sensorId: %d", __func__, sensorId);
	}

	data = (struct lsm330dlc_acceleration_data *) handlers->data;

	input_fd = accelGetDataFd(mAccel);
	if (input_fd < 0) {
		ALOGE("%s: received wrong fd=%d", __func__, input_fd);
		return -1;
	}

	memset(event, 0, sizeof(struct sensors_event_t));
	event->version = sizeof(struct sensors_event_t);
	event->sensor = handlers->handle;
	event->type = handlers->handle;

	event->acceleration.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	do {
		rc = read(input_fd, &input_event, sizeof(input_event));
		if (rc < (int) sizeof(input_event))
			break;

		if (input_event.type == EV_REL) {
			switch (input_event.code) {
				case REL_X:
					event->acceleration.x = lsm330dlc_acceleration_convert(input_event.value);
					break;
				case REL_Y:
					event->acceleration.y = lsm330dlc_acceleration_convert(input_event.value);
					break;
				case REL_Z:
					event->acceleration.z = lsm330dlc_acceleration_convert(input_event.value);
					break;
				default:
					continue;
			}
		} else if (input_event.type == EV_SYN) {
			if (input_event.code == SYN_REPORT)
				event->timestamp = input_timestamp(&input_event);
		}
	} while (input_event.type != EV_SYN);

	return 0;
}

struct smdk4x12_sensors_handlers lsm330dlc_acceleration = {
	.name = "LSM330DLC Acceleration",
	.handle = SENSOR_TYPE_ACCELEROMETER,
	.init = lsm330dlc_acceleration_init,
	.deinit = lsm330dlc_acceleration_deinit,
	.activate = lsm330dlc_acceleration_activate,
	.deactivate = lsm330dlc_acceleration_deactivate,
	.set_delay = lsm330dlc_acceleration_set_delay,
	.get_data = lsm330dlc_acceleration_get_data,
	.activated = 0,
	.needed = 0,
	.poll_fd = -1,
	.data = NULL,
};
