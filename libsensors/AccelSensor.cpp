/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/select.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/uinput.h>
#include <cutils/log.h>
#include <cstring>


#include "AccelSensor.h"

#define LOGTAG "AccelerometerSensor"

// ioctls
#define LSM330DLC_ACCEL_IOCTL_BASE 'a'
#define LSM330DLC_ACCEL_IOCTL_SET_ENABLE   \
	_IOW(LSM330DLC_ACCEL_IOCTL_BASE, 9, int)


/*****************************************************************************/
AccelSensor::AccelSensor()
    : SensorBase("/dev/acceleration", "accelerometer_sensor"),
    mEnabled(0),
    mInputReader(4),
    mHasPendingEvent(false)
{

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_A;
    mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd) {
        strcpy(input_sysfs_path, "/sys/class/input/");
        strcat(input_sysfs_path, input_name);
        strcat(input_sysfs_path, "/device/");
        input_sysfs_path_len = strlen(input_sysfs_path);
    }
}

AccelSensor::~AccelSensor() {

    //  ALOGD("AccelSensor::~AccelSensor()");
    if (mEnabled) {
        enable(0, 0);
    }
}

int AccelSensor::setInitialState() {
    return 0;
}

int AccelSensor::enable(int32_t handle, int en) {
    int flags = en ? 1 : 0;
    int fd;
    if (flags != mEnabled) {
        strcpy(&input_sysfs_path[input_sysfs_path_len], "enable");
        fd = open(input_sysfs_path, O_RDWR);
        if (fd >= 0) {
            write(fd, en == 1 ? "1" : "0", 2);
            close(fd);
            mEnabled = flags;
            setInitialState();
            return 0;
        }
        return -1;
    }
    return 0;
}


bool AccelSensor::hasPendingEvents() const {
    /* FIXME probably here should be returning mEnabled but instead
       mHasPendingEvents. It does not work, so we cheat.*/
    //ALOGD("AccelSensor::~hasPendingEvents %d", mHasPendingEvent ? 1 : 0 );
    return mHasPendingEvent;
}


int AccelSensor::setDelay(int32_t handle, int64_t ns)
{
    int fd;

    if (ns < 10000000) {
        ns = 10000000; // Minimum on stock
    }

    strcpy(&input_sysfs_path[input_sysfs_path_len], "poll_delay");
    fd = open(input_sysfs_path, O_RDWR);
    if (fd >= 0) {
        char buf[80];
        sprintf(buf, "%lld", ns);
        write(fd, buf, strlen(buf)+1);
        close(fd);
        return 0;
    }
    return -1;
}


int AccelSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;
    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            float value = event->value;
            if (event->code == EVENT_TYPE_ACCEL_X) {
                mPendingEvent.acceleration.x = value * CONVERT_A_X;
            } else if (event->code == EVENT_TYPE_ACCEL_Y) {
                mPendingEvent.acceleration.y = value * CONVERT_A_Y;
            } else if (event->code == EVENT_TYPE_ACCEL_Z) {
                mPendingEvent.acceleration.z = value * CONVERT_A_Z;
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
        } else {
            ALOGE("%s: unknown event (type=%d, code=%d)", LOGTAG,
                    type, event->code);
        }

        mInputReader.next();
    }
    return numEventReceived++;

}

/*****************************************************************************/
extern "C" {

enum {
    accel           = 0,
    numSensorDrivers,
    numFds,
};

int wake = 1; // wake = numFds - 1;

struct pollfd pollFds[numFds];
int wakeFds[2];
int writePipeFd;

#define WAKE_MESSAGE 'W'

void *createAccelSensor(void) {
    AccelSensor *accelSensor = new AccelSensor();
    pollFds[accel].fd = accelSensor->getFd();
    pollFds[accel].events = POLLIN;
    pollFds[accel].revents = 0;

    int result = pipe(wakeFds);
    ALOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
    writePipeFd = wakeFds[1];

    pollFds[wake].fd = wakeFds[0];
    pollFds[wake].events = POLLIN;
    pollFds[wake].revents = 0;
    return reinterpret_cast<void*>(accelSensor);
}

void deleteAccelSensor(void *sensorPtr)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);
    delete accelSensor;
    close(pollFds[wake].fd);
    close(writePipeFd);
}

int accelPollEvents(void *sensorPtr, sensors_event_t* data, int count)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);

    int nbEvents = 0;
    int n = 0;

    do {
        // see if we have some leftover from the last poll()
        SensorBase* const sensor(accelSensor);
        if ((pollFds[accel].revents & POLLIN) || (sensor->hasPendingEvents())) {
            int nb = sensor->readEvents(data, count);
            if (nb < count) {
                // no more data for this sensor
                pollFds[accel].revents = 0;
            }
            count -= nb;
            nbEvents += nb;
            data += nb;
        }

        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            n = poll(pollFds, numFds, nbEvents ? 0 : -1);
            if (n<0) {
                ALOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (pollFds[wake].revents & POLLIN) {
                char msg;
                int result = read(pollFds[wake].fd, &msg, 1);
                ALOGE_IF(result<0, "error reading from wake pipe (%s)", strerror(errno));
                ALOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
                pollFds[wake].revents = 0;
            }
        }
        // if we have events and space, go read them
    } while (n && count);

    return nbEvents;
}

int accelEnable(void *sensorPtr, int32_t handle, int en)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);

    int err = accelSensor->enable(handle, en);

    if (en && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(writePipeFd, &wakeMessage, 1);
        ALOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int accelSetDelay(void *sensorPtr, int32_t handle, int64_t ns)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);
    return accelSensor->setDelay(handle, ns);
}

int accelReadEvents(void *sensorPtr, sensors_event_t* data, int count)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);
    return accelSensor->readEvents(data, count);
}


int accelGetDataFd(void *sensorPtr)
{
    AccelSensor *accelSensor = reinterpret_cast<AccelSensor *>(sensorPtr);
    return accelSensor->data_fd;
}

} // extern "C"
