#ifndef SENSOR_H
#define SENSOR_H

namespace ForceSensors
{
    void setup(void);
    void loop(void);

    // Controllers
    void sendEnabledState(char *buff, int count);
    void setEnabledState(char *buff, int count);
}

#endif // FORCE_SENSORS_H
