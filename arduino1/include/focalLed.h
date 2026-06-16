#ifndef FOCALLED_H
#define FOCALLED_H

namespace FocalLed
{
    void setup();
    void sendSPI(char *buff, int count);
    void disable(char *buff, int count);
}

#endif // FOCALLED_H
