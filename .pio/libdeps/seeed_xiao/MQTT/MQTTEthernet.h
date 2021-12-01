#if !defined(MQTTETHERNET_H)
#define MQTTETHERNET_H

#include "MQTT_mbed.h"
#include "MQTTSocket.h"
#include "../W5500Interface/EthernetInterface.h"

class MQTTEthernet : public MQTTSocket
{
public:    
    MQTTEthernet(SPI* spi, PinName cs, PinName reset): eth(spi, cs, reset)
    {
        wait(1);
        this->createSocket();
        eth.init();
       eth.connect();
    }
    
    EthernetInterface& getEth()
    {
        return eth;
    }
    
    void reconnect()
    {
        eth.connect();  // nothing I've tried actually works to reconnect 
    }
    
private:
    EthernetInterface eth;
    
};

#endif
