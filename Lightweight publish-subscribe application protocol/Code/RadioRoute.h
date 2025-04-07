

#ifndef RADIO_ROUTE_H
#define RADIO_ROUTE_H

typedef nx_struct radio_route_msg {
    nx_uint8_t type; // three types of messages , type 0 ack and connection, type 1 sub messages, type 2 pub messages, type 3 data messages	
    nx_uint8_t source;
    nx_uint8_t destination;
    nx_uint8_t topic_name; // 1 temperature, 2 humidity, 3 luminosity
    nx_uint8_t payload;// the payload of the message	
} radio_route_msg_t;


enum {
  AM_RADIO_COUNT_MSG = 10,
};

#endif

