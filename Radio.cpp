#include "Radio.h"
#include <stdint.h>
#include <RF24.h>

static RF24 radio(NRF24_CE_PIN, NRF24_CS_PIN);
static const byte readAddress[6] = "OD3tg";
static const byte remoteAddress[6] = "OD3rm";
static const uint8_t openDogChannel = 42;

bool Radio::initRadio() {
  radio.begin();
  if (radio.failureDetected || !radio.isChipConnected()) {
    return false;
  }
  radio.openWritingPipe(remoteAddress);  // Pipe to remote
  radio.openReadingPipe(1, readAddress); // Our reading pipe
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(openDogChannel);
  //radio.setAutoAck(true);
  radio.startListening();

  radio.printPrettyDetails();
  return true;
}

void Radio::powerDown() {
  radio.powerDown();
}
