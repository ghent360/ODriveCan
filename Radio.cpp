#include "Radio.h"
#include "Display.h"
#include "globals.h"
#include <stdint.h>
#include <RF24.h>

#define NRF24_CE_PIN  4
#define NRF24_IRQ_PIN 5
#define NRF24_CS_PIN  6

static RF24 nrf24radio(NRF24_CE_PIN, NRF24_CS_PIN);
static const byte addressCtl[6] = "ctlOD";
static const byte addressData[6] = "dtaOD";

static uint8_t newChannelNo() {
  return random(0, 125 / 2) * 2;
}

static void switchChannel(uint8_t channel) {
  display.setRadioStatus(String("ch ") + String(channel));
  nrf24radio.setChannel(channel);
  nrf24radio.startListening();
}

Radio::Radio()
  : init_ok_(false), last_received_ts_(0), rx_timeout_ms_(1000) {}

void Radio::initRadio() {
  pinMode(NRF24_IRQ_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(5), isrCallbackFunction, FALLING);

  nrf24radio.begin();
  if (nrf24radio.failureDetected || !nrf24radio.isChipConnected()) {
    display.setRadioStatus("Radio init failed");
    init_ok_ = false;
    return;
  }
  nrf24radio.setAddressWidth(5);
  nrf24radio.openReadingPipe(0, addressCtl);
  nrf24radio.openReadingPipe(1, addressData);
  nrf24radio.setPALevel(RF24_PA_MAX);
  nrf24radio.setDataRate(RF24_2MBPS);
  nrf24radio.setCRCLength(RF24_CRC_16);
  nrf24radio.setAutoAck(true);
  nrf24radio.enableAckPayload();
  nrf24radio.enableDynamicPayloads();
  switchChannel(newChannelNo());

  rx_timeout_ms_ = 5000;
  init_ok_ = true;
}

void Radio::powerDown() {
  if (init_ok_)
    nrf24radio.powerDown();
}

void Radio::poll() {
  if (init_ok_ && digitalReadFast(NRF24_IRQ_PIN) == 0) {
    uint8_t pipe_no;
    if (nrf24radio.available(&pipe_no)) {
      last_received_ts2_ = last_received_ts_ = millis();
      display.setRadioStatusColor(ST7735_GREEN);
      uint8_t len = nrf24radio.getDynamicPayloadSize();
      nrf24radio.read(rx_data_, min(len, sizeof(rx_data_)));
      if (pipe_no == 0) {
        if (rx_data_[0] == 'C' && len > 1)
          switchChannel(rx_data_[1]);
      } else if (pipe_no == 1) {
        if (cb_) {
          cb_(rx_data_, len);
        }
      }
    }
  }
}

void Radio::poll10ms(uint32_t timeNow) {
  if (init_ok_) {
    if ((timeNow - last_received_ts_) > rx_timeout_ms_ / 3) {
      display.setRadioStatusColor(ST7735_YELLOW);
    }
    if ((timeNow - last_received_ts_) > rx_timeout_ms_) {
      display.setRadioStatusColor(ST7735_RED);
    }
    if ((timeNow - last_received_ts2_) > rx_timeout_ms_) {
      switchChannel(newChannelNo());
      last_received_ts2_ = timeNow;
    }
  }
}

bool Radio::writeTxData(const uint8_t* data, uint8_t len) {
  if (init_ok_)
    return nrf24radio.writeAckPayload(1, data, len);
  return false;
}

Radio radio;