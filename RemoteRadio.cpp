/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteRadio.h"
#include "RemoteDisplay.h"
#include <RF24.h>

#define NRF_CE_PIN 2
#define NRF_CS_PIN 3
#define NRF_IRQ_PIN 4

extern TaskManager taskManager;

static RF24 radio(NRF_CE_PIN, NRF_CS_PIN);
static const byte addressCtl[6] = "ctlOD";
static const byte addressData[6] = "dtaOD";
static const char testChannelData[2] = {'P', 0};

void RemoteRadio::initPins() {
  pinMode(NRF_CS_PIN, OUTPUT);
  pinMode(NRF_IRQ_PIN, INPUT);

  digitalWriteFast(NRF_CS_PIN, 1);
}

void RemoteRadio::begin() {
  randomSeed(analogRead(A1) * micros());

  radio.begin();
  if (radio.failureDetected || !radio.isChipConnected()) {
    Serial.println("Radio begin failed");
  } else {
    Serial.println("Radio begin success");
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(0, testPktRetries);

  channel_ = random(0, 125/2) * 2;
  channelConnected_ = false;
  startConnection();
}

void RemoteRadio::testChannelCB(TaskNode* self) {
  if (!testChannelTriesRemaining_) {
    channel_ = newChannelNo();
    radio.setChannel(channel_);
    remoteDisplay.setRadioStatus(String("ch ") + String(channel_));
    remoteDisplay.setRadioStatusColor(ILI9341_ORANGE);
    testChannelTriesRemaining_ = testRetries;
    return;
  }
  radio.startWrite(testChannelData, sizeof(testChannelData), false);
  while(digitalReadFast(NRF_IRQ_PIN) == 1);
  bool tx_ds, tx_df, rx_dr;
  radio.whatHappened(tx_ds, tx_df, rx_dr); // resets the IRQ pin to HIGH
  if (tx_df) {
    radio.flush_tx();// clear all payloads from the TX FIFO
  }
  if (tx_ds) {
    radio.setRetries(1, dataPktRetries);
    radio.openWritingPipe(addressData);
    channelConnected_ = true;
    remoteDisplay.setRadioStatusColor(ILI9341_GREEN);
    taskManager.remove(self, true);
    return;
  }
  testChannelTriesRemaining_--;
}

void RemoteRadio::startConnection() {
  channelConnected_ = false;
  radio.setRetries(0, testPktRetries);
  radio.openWritingPipe(addressCtl);
  remoteDisplay.setRadioStatus(String("ch ") + String(channel_));
  remoteDisplay.setRadioStatusColor(ILI9341_ORANGE);
  radio.setChannel(channel_);
  testChannelTriesRemaining_ = testRetries;
  taskManager.removeById(3);
  taskManager.addBack(taskManager.newPeriodicTask(
    3,
    2,
    [](TaskNode* self, uint32_t) { remoteRadio.testChannelCB(self); }));
}

bool RemoteRadio::txData(const uint8_t *data, uint8_t len) {
  bool tx_ds, tx_df, rx_dr;

  if (!channelConnected_) {
    return false;
  }
  if (len > 32) {
    len = 32;
  }
  radio.startWrite(data, len, false);

  while(digitalReadFast(NRF_IRQ_PIN) == 1);
  radio.whatHappened(tx_ds, tx_df, rx_dr); // resets the IRQ pin to HIGH
  if (tx_df)           // if TX payload failed
    radio.flush_tx();  // clear all payloads from the TX FIFO

  uint8_t quality = radio.getARC();
  bool success = (quality <= dataPktMinArc);
  if (!success) {
    Serial.print("Signal quality low ");
    Serial.print(quality);
    Serial.println(" selecting new channel.");
    remoteDisplay.setRadioStatusColor(ILI9341_RED);
    channelConnected_ = false;
    startAnnounceNewChannel();
  }
  return success;
}

void RemoteRadio::startAnnounceNewChannel() {
  channel_ = random(0, 125/2) * 2;
  Serial.print("Announcing new channel ");
  Serial.print(channel_);
  radio.setRetries(0, announcePktRetries);
  radio.openWritingPipe(addressCtl);
  announceChannelTriesRemaining_ = announceRetries;
  //delay(10);
  taskManager.removeById(4);
  taskManager.addBack(taskManager.newPeriodicTask(
    4,
    5,
    [](TaskNode* self, uint32_t) { remoteRadio.announceChannelCB(self); }));
}

void RemoteRadio::announceChannelCB(TaskNode* self) {
  if (!announceChannelTriesRemaining_) {
    Serial.println(" timeout");
    startConnection();
    taskManager.remove(self, true);
    return;
  }
  char data[3] = {'C', channel_, 0};
  radio.startWrite(data, sizeof(data), false);
  while(digitalReadFast(NRF_IRQ_PIN) == 1);
  bool tx_ds, tx_df, rx_dr;
  radio.whatHappened(tx_ds, tx_df, rx_dr); // resets the IRQ pin to HIGH
  if (tx_df)
    radio.flush_tx();// clear all payloads from the TX FIFO
  if (tx_ds) {
    Serial.println(" success");
    startConnection();
    taskManager.remove(self, true);
    return;
  }
  announceChannelTriesRemaining_--;
}

void RemoteRadio::poll() {
  if (channelConnected_ && (digitalReadFast(NRF_IRQ_PIN) == 0)) {
    if (radio.available()) {
      int len = radio.getDynamicPayloadSize();
      radio.read(ackData_, min(len, (int)sizeof(ackData_)));
      Serial.print("ACK: len = ");
      Serial.println(len);
      //Serial.print(" data:");
      //Serial.println((const char*)ackData_);
    }
  }
}

RemoteRadio remoteRadio;
