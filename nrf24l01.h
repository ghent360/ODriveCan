// Copyright 2020 Josh Pieper, jjp@pobox.com.
// Modifications Copyright 2022 Ghent The slicer, ghent360@iqury.us
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>
#include <cstddef>
#include <SPI.h>

namespace nrf24 {

class Nrf24l01 {
 public:
  struct Pins {
    ////////////////////
    // Pin configuration
    uint16_t cs = -1;
    uint16_t irq = -1;
    uint16_t ce = -1;
  };

  struct Options {
    Pins pins;

    ///////////////////////
    // Device configuration

    bool ptx = true;  // if false, then PRX mode
    int address_length = 5;
    uint64_t id = 0;
    bool dynamic_payload_length = true;
    bool enable_crc = true;
    int crc_length = 2;
    int auto_retransmit_count = 0;
    int auto_retransmit_delay_us = 1000;
    bool automatic_acknowledgment = false;
    int initial_channel = 2;

    // Can be one of 250000, 1000000, or 2000000;
    int data_rate = 1000000;

    // Can be one of -18, -12, -6, 0.
    int output_power = 0;

    Options() {}
  };

  Nrf24l01(const Options& = Options());
  ~Nrf24l01();

  void Poll();

  void PollMillisecond(uint32_t);

  /// Return true if the device has completed its power on cycle.
  bool ready() const;

  /// Switch to one of the possible 125 channels (0-124)
  void SelectRfChannel(uint8_t);

  /// Switch to a different shockburst ID.
  void SelectId(uint64_t id);

  /// Return true if there is data available to read.
  bool is_data_ready();

  struct Status {
    uint8_t status_reg = 0;
    uint32_t retransmit_exceeded = 0;
  };
  Status status();

  struct Packet {
    uint8_t size = 0;
    uint8_t data[32] = {};
  };

  /// Read the next available data packet.  @return false if no data
  /// was available.
  bool Read(Packet*);

  /// Transmit a packet.
  void Transmit(const Packet*);

  /// Queue the given packet to be sent as the next auto
  /// acknowledgement.  This can only be called if Options::ptx == false
  void QueueAck(const Packet*);

  uint8_t ReadRegister(uint8_t);
  void ReadRegister(uint8_t, uint8_t *data_out, uint8_t data_out_size);

  void WriteRegister(uint8_t, const uint8_t *data_in, uint8_t data_in_size);

  uint32_t error() const { return error_; }

 private:
  void ReadPacket();
  void VerifyRegister(uint8_t address, const uint8_t *data_in, uint8_t data_in_size);
  void VerifyRegister(uint8_t address, uint8_t value);

  void WriteConfig();
  void Configure();
  uint8_t GetConfig() const;

  const Options options_;

  SPIClass* spi_;

  class SpiMaster {
   public:
    SpiMaster(SPIClass*, uint16_t cs);
    uint8_t Command(uint8_t command,
                    const uint8_t *data_in, uint8_t data_in_size,
                    uint8_t *data_out, uint8_t data_out_size);
    uint8_t WriteRegister(
      uint8_t address, const uint8_t *data_in, uint8_t data_in_size);
    uint8_t WriteRegister(uint8_t address, uint8_t data);

    uint8_t ReadRegister(
      uint8_t address, uint8_t *data_out, uint8_t data_out_size);
    uint8_t ReadRegister(uint8_t address);
    bool VerifyRegister(
      uint8_t address, const uint8_t *data_in, uint8_t data_in_size);
    bool VerifyRegister(uint8_t address, uint8_t value);

   private:
    SPIClass* spi_;
    uint16_t cs_;
    uint8_t tx_buf_[33];
    uint8_t rx_buf_[33];
  };

  SpiMaster nrf_;

  uint16_t irq_;
  uint16_t ce_;

  enum ConfigureState {
    kPowerOnReset,
    kEnteringStandby,
    kStandby,
  };
  ConfigureState configure_state_ = kPowerOnReset;
  uint32_t start_entering_standby_ = 0;

  bool is_data_ready_ = false;
  bool rx_overflow_ = false;

  uint32_t retransmit_exceeded_ = 0;
  Packet rx_packet_;

  uint32_t error_ = 0;
};

} // namespace nrf24
