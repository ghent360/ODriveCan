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

#include "nrf24l01.h"

namespace nrf24 {

Nrf24l01::SpiMaster::SpiMaster(SPIClass* spi, uint16_t cs)
    : spi_(spi), cs_(cs) {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  spi_->begin();
}

uint8_t Nrf24l01::SpiMaster::Command(
    uint8_t command,
    const uint8_t* data_in, uint8_t data_in_size,
    uint8_t* data_out = nullptr, uint8_t data_out_size = 0) {
  if (data_in_size > 32 || data_out_size > 32) return -1;
  spi_->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);

  uint8_t size = max(data_in_size, data_out_size);
  tx_buf_[0] = command;
  if (data_in) {
    memcpy(tx_buf_ + 1, data_in, data_in_size);
  }
  if (size > data_in_size) {
    memset(tx_buf_ + 1 + data_in_size, 0, size - data_in_size);
  }

  spi_->transfer(tx_buf_, rx_buf_, size + 1);
  uint8_t status = rx_buf_[0];
  if (data_out) {
    memcpy(data_out, rx_buf_ + 1, data_out_size);
  }

  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
  return status;
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(
  uint8_t address, const uint8_t* data_in, uint8_t data_in_size) {
  return Command(0x20 + address, data_in, data_in_size);
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(uint8_t address, uint8_t data) {
  return WriteRegister(address, &data, 1);
}

uint8_t Nrf24l01::SpiMaster::ReadRegister(
    uint8_t address, uint8_t *data_out, uint8_t data_out_size) {
  return Command(0x00 + address, nullptr, 0, data_out, data_out_size);
}

uint8_t Nrf24l01::SpiMaster::ReadRegister(uint8_t address) {
  uint8_t result = 0;
  ReadRegister(address, &result, 1);
  return result;
}

bool Nrf24l01::SpiMaster::VerifyRegister(
  uint8_t address, const uint8_t* data_in, uint8_t data_in_size) {
  if (data_in_size > 32) return false;
  constexpr int kMaxRetries = 2;

  for (int i = 0; i < kMaxRetries; i++) {
    WriteRegister(address, data_in, data_in_size);
    ReadRegister(address, nullptr, data_in_size);
    if (memcmp(data_in, rx_buf_ + 1, data_in_size) == 0) {
      return true;
    }
  }

  // Well, we failed.  Report an error.
  return false;
}

bool Nrf24l01::SpiMaster::VerifyRegister(uint8_t address, uint8_t value) {
  return VerifyRegister(address, &value, 1);
}

Nrf24l01::Nrf24l01(const Options& options)
    : options_(options),
      spi_(&SPI),
      nrf_(spi_, options.pins.cs),
      irq_(options.pins.irq),
      ce_(options.pins.ce) {
  pinMode(ce_, OUTPUT);
  digitalWrite(ce_, LOW);
}

Nrf24l01::~Nrf24l01() {}

void Nrf24l01::Poll() {
  if (digitalRead(irq_) == 0) {
    // We have some interrupt to deal with.  Read the status.
    const uint8_t status = nrf_.Command(0xff, nullptr, 0);

    if (status & (1 << 6)) {
      ReadPacket();

      if (is_data_ready_) { rx_overflow_ = true; }
      is_data_ready_ = true;
    }
    if (status & (1 << 4)) {
      // Retransmit count exceeded!
      retransmit_exceeded_++;

      // Flush our TX FIFO.
      nrf_.Command(0xe1, nullptr, 0);
    }

    const uint8_t maybe_to_clear = status & 0x70;
    if (maybe_to_clear) {
      // Yes!
      nrf_.WriteRegister(0x07, maybe_to_clear);
    }
  }
}

void Nrf24l01::PollMillisecond(uint32_t now) {
  // The NRF isn't turned on for 100ms after power up.
  switch (configure_state_) {
    case kPowerOnReset: {
      // While we are in power on reset, leave CE off.
      digitalWrite(ce_, LOW);

      // This check can be absolute, because the device only has to
      // do power on reset once.
      if (now < 150) { return; }

      WriteConfig();
      configure_state_ = kEnteringStandby;
      start_entering_standby_ = now;
      return;
    }
    case kEnteringStandby: {
      if ((now - start_entering_standby_) < 2) { return; }

      Configure();
      configure_state_ = kStandby;
      return;
    }
    case kStandby: {
      break;
    }
  }
}

bool Nrf24l01::ready() const {
  return configure_state_ == kStandby;
}

void Nrf24l01::SelectRfChannel(uint8_t channel) {
  //MJ_ASSERT(channel < 125);
  if (options_.ptx == 0) {
    // To reliably change the frequency, the receiver needs to be
    // disabled.  It seems to kinda work only for a few limited
    // channels without doing this.
    digitalWrite(ce_, LOW);
  }
  VerifyRegister(0x05, channel & 0x7f);  // RF_CH
  if (options_.ptx == 0) {
    digitalWrite(ce_, HIGH);
  }
}

bool Nrf24l01::is_data_ready() {
  return is_data_ready_;
}

bool Nrf24l01::Read(Packet* packet)  {
  if (!is_data_ready_) {
    packet->size = 0;
    return false;
  }
  *packet = rx_packet_;
  rx_packet_.size = 0;

  // Check to see if there is more remaining.
  const auto status_reg = nrf_.Command(0xff, nullptr, 0);
  is_data_ready_ = ((status_reg >> 1) & 0x07) != 0x07;
  if (is_data_ready_) {
    ReadPacket();
  }

  return true;
}

void Nrf24l01::Transmit(const Packet* packet) {
  //MJ_ASSERT(options_.ptx == 1);
  nrf_.Command(0xa0, &packet->data[0], packet->size);
  // Strobe CE to start this transmit.
  digitalWrite(ce_, HIGH);
  delayMicroseconds(10);
  digitalWrite(ce_, LOW);
}

void Nrf24l01::QueueAck(const Packet* packet) {
  // We always use PPP == 0
  nrf_.Command(0xa8, &packet->data[0], packet->size);
}

void Nrf24l01::ReadPacket() {
  uint8_t payload_width = 0;
  nrf_.Command(0x60,  // R_RX_PL_WID
               nullptr, 0,
               &payload_width, 1);

  rx_packet_.size = payload_width;
  if (payload_width) {
    nrf_.Command(0x61, nullptr, 0,
                 &rx_packet_.data[0],
                 min(payload_width, sizeof(rx_packet_.data)));
  }
}

void Nrf24l01::VerifyRegister(
  uint8_t address, const uint8_t *data_in, uint8_t data_in_size) {
  if (!nrf_.VerifyRegister(address, data_in, data_in_size) && error_ == 0) {
    // Just report the first error.
    error_ = 0x100 | address;
  }
}

void Nrf24l01::VerifyRegister(uint8_t address, uint8_t value) {
  if (!nrf_.VerifyRegister(address, value) && error_ == 0) {
    error_ = 0x100 | address;
  }
}

void Nrf24l01::WriteConfig() {
  nrf_.WriteRegister(0x00, GetConfig());  // CONFIG
  // Now we need to wait another 1.5ms to enter standby mode for this
  // to take effect.
}

void Nrf24l01::Configure() {
  VerifyRegister(0x00, GetConfig());

  VerifyRegister(
      0x01, // EN_AA - enable auto-acknowledge per rx channel
      (options_.automatic_acknowledgment ? 0x01 : 0x00));
  VerifyRegister(
      0x02, // EN_RXADDR
      (options_.ptx == 0 || options_.automatic_acknowledgment) ?
      0x01 : 0);  // EN_RXADDR enable 0
  VerifyRegister(
      0x03,  // SETUP_AW
      [&]() {
        if (options_.address_length == 3) { return 1; }
        if (options_.address_length == 4) { return 2; }
        if (options_.address_length == 5) { return 3; }
        // default to length 5 address
        return 3;
      }());
  VerifyRegister(
      0x04,  // SETUP_RETR
      min(15, options_.auto_retransmit_delay_us / 250) << 4 |
      min(15, options_.auto_retransmit_count));

  SelectRfChannel(options_.initial_channel);

  VerifyRegister(
      0x06,  // RF_SETUP
      [&]() {
        if (options_.data_rate == 250000) {
          return (1 << 5);
        } else if (options_.data_rate == 1000000) {
          return (0 << 5) | (0 << 3);
        } else if (options_.data_rate == 2000000) {
          return (0 << 5) | (1 << 3);
        }
        // default to 250kbps
        return (1 << 5);
      }() |
      [&]() {
        if (options_.output_power == -18) {
          return 0;
        } else if (options_.output_power == -12) {
          return 2;
        } else if (options_.output_power == -6) {
          return 4;
        } else if (options_.output_power == 0) {
          return 6;
        }
        // default to 0dB output power
        return 6;
      }());

  SelectId(options_.id);
  VerifyRegister(
      0x1c,
      (options_.dynamic_payload_length  ||
       options_.automatic_acknowledgment) ? 1 : 0);  // DYNPD
  VerifyRegister(
      0x1d,  0 // FEATURE
      | (((options_.dynamic_payload_length ||
           options_.automatic_acknowledgment) ? 1 : 0) << 2) // EN_DPL
      | ((options_.automatic_acknowledgment ? 1 : 0) << 1) // EN_ACK_PAY
      | ((options_.automatic_acknowledgment ? 1 : 0 ) << 0) // EN_DYN_ACK
  );

  // In read mode, we leave CE high.
  if (options_.ptx == 0) {
    digitalWrite(ce_, HIGH);
  }
}

void Nrf24l01::SelectId(uint64_t id) {
  uint8_t id_buf[5];
  for (int i = 0; i < options_.address_length; i++) {
    id_buf[i] = (id >> (i * 8)) & 0xff;
  }
  VerifyRegister(0x0a, id_buf, sizeof(id_buf)); // RX_ADDR_P0
  VerifyRegister(0x10, id_buf, sizeof(id_buf)); // TX_ADDR
}

uint8_t Nrf24l01::GetConfig() const {
  return 0
      | (0 << 6) // MASK_RX_DR - enable RX_DR interrupt
      | (0 << 5) // MASK_TX_DS - enable TX_DS interrupt
      | (0 << 4) // MASK_MAX_RT - enable MAX_RT interrupt
      | ((options_.enable_crc ? 1 : 0) << 3) // EN_CRC
      | (((options_.crc_length == 2) ? 1 : 0) << 2) // CRCO (0=1 byte, 1=2 bytes)
      | (1 << 1) // PWR_UP
      | ((options_.ptx ? 0 : 1) << 0) // PRIM_RX
      ;
}

Nrf24l01::Status Nrf24l01::status() {
  Status result;
  result.status_reg = nrf_.Command(0xff, nullptr, 0);
  result.retransmit_exceeded = retransmit_exceeded_;
  return result;
}

uint8_t Nrf24l01::ReadRegister(uint8_t reg) {
  return nrf_.ReadRegister(reg);
}

void Nrf24l01::ReadRegister(
  uint8_t reg, uint8_t *data_out, uint8_t data_out_size) {
  nrf_.ReadRegister(reg, data_out, data_out_size);
}

void Nrf24l01::WriteRegister(
  uint8_t reg, const uint8_t *data_in, uint8_t data_in_size) {
  nrf_.WriteRegister(reg, data_in, data_in_size);
}

}  // namespace nrf24
