/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include "../Bitstream.hpp"

#include "doctest.h"

TEST_SUITE("BitstreamReader tests")
{
    TEST_CASE("read bit")
    {
        uint8_t data[4] = {0xa5, 0, 0, 0};
        BitstreamReader br(data, sizeof(data));

        CHECK(br.error() == false);
        CHECK(br.readBit() == 1);
        CHECK(br.error() == false);
        CHECK(br.readBit() == 0);
        CHECK(br.readBit() == 1);
        CHECK(br.readBit() == 0);
        CHECK(br.readBit() == 0);
        CHECK(br.readBit() == 1);
        CHECK(br.readBit() == 0);
        CHECK(br.readBit() == 1);
        CHECK(br.error() == false);
        for(int i = 0; i < 24; i++) {
            CHECK(br.readBit() == 0);
            CHECK(br.error() == false);
        }
        CHECK(br.readBit() == 0);
        CHECK(br.error() == true);
    }

    TEST_CASE("read bits 32")
    {
        uint8_t data[4] = {0xa5, 0, 0, 0};
        BitstreamReader br1(data, sizeof(data));

        CHECK(br1.readBits(32) == 0xa5000000);
        CHECK(br1.error() == false);
        CHECK(br1.readBits(5) == 0);
        CHECK(br1.error() == true);

        BitstreamReader br2(data, sizeof(data));

        CHECK(br2.readBits(5) == 0x14); // 0b10100
        CHECK(br2.error() == false);
        CHECK(br2.readBits(32) == 0xa0000000);
        CHECK(br2.error() == true);
    }

    TEST_CASE("read bits n")
    {
        uint8_t data[4] = {0xa5, 0xf0, 0x8c, 0x0f};
        BitstreamReader br1(data, sizeof(data));

        CHECK(br1.readBits(8) == 0xa5);
        CHECK(br1.readBits(4) == 0x0f);
        CHECK(br1.readBits(6) == 0x02); // 0b000010
        CHECK(br1.readBits(6) == 0x0c); // 0b001100
        CHECK(br1.readBits(5) == 0x01); // 0b00001
        CHECK(br1.readBits(3) == 0x07); // 0b00111
        CHECK(br1.error() == false);
    }

    TEST_CASE("read bits cross")
    {
        uint8_t data[6] = {0xa5, 0xf0, 0x8c, 0x0f, 0x75, 0x36};
        BitstreamReader br1(data, sizeof(data));

        CHECK(br1.readBits(20) == 0x0a5f08);
        CHECK(br1.readBits(20) == 0x0c0f75);
        CHECK(br1.readBits(20) == 0x036000);
        CHECK(br1.error() == false);
    }
}

TEST_SUITE("BitstreamWriter tests")
{
    TEST_CASE("write bit")
    {
        uint8_t data[4];
        BitstreamWriter bw(data, sizeof(data));

        CHECK(bw.error() == false);
        bw.writeBit(1);
        CHECK(bw.error() == false);
        bw.writeBit(0);
        bw.writeBit(1);
        bw.writeBit(0);
        bw.writeBit(0);
        bw.writeBit(1);
        bw.writeBit(0);
        bw.writeBit(1);
        for(int i = 0; i < 24; i++) {
            bw.writeBit(0);
            CHECK(bw.error() == false);
        }
        bw.flush();
        CHECK(bw.error() == false);
        CHECK(data[0] == 0xa5);
        CHECK(data[1] == 0);
        CHECK(data[2] == 0);
        CHECK(data[3] == 0);
   }

   TEST_CASE("write bit 32")
   {
        uint8_t data[5];
        BitstreamWriter bw(data, sizeof(data));

        bw.writeBits(0xa5000000, 32);
        CHECK(bw.error() == false);
        bw.writeBits(0x1a, 5);
        CHECK(bw.error() == false);
        bw.flush();
        CHECK(bw.error() == false);
        CHECK(data[0] == 0xa5);
        CHECK(data[1] == 0);
        CHECK(data[2] == 0);
        CHECK(data[3] == 0);
        CHECK(data[4] == 0xd0); // 0b11010000

        uint8_t data1[5];
        BitstreamWriter bw2(data1, sizeof(data1));
        bw2.writeBits(0x14, 5); // 0b10100
        bw2.writeBits(0xa0000005, 32);
        bw2.flush();
        CHECK(bw2.error() == false);
        CHECK(data1[0] == 0xa5);
        CHECK(data1[1] == 0);
        CHECK(data1[2] == 0);
        CHECK(data1[3] == 0);
        CHECK(data1[4] == 0x28); // 0b0101000
   }

   TEST_CASE("write bits n")
   {
        uint8_t data[4];
        BitstreamWriter bw(data, sizeof(data));
        bw.writeBits(0xa5, 8);
        bw.writeBits(0x0f, 4);
        bw.writeBits(0x02, 6); // 0b000010
        bw.writeBits(0x0c, 6); // 0b001100
        bw.writeBits(0x01, 5); // 0b00001
        bw.writeBits(0x07, 3); // 0b00111
        bw.flush();
        CHECK(bw.error() == false);
        CHECK(data[0] == 0xa5);
        CHECK(data[1] == 0xf0);
        CHECK(data[2] == 0x8c);
        CHECK(data[3] == 0x0f);
   }

    TEST_CASE("write bits cross")
    {
        uint8_t data[8];// = {0xa5, 0xf0, 0x8c, 0x0f, 0x75, 0x36};
        BitstreamWriter bw(data, sizeof(data));

        bw.writeBits(0x0a5f08, 20);
        bw.writeBits(0x0c0f75, 20);
        bw.writeBits(0x036001, 20);
        bw.flush();
        CHECK(bw.error() == false);
        CHECK(data[0] == 0xa5);
        CHECK(data[1] == 0xf0);
        CHECK(data[2] == 0x8c);
        CHECK(data[3] == 0x0f);
        CHECK(data[4] == 0x75);
        CHECK(data[5] == 0x36);
        CHECK(data[6] == 0);
        CHECK(data[7] == 0x10); // 0b00010000
    }
}