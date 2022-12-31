/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
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