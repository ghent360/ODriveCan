/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>
#include "TaskManager.hpp"

void initSerialInteraction();
void checkSerialInput(TaskNode*, uint32_t);
