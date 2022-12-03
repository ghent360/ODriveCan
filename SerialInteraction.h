/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>
#include "TaskManager.hpp"

void initSerialInteraction();
void checkSerialInput(TaskNode*, uint32_t);
