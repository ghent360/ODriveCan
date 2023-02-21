/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#pragma once

class RemoteTouch {
public:
  void initPins();
  void begin();
};

extern RemoteTouch remoteTouch;