/*
 * Copyright (c) 2022-2023 ghent360. See LICENSE file for details.
*/
#pragma once

void inverseKinematics(
    float x,
    float y,
    float z,
    bool posShinAngle,
    float& hipAngle,
    float& tieAngle,
    float& shinAngle);

void forwardKinematics(
    float h, float t, float s, float &x, float &y, float &z);
