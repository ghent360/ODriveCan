/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
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

void inverseKinematics_new(
    float x,
    float y,
    float z,
    bool posShinAngle,
    float &h,
    float &t,
    float &s);

void forwardKinematics(
    float h, float t, float s, float &x, float &y, float &z);
