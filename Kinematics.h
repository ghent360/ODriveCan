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

void forwardVelocities(
    float h, float t, float s,
    float wh, float wt, float ws,
    float &vx, float &vy, float &vz);

void forwardAcceleration(
    float h, float t, float s,
    float wh, float wt, float ws,
    float ah, float at, float as,
    float &ax, float &ay, float &az);

void forwardStandingAcceleration(
    float h, float t, float s,
    float ah, float at, float as,
    float &ax, float &ay, float &az);
