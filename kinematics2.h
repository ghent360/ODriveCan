/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

void forwardKinematics(float th1, float th2, float& x, float & y);
void inverseKinematics(float x, float y, float& th1, float& th2);

void forwardKinematics2(
    float th1, float th2, float th3, float& x, float& y, float& z);
void inverseKinematics2(
    float x, float y, float z, float& th1, float& th2, float& th3);