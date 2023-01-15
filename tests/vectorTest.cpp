/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 */
#include "../Vector.hpp"

#include "doctest.h"

using Vector2Df = Vector<float, 2>;
using Vector2D = Vector<double, 2>;
using Vector3D = Vector<double>;

TEST_SUITE("Vector template tests")
{
    TEST_CASE("test 2Df")
    {
        Vector2Df x({1, 0});
        Vector2Df y({0, 1});
        CHECK(y.getSize() == 1);
        CHECK(x.getSize() == 1);
        CHECK(x.dot(y) == 0);
        CHECK(y.dot(x) == 0);
        Vector2Df v({100, 200});
        CHECK(y.dot(v) == 200);
        CHECK(x.dot(v) == 100);
        v.normalize();
        CHECK(fabsf(v.getSize() - 1) < 1E-7);
    }

    TEST_CASE("test 2D")
    {
        Vector2D x({1, 0});
        Vector2D y({0, 1});
        CHECK(y.getSize() == 1);
        CHECK(x.getSize() == 1);
        CHECK(x.dot(y) == 0);
        CHECK(y.dot(x) == 0);
        Vector2D v({100, 200});
        CHECK(y.dot(v) == 200);
        CHECK(x.dot(v) == 100);
        v.normalize();
        CHECK(fabs(v.getSize() - 1) < 1E-15);
    }

    TEST_CASE("test 3D")
    {
        Vector3D x({1, 0, 0});
        Vector3D y({0, 1, 0});
        Vector3D z({0, 0, 1});
        CHECK(y.getSize() == 1);
        CHECK(x.getSize() == 1);
        CHECK(y.getSize() == 1);
        CHECK(y.dot(x) == 0);
        CHECK(x.dot(y) == 0);
        CHECK(x.dot(z) == 0);
        CHECK(z.dot(x) == 0);
        CHECK(y.dot(z) == 0);
        CHECK(z.dot(y) == 0);
        Vector3D v1({100, 200, 300});
        Vector3D v2({300, 200, 100});
        CHECK(x.dot(v1) == 100);
        CHECK(y.dot(v1) == 200);
        CHECK(z.dot(v1) == 300);
        auto v = v1 + v2;
        v1.normalize();
        CHECK(fabs(v1.getSize() - 1) < 1E-15);
        auto v2n = normalize(v2);
        CHECK(fabs(v2n.getSize() - 1) < 1E-15);
        CHECK(v.get(0) == 400);
        CHECK(v.get(1) == 400);
        CHECK(v.get(2) == 400);
        v += v2;
        CHECK(v[0] == 700);
        CHECK(v[1] == 600);
        CHECK(v[2] == 500);
        v -= v2;
        CHECK(v.getX() == 400);
        CHECK(v.getY() == 400);
        CHECK(v.getZ() == 400);
        Vector3D m = v2 * 3;
        CHECK(m.getX() == 900);
        CHECK(m.getY() == 600);
        CHECK(m.getZ() == 300);
        m /= 3;
        CHECK(m.getX() == 300);
        CHECK(m.getY() == 200);
        CHECK(m.getZ() == 100);
    }
}
