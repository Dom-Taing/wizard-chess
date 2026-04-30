#include <unity.h>
#include "../../src/MovePlanner.h"

void setUp()    {}
void tearDown() {}

// ── Coordinate mapping ─────────────────────────────────────────

void test_coord_a1() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'A', 1}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f, x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, y);
}

void test_coord_h8() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'H', 8}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f + 7*5.0f, x);  // 38.8
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f + 7*5.0f, y);  // 40.5
}

void test_coord_e4() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'E', 4}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f + 4*5.0f, x);  // 23.8
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f + 3*5.0f, y);  // 20.5
}

// ── Step queue state ───────────────────────────────────────────

void test_is_move_done_initially() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.isMoveDone());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_coord_a1);
    RUN_TEST(test_coord_h8);
    RUN_TEST(test_coord_e4);
    RUN_TEST(test_is_move_done_initially);
    return UNITY_END();
}
