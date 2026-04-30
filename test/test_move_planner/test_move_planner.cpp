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

// ── Simple move step sequence ──────────────────────────────────

void test_simple_move_e2e4_step_count() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'E', 2}, {'E', 4}));
    // E2->E4: dc=0, dr=2 → MAGNET_ON + MOVE_TO(vertical) + MAGNET_OFF = 3 steps
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(3, count);
}

void test_simple_move_sequence_e2e4() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    mp.startMove({'E', 2}, {'E', 4});

    float dstX = 3.8f + 4*5.0f;  // E column
    float dstY = 5.5f + 3*5.0f;  // rank 4

    // Step 1: MAGNET_ON at E2 — x/y are 0 per Step struct contract; position in target
    Step s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MAGNET_ON, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.y);
    mp.nextStep();

    // Step 2: MOVE_TO E4 (vertical only — dc==0, same column)
    s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MOVE_TO, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstX, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstY, s.y);
    mp.nextStep();

    // Step 3: MAGNET_OFF at E4 — x/y are 0 per Step struct contract; position in target
    s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MAGNET_OFF, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.y);
    mp.nextStep();

    TEST_ASSERT_TRUE(mp.isMoveDone());
}

void test_simple_move_diagonal_two_legs() {
    // Use a rook on A1 moving to A5 (purely vertical, 1 leg)
    // then test a bishop move for two legs: C1 -> F4
    Piece board[8][8] = {};
    board[0][2] = {BISHOP, WHITE};  // C1
    ChessGame g(board);
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'C', 1}, {'F', 4}));
    // dc=3, dr=3 → MAGNET_ON + MOVE_TO(horiz) + MOVE_TO(vert) + MAGNET_OFF = 4 steps
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(4, count);
}

void test_simple_move_updates_game_state() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    mp.startMove({'E', 2}, {'E', 4});
    // drain all steps
    while (!mp.isMoveDone()) mp.nextStep();
    // game board must reflect the move and turn must have flipped
    TEST_ASSERT_TRUE(g.isEmpty({'E', 2}));
    Piece p = g.getPieceAt({'E', 4});
    TEST_ASSERT_EQUAL(PAWN, p.type);
    TEST_ASSERT_EQUAL(WHITE, p.color);
    TEST_ASSERT_EQUAL(BLACK, g.getTurn());
}

void test_start_move_illegal_returns_false() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_FALSE(mp.startMove({'E', 2}, {'E', 5}));  // pawn can't jump 3
    TEST_ASSERT_TRUE(mp.isMoveDone());                    // queue stays empty
}

void test_start_move_while_in_progress_returns_false() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'E', 2}, {'E', 4}));   // first move starts
    TEST_ASSERT_FALSE(mp.startMove({'D', 2}, {'D', 4}));  // rejected while in progress
}

// ── Obstacle displacement ──────────────────────────────────────

void test_rook_blocked_path_displaces_piece() {
    // White rook at A1, white pawn at A3 (blocker), moving rook to A5
    Piece board[8][8] = {};
    board[0][0] = {ROOK, WHITE};   // A1
    board[2][0] = {PAWN, WHITE};   // A3 — blocker on vertical leg
    ChessGame g(board);
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'A', 1}, {'A', 5}));
    // Sequence: park A3 (3 steps) + main move (3 steps: MAGNET_ON+MOVE_TO+MAGNET_OFF) + restore A3 (3 steps) = 9
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(9, count);
}

void test_displacement_first_step_is_magnet_on_blocker() {
    // White rook at A1, white pawn at A3, moving rook to A5
    Piece board[8][8] = {};
    board[0][0] = {ROOK, WHITE};
    board[2][0] = {PAWN, WHITE};
    ChessGame g(board);
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    mp.startMove({'A', 1}, {'A', 5});
    Step s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MAGNET_ON, s.type);
    // target should be the blocker position A3
    TEST_ASSERT_EQUAL('A', s.target.col);
    TEST_ASSERT_EQUAL(3,   s.target.row);
}

void test_no_displacement_when_path_clear() {
    // White rook at A1, moving to A5 with no pieces in between
    Piece board[8][8] = {};
    board[0][0] = {ROOK, WHITE};
    ChessGame g(board);
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'A', 1}, {'A', 5}));
    // 3 steps: MAGNET_ON + MOVE_TO + MAGNET_OFF (vertical only)
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(3, count);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_coord_a1);
    RUN_TEST(test_coord_h8);
    RUN_TEST(test_coord_e4);
    RUN_TEST(test_is_move_done_initially);
    RUN_TEST(test_simple_move_e2e4_step_count);
    RUN_TEST(test_simple_move_sequence_e2e4);
    RUN_TEST(test_simple_move_diagonal_two_legs);
    RUN_TEST(test_simple_move_updates_game_state);
    RUN_TEST(test_start_move_illegal_returns_false);
    RUN_TEST(test_start_move_while_in_progress_returns_false);
    RUN_TEST(test_rook_blocked_path_displaces_piece);
    RUN_TEST(test_displacement_first_step_is_magnet_on_blocker);
    RUN_TEST(test_no_displacement_when_path_clear);
    return UNITY_END();
}
