// Host unit tests for the pure curb free-space reducer (src/curb_reduce.h).
// Run on a machine with a host C++ compiler:  pio test -e native
// Expected values were cross-checked against an independent Python reference.
#include <unity.h>
#include "curb_reduce.h"

using namespace curb_reduce;

// Build a cell array from a pattern: 'F' free-enabled, 'O' occupied-enabled,
// 'D' disabled (dead zone). Returns the count.
static int mk(Cell* c, const char* pat, float lenM, uint8_t strip) {
  int n = 0;
  for (const char* p = pat; *p; ++p) {
    c[n].occupied = (*p == 'O');
    c[n].enabled  = (*p != 'D');
    c[n].strip    = strip;
    c[n].lenM     = lenM;
    n++;
  }
  return n;
}

void setUp(void) {}
void tearDown(void) {}

// Empty 5-cell strip (2 m each), pitch 6, clearInterior 1.2, clearEnd 1.8.
// One 10 m run flush against both physical ends -> terminal: floor((10-1.8)/6)=1.
void test_empty_strip(void) {
  Cell c[8]; int n = mk(c, "FFFFF", 2.0f, 0);
  Params p{6.0f, 1.2f, 1.8f}; Aggregate ag;
  aggregate(c, n, 1, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ag.longest_free_run_m);
  TEST_ASSERT_EQUAL_INT(1, ag.raw_spaces);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ag.reliable_range_m);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, ag.occupied_len);
}

// One car in the middle splits the strip into two 4 m runs (each < pitch -> 0).
void test_car_in_middle(void) {
  Cell c[8]; int n = mk(c, "FFOFF", 2.0f, 0);
  Params p{6.0f, 1.2f, 1.8f}; Aggregate ag;
  aggregate(c, n, 1, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(8.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, ag.longest_free_run_m);
  TEST_ASSERT_EQUAL_INT(0, ag.raw_spaces);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, ag.occupied_len);
}

// Interior gap (parked cars both sides): 8 m, floor((8-1.2)/6)=1.
void test_interior_gap(void) {
  Cell c[8]; int n = mk(c, "OFFFFO", 2.0f, 0);
  Params p{6.0f, 1.2f, 1.8f}; Aggregate ag;
  aggregate(c, n, 1, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(8.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_INT(1, ag.raw_spaces);
}

// The clearance rule matters: the SAME 7 m gap yields one space when interior
// (parked cars both sides) but zero when it touches a dead zone (disabled cell).
// pitch 5, clearInterior 1.0, clearEnd 2.5, cell 1.75 m.
void test_deadzone_vs_interior_clearance(void) {
  Cell c[8]; Aggregate ag;
  Params p{5.0f, 1.0f, 2.5f};

  int n = mk(c, "DFFFFO", 1.75f, 0);   // gap bounded by a dead zone -> terminal
  aggregate(c, n, 1, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(7.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_INT(0, ag.raw_spaces);

  n = mk(c, "OFFFFO", 1.75f, 0);       // same gap, parked cars both sides -> interior
  aggregate(c, n, 1, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(7.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_INT(1, ag.raw_spaces);
}

// Two strips aggregate independently and sum into the totals.
void test_two_strips(void) {
  Cell c[8];
  int n0 = mk(c, "FFF", 2.0f, 0);
  int n  = n0 + mk(c + n0, "FOF", 2.0f, 1);
  Params p{6.0f, 1.2f, 1.8f}; Aggregate ag;
  aggregate(c, n, 2, p, ag);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ag.free_curb_m);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, ag.longest_free_run_m);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, ag.strip[0].free_curb_m);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, ag.strip[1].free_curb_m);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, ag.occupied_len);
}

// Width-3 median fills a single-cell hole and removes a single-cell blip.
void test_spatial_median(void) {
  Cell c[8];
  int n = mk(c, "OFO", 2.0f, 0); spatialMedian(c, n, 1);
  TEST_ASSERT_TRUE(c[1].occupied);   // hole between two cars filled
  n = mk(c, "FOF", 2.0f, 0); spatialMedian(c, n, 1);
  TEST_ASSERT_FALSE(c[1].occupied);  // lone blip removed
}

// One isolated car folds exactly one observation; a second identical frame does
// not re-count it (episode still active).
void test_pitch_learn_one_episode(void) {
  Cell c[8]; int n = mk(c, "FOOF", 2.5f, 0);   // 5 m car, free-enabled both sides
  PitchLearnState st{};
  PitchLearnParams lp{1.0f, 3.0f, 6.5f, 0.05f, 0xFFFF};
  pitchLearn(c, n, 1, lp, st);
  TEST_ASSERT_EQUAL_UINT16(1, st.learnSamples);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, st.carPitchLearned);   // 5 m + 1 m clearance
  pitchLearn(c, n, 1, lp, st);
  TEST_ASSERT_EQUAL_UINT16(1, st.learnSamples);
}

// The integer stability hold promotes only after stableNeed identical frames and
// ignores a single-frame blip.
void test_spaces_hold(void) {
  SpacesHold h{-1, -1, 0};
  TEST_ASSERT_EQUAL_INT(3, spacesHold(3, 2, h));   // first sight -> report raw
  TEST_ASSERT_EQUAL_INT(3, spacesHold(3, 2, h));   // held stableNeed frames -> promote
  TEST_ASSERT_EQUAL_INT(3, h.reported);
  TEST_ASSERT_EQUAL_INT(3, spacesHold(5, 2, h));   // one blip -> not yet promoted
  TEST_ASSERT_EQUAL_INT(3, spacesHold(3, 2, h));   // back to reported -> pending cleared
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_strip);
  RUN_TEST(test_car_in_middle);
  RUN_TEST(test_interior_gap);
  RUN_TEST(test_deadzone_vs_interior_clearance);
  RUN_TEST(test_two_strips);
  RUN_TEST(test_spatial_median);
  RUN_TEST(test_pitch_learn_one_episode);
  RUN_TEST(test_spaces_hold);
  return UNITY_END();
}
