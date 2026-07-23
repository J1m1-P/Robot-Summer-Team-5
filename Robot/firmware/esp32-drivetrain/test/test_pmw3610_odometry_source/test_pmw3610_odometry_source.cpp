/* Tests the PMW3610 cumulative-pose-packet to body-frame delta bridge. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/pmw3610_odometry_source.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

OdometryPacket make_packet(float x_mm, float y_mm, float theta_rad,
                           uint32_t sequence, bool valid) {
    OdometryPacket packet = {};
    packet.x_mm = x_mm;
    packet.y_mm = y_mm;
    packet.theta_rad = theta_rad;
    packet.sequence = sequence;
    packet.valid = valid;
    return packet;
}

}  // namespace

void setUp() {}
void tearDown() {}

// The first packet ever received only captures a baseline: there is nothing
// to diff against yet, so no delta is produced.
void test_first_packet_only_captures_baseline() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket packet = make_packet(10.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &packet, &delta));
}

// A packet whose sequence number has not advanced produces no new delta.
void test_stale_sequence_produces_no_delta() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    const OdometryPacket same_sequence = make_packet(10.0f, 0.0f, 0.0f, 1, true);
    TEST_ASSERT_FALSE(
        pmw3610_odometry_source_update(&source, &same_sequence, &delta));
}

// An invalid packet (arm-reported fault) produces no delta, even with a new
// sequence number.
void test_invalid_packet_produces_no_delta() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    const OdometryPacket invalid = make_packet(10.0f, 0.0f, 0.0f, 2, false);
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &invalid, &delta));
}

// Pure forward motion at heading 0 de-integrates directly into forward_mm.
void test_pure_forward_motion_at_zero_heading() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    const OdometryPacket second = make_packet(50.0f, 0.0f, 0.0f, 2, true);
    TEST_ASSERT_TRUE(pmw3610_odometry_source_update(&source, &second, &delta));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 50.0f, delta.forward_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 0.0f, delta.lateral_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 0.0f, delta.heading_delta_rad);
}

// Pure lateral (world +y) motion at heading 0 de-integrates directly into
// lateral_mm, not forward_mm.
void test_pure_lateral_motion_at_zero_heading() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    const OdometryPacket second = make_packet(0.0f, 15.0f, 0.0f, 2, true);
    TEST_ASSERT_TRUE(pmw3610_odometry_source_update(&source, &second, &delta));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 0.0f, delta.forward_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 15.0f, delta.lateral_mm);
}

// A world-frame delta accrued while facing +90deg rotates entirely into
// lateral_mm -- confirms de-integration uses the *previous* heading, the
// same frame the arm's own integration used.
void test_lateral_motion_at_ninety_degree_heading() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, kPi / 2.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    // Facing +90deg (body +x is world +y), so a world +y displacement is a
    // pure forward move in body frame.
    const OdometryPacket second =
        make_packet(0.0f, 20.0f, kPi / 2.0f, 2, true);
    TEST_ASSERT_TRUE(pmw3610_odometry_source_update(&source, &second, &delta));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 20.0f, delta.forward_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 0.0f, delta.lateral_mm);
}

// Heading delta wraps through +-pi rather than jumping by ~2*pi.
void test_heading_wraps_across_positive_negative_pi() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first =
        make_packet(0.0f, 0.0f, kPi - 0.1f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    const OdometryPacket second =
        make_packet(0.0f, 0.0f, -kPi + 0.1f, 2, true);
    TEST_ASSERT_TRUE(pmw3610_odometry_source_update(&source, &second, &delta));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-3f, 0.2f, delta.heading_delta_rad);
}

// Reset clears the tracked baseline and sequence so the next packet
// re-primes instead of diffing against stale state.
void test_reset_forces_new_baseline() {
    Pmw3610OdometrySource source = {};
    const OdometryPacket first = make_packet(0.0f, 0.0f, 0.0f, 1, true);
    DrivetrainOdometryDelta delta = {};
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &first, &delta));

    pmw3610_odometry_source_reset(&source);
    TEST_ASSERT_FALSE(source.has_previous);
    TEST_ASSERT_FALSE(source.has_last_sequence);

    const OdometryPacket second = make_packet(999.0f, 0.0f, 0.0f, 1, true);
    TEST_ASSERT_FALSE(pmw3610_odometry_source_update(&source, &second, &delta));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_packet_only_captures_baseline);
    RUN_TEST(test_stale_sequence_produces_no_delta);
    RUN_TEST(test_invalid_packet_produces_no_delta);
    RUN_TEST(test_pure_forward_motion_at_zero_heading);
    RUN_TEST(test_pure_lateral_motion_at_zero_heading);
    RUN_TEST(test_lateral_motion_at_ninety_degree_heading);
    RUN_TEST(test_heading_wraps_across_positive_negative_pi);
    RUN_TEST(test_reset_forces_new_baseline);
    return UNITY_END();
}
