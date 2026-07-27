#include <string.h>

#include <unity.h>

extern "C" {
#include "control/odometry/pose_service.h"
#include <robot_common/odometry_packet.h>
}

namespace {

// arm_uart's simulated rx queue -- pose_service_update() must drain it all
// in one call, in order.
PacketFrame queue[4] = {};
size_t queue_len = 0;
size_t queue_head = 0;

size_t odometry_ingest_calls = 0;
size_t sequence_frame_calls = 0;
size_t wheel_counts_calls = 0;
size_t pose_tracker_update_calls = 0;
const OdometryPacket *last_optical_packet = nullptr;

void queue_frame(uint8_t message_type) {
    TEST_ASSERT_TRUE(queue_len < 4);
    queue[queue_len] = {};
    queue[queue_len].message_type = message_type;
    queue_len++;
}

}  // namespace

extern "C" esp_err_t uart_link_update(UartLink *) { return ESP_OK; }

extern "C" bool uart_link_has_packet(const UartLink *) {
    return queue_head < queue_len;
}

extern "C" esp_err_t uart_link_take_packet(UartLink *, PacketFrame *packet_out) {
    if (queue_head >= queue_len) return ESP_ERR_NOT_FOUND;
    *packet_out = queue[queue_head++];
    return ESP_OK;
}

extern "C" bool odometry_packet_is(const PacketFrame *frame) {
    return frame != nullptr && frame->message_type == PACKET_TYPE_ODOMETRY;
}

extern "C" void odometry_link_ingest(Pmw3610OdometryLink *, const PacketFrame *) {
    odometry_ingest_calls++;
}

extern "C" void robot_sequence_controller_handle_frame(
    RobotSequenceController *, const PacketFrame *, uint32_t) {
    sequence_frame_calls++;
}

extern "C" DrivetrainWheelCounts drivetrain_get_wheel_counts(const Drivetrain *) {
    wheel_counts_calls++;
    return {};
}

extern "C" esp_err_t pose_tracker_update(
    PoseTracker *,
    const DrivetrainWheelCounts *,
    const OdometryPacket *optical_packet) {
    pose_tracker_update_calls++;
    last_optical_packet = optical_packet;
    return ESP_OK;
}

void setUp() {
    queue_len = 0;
    queue_head = 0;
    odometry_ingest_calls = 0;
    sequence_frame_calls = 0;
    wheel_counts_calls = 0;
    pose_tracker_update_calls = 0;
    last_optical_packet = nullptr;
}

void tearDown() {}

void test_drains_all_queued_frames_and_routes_by_type() {
    UartLink arm_uart = {};
    Pmw3610OdometryLink odometry_link = {};
    PoseService service = {
        .pose_tracker = nullptr,
        .drivetrain = nullptr,
        .arm_uart = &arm_uart,
        .odometry_link = &odometry_link,
        .sequence_controller = nullptr,
    };

    queue_frame(PACKET_TYPE_ODOMETRY);
    queue_frame(PACKET_TYPE_STATUS);
    queue_frame(PACKET_TYPE_ODOMETRY);
    queue_frame(PACKET_TYPE_PI_REPORT);

    TEST_ASSERT_EQUAL(ESP_OK, pose_service_update(&service, 100));

    TEST_ASSERT_EQUAL_UINT32(2, odometry_ingest_calls);
    TEST_ASSERT_EQUAL_UINT32(2, sequence_frame_calls);
    TEST_ASSERT_EQUAL_UINT32(0, queue_len - queue_head);
}

void test_advances_pose_with_cached_optical_sample() {
    UartLink arm_uart = {};
    Pmw3610OdometryLink odometry_link = {};
    odometry_link.has_packet = true;
    odometry_link.latest.sequence = 7;
    PoseService service = {
        .pose_tracker = nullptr,
        .drivetrain = nullptr,
        .arm_uart = &arm_uart,
        .odometry_link = &odometry_link,
        .sequence_controller = nullptr,
    };

    TEST_ASSERT_EQUAL(ESP_OK, pose_service_update(&service, 200));

    TEST_ASSERT_EQUAL_UINT32(1, wheel_counts_calls);
    TEST_ASSERT_EQUAL_UINT32(1, pose_tracker_update_calls);
    TEST_ASSERT_NOT_NULL(last_optical_packet);
    TEST_ASSERT_EQUAL_UINT32(7, last_optical_packet->sequence);
}

void test_advances_pose_with_no_optical_sample_yet() {
    UartLink arm_uart = {};
    Pmw3610OdometryLink odometry_link = {};
    PoseService service = {
        .pose_tracker = nullptr,
        .drivetrain = nullptr,
        .arm_uart = &arm_uart,
        .odometry_link = &odometry_link,
        .sequence_controller = nullptr,
    };

    TEST_ASSERT_EQUAL(ESP_OK, pose_service_update(&service, 300));

    TEST_ASSERT_EQUAL_UINT32(1, pose_tracker_update_calls);
    TEST_ASSERT_NULL(last_optical_packet);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_drains_all_queued_frames_and_routes_by_type);
    RUN_TEST(test_advances_pose_with_cached_optical_sample);
    RUN_TEST(test_advances_pose_with_no_optical_sample_yet);
    return UNITY_END();
}
