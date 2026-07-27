/* Implements the single arm_uart reader that keeps pose fusion continuously
 * up to date, independent of whatever else is running. */
#include "control/odometry/pose_service.h"

#include <stddef.h>

#include <robot_common/odometry_packet.h>

esp_err_t pose_service_update(PoseService *service, uint32_t now_ms) {
    if (service == NULL) return ESP_ERR_INVALID_ARG;

    (void)uart_link_update(service->arm_uart);
    while (uart_link_has_packet(service->arm_uart)) {
        PacketFrame frame = {0};
        if (uart_link_take_packet(service->arm_uart, &frame) != ESP_OK) break;

        if (odometry_packet_is(&frame)) {
            odometry_link_ingest(service->odometry_link, &frame);
        } else {
            robot_sequence_controller_handle_frame(
                service->sequence_controller, &frame, now_ms);
        }
    }

    const DrivetrainWheelCounts wheel_counts =
        drivetrain_get_wheel_counts(service->drivetrain);
    const OdometryPacket *optical_packet =
        service->odometry_link->has_packet ? &service->odometry_link->latest
                                            : NULL;
    return pose_tracker_update(service->pose_tracker, &wheel_counts, optical_packet);
}
