/* Implements the PMW3610-to-drivetrain odometry packet producer. */
#include "comm/odometry_link_producer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "config/static_calibration.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool odometry_link_producer_init(OdometryLinkProducer *producer,
                                 const PmwPinConfig *pins) {
    if (producer == NULL) return false;
    memset(producer, 0, sizeof(*producer));

    FusionConfig config = {};
    if (!static_calibration_load(&config) ||
        !pmw3610_fusion_configure(&producer->fusion, &config)) {
        producer->ready = false;
        return false;
    }

    dual_pmw3610_init(&producer->dual, pins);
    pmw3610_pose_init(&producer->pose);
    producer->sequence = 0U;
    producer->ready = true;
    return true;
}

esp_err_t odometry_link_producer_update(OdometryLinkProducer *producer,
                                        UartLink *drivetrain_link) {
    if (producer == NULL || drivetrain_link == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!producer->ready) return ESP_OK;

    int16_t ldx = 0, ldy = 0, rdx = 0, rdy = 0;
    bool l_valid = false, r_valid = false;
    dual_pmw3610_poll(&producer->dual, &ldx, &ldy, &l_valid, &rdx, &rdy,
                      &r_valid);
    producer->last_l_valid = l_valid;
    producer->last_r_valid = r_valid;

    const DeltaPose delta =
        pmw3610_fusion_process(&producer->fusion, ldx, ldy, rdx, rdy);
    pmw3610_pose_update(&producer->pose, &delta, l_valid, r_valid);

    const Pose pose = pmw3610_pose_get(&producer->pose);
    const PoseStatus status = pmw3610_pose_get_status(&producer->pose);

    const OdometryPacket packet = {
        .x_mm = pose.x_mm,
        .y_mm = pose.y_mm,
        .theta_rad = pose.heading_deg * (float)M_PI / 180.0f,
        .sequence = ++producer->sequence,
        .valid = !status.output_killed,
    };
    return odometry_packet_send(drivetrain_link, &packet);
}
