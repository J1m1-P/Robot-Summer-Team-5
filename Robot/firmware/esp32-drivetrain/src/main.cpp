// #include <Arduino.h>
// #include <string.h>

// #include "communication/uart/packet_link.h"
// #include "config/packet_link_config.h"

// typedef struct {
//     uint32_t sequence;
//     float x_m;
//     float y_m;
//     float heading_rad;
// } TestOdometryData;

// static_assert(
//     sizeof(TestOdometryData) <= PACKET_MAX_PAYLOAD_SIZE,
//     "TestOdometryData is too large"
// );

// static PacketLink packet_link = {0};
// static uint32_t sequence = 0U;

// void setup()
// {
//     Serial.begin(115200);
//     delay(1000);

//     esp_err_t err = packet_link_init(
//         &packet_link,
//         &TOP_ESP_PACKET_LINK_CONFIG
//     );

//     if (err != ESP_OK) {
//         Serial.printf(
//             "Packet link init failed: %s\n",
//             esp_err_to_name(err)
//         );
//         return;
//     }

//     Serial.println("Sender ready");
// }

// void loop()
// {
//     TestOdometryData data = {
//         .sequence = sequence,
//         .x_m = sequence * 0.10f,
//         .y_m = sequence * 0.05f,
//         .heading_rad = sequence * 0.01f
//     };

//     esp_err_t err = packet_link_send(
//         &packet_link,
//         PACKET_TYPE_ODOMETRY,
//         (const uint8_t *)&data,
//         (uint8_t)sizeof(data)
//     );

//     if (err == ESP_OK) {
//         Serial.printf(
//             "Sent %lu: x=%.2f, y=%.2f, heading=%.2f\n",
//             (unsigned long)data.sequence,
//             data.x_m,
//             data.y_m,
//             data.heading_rad
//         );

//         sequence++;
//     } else {
//         Serial.printf(
//             "Send failed: %s\n",
//             esp_err_to_name(err)
//         );
//     }

//     delay(500);
// }

#include <Arduino.h>
#include <string.h>

#include "communication/uart/packet_link.h"
#include "config/packet_link_config.h"

typedef struct {
    uint32_t sequence;
    float x_m;
    float y_m;
    float heading_rad;
} TestOdometryData;

static PacketLink packet_link = {0};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    esp_err_t err = packet_link_init(
        &packet_link,
        &TOP_ESP_PACKET_LINK_CONFIG
    );

    if (err != ESP_OK) {
        Serial.printf(
            "Packet link init failed: %s\n",
            esp_err_to_name(err)
        );
        return;
    }

    Serial.println("Receiver ready");
}

void loop()
{
    esp_err_t err = packet_link_update(&packet_link);

    if (err != ESP_OK) {
        Serial.printf(
            "Update failed: %s\n",
            esp_err_to_name(err)
        );
        delay(100);
        return;
    }

    PacketFrame packet;

    if (packet_link_take_packet(&packet_link, &packet) == ESP_OK) {
        if (packet.message_type == PACKET_TYPE_ODOMETRY &&
            packet.payload_len == sizeof(TestOdometryData)) {

            TestOdometryData data;

            memcpy(
                &data,
                packet.payload,
                sizeof(data)
            );

            Serial.printf(
                "Received %lu: x=%.2f, y=%.2f, heading=%.2f\n",
                (unsigned long)data.sequence,
                data.x_m,
                data.y_m,
                data.heading_rad
            );
        } else {
            Serial.printf(
                "Unexpected packet: type=%u, length=%u\n",
                packet.message_type,
                packet.payload_len
            );
        }
    }

    delay(1);
}