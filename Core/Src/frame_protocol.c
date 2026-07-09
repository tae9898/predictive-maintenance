/* frame_protocol.c — STM32↔RPi3 UART 프레임 인코딩 (Project 4 Phase 0) */
#include "frame_protocol.h"
#include <string.h>

/* CRC16-CCITT: poly 0x1021, init 0xFFFF, no final xor, MSB-first. */
uint16_t frame_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* buf에 [MAGIC|TYPE|LEN(LE)|PAYLOAD|CRC(LE)] 기록. CRC 범위 = TYPE|LEN|PAYLOAD. */
size_t frame_encode(uint8_t *buf, size_t bufsize, frame_type_t type,
                    const void *payload, uint16_t payload_len) {
    size_t total = FRAME_HDR_LEN + (size_t)payload_len + FRAME_CRC_LEN;
    if (bufsize < total || payload_len > FRAME_MAX_PAYLOAD) return 0;

    buf[0] = (uint8_t)FRAME_MAGIC;
    buf[1] = (uint8_t)type;
    buf[2] = (uint8_t)(payload_len & 0xFFu);          /* LEN LE */
    buf[3] = (uint8_t)((payload_len >> 8) & 0xFFu);
    if (payload_len && payload)
        memcpy(&buf[FRAME_HDR_LEN], payload, payload_len);

    /* CRC over TYPE(1) + LEN(2) + PAYLOAD = buf[1 .. 3+plen] */
    uint16_t crc = frame_crc16(&buf[1], 3u + payload_len);
    buf[FRAME_HDR_LEN + payload_len]      = (uint8_t)(crc & 0xFFu);        /* CRC LE */
    buf[FRAME_HDR_LEN + payload_len + 1]  = (uint8_t)((crc >> 8) & 0xFFu);
    return total;
}
