#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FRAME_BEGIN_FLAG (0xFF00)
#define FRAME_END_FLAG (0xDD)

#define FRAME_HEAD_SIZE (20)
#define FRAME_HEAD_DATA_SIZE (16)
#define FRAME_CHECKSUM_SIZE (1)
#define FRAME_END_SIZE (1)

typedef struct {
  uint16_t frame_begin_flag;
  uint16_t frame_data_len;
  uint8_t reserved1;    // fixed to 0xff
  uint8_t output_mode;  // 0:depth only, 1:depth+ir
  uint8_t senser_temp;
  uint8_t driver_temp;
  uint8_t exposure_time[4];
  uint8_t error_code;
  uint8_t reserved2;  // fixed to 0x00
  uint8_t resolution_rows;
  uint8_t resolution_cols;
  uint16_t frame_id;  // 12-bit, 0~4095
  uint8_t isp_version;
  uint8_t reserved3;  // fixed to 0xff
} __attribute__((packed)) frame_head_t;

typedef struct {
  frame_head_t frame_head;
  uint8_t payload[];
} __attribute__((packed)) frame_t;

// typedef struct {
//   uint8_t cali_mode;  // 0:Normal, 1:Fisheye
//   uint32_t fx;        // fixpoint: u14p18
//   uint32_t fy;        // fixpoint: u14p18
//   uint32_t u0;        // fixpoint: u14p18
//   uint32_t v0;        // fixpoint: u14p18
//   uint32_t k1;        // fixpoint: s5p27
//   uint32_t k2;        // fixpoint: s5p27
//   uint32_t k3;        // fixpoint: s5p27
//   uint32_t k4_p1;     // fixpoint: s5p27, normal mode is k4, fisheye mode is p1
//   uint32_t k5_p2;     // fixpoint: s5p27, normal mode is k5 or unused, fisheye
//                       // mode is p2
//   uint32_t skew;      // fixpoint: s8p24
// } __attribute__((packed)) LensCoeff_t;

#ifdef __cplusplus
}
#endif