#include <string.h>

#include <algorithm>
// #include <iostream>
// using namespace std;
#include <numeric>
#include <string>
#include <vector>

#include "frame_struct.h"
frame_t *handle_process(const std::string &s) {
  static std::vector<uint8_t> vecChar;
  static const uint8_t sflag_l = FRAME_BEGIN_FLAG & 0xff;
  static const uint8_t sflag_h = (FRAME_BEGIN_FLAG >> 8) & 0xff;
  static const uint8_t eflag = FRAME_END_FLAG & 0xff;

  uint32_t frame_payload_len = 0;
  frame_t *pf = NULL;
  std::vector<uint8_t>::iterator it;

  // cout << "vecChar before: " << vecChar.size() << endl;
  vecChar.insert(vecChar.end(), s.cbegin(), s.cend());
  // cout << "vecChar after: " << vecChar.size() << endl;

  if (vecChar.size() < 2) {
    // cerr << "data is not enough!" << endl;
    goto __finished;
  }

__find_header:
  for (it = vecChar.begin(); (*(it) != sflag_l) || (*(it + 1) != sflag_h);) {
    /* find sflag_l from [1:] first and next in [it+1:] */
    it = find(it + 1, vecChar.end(), sflag_l);
    /* sflag_l not found */
    if (it == vecChar.end()) {
      /* clear all data and wait next data */
      vecChar.resize(0);
      // cerr << "frame head not found! wait more data." << endl;
      goto __finished;
    }
  }

  if (it != vecChar.begin()) {
    std::vector<uint8_t>(it, vecChar.end()).swap(vecChar);
    // cerr << "frame move to first!" << endl;
  }

  if (vecChar.size() < sizeof(frame_t)) {
    // cerr << "frame head data not enough now! wait more data." << endl;
    goto __finished;
  }

  pf = (frame_t *)&vecChar[0];
  frame_payload_len = pf->frame_head.frame_data_len - FRAME_HEAD_DATA_SIZE;

  /* max frame payload size */
  if (frame_payload_len > 100 * 100) {
    // cerr << "frame head data invalid for large frame_payload_len." << endl;
    vecChar.pop_back();
    vecChar.pop_back();
    goto __find_header;
  }

  if (vecChar.size() < FRAME_HEAD_SIZE + frame_payload_len +
                           FRAME_CHECKSUM_SIZE + FRAME_END_SIZE) {
    // cerr << "expected frame payload length: " << frame_payload_len << endl;
    // cerr << "frame payload data not enough now! wait more data." << endl;
    goto __finished;
  }

  {
    uint8_t check_sum = std::accumulate(
        vecChar.begin(), vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len,
        (uint8_t)0);

    if (check_sum != ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len] ||
        eflag != ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len +
                                 FRAME_CHECKSUM_SIZE]) {
      // cerr << "src\tchecksum\ttail" << endl;
      // cerr << "data\t"
      //      << *(vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len) <<
      //      '\t'
      //      << *(vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len +
      //           FRAME_CHECKSUM_SIZE)
      //      << endl;
      // cerr << "data\t" << check_sum << '\t' << eflag << endl;
      vecChar.pop_back();
      vecChar.pop_back();
      goto __find_header;
    }
  }

  pf = (frame_t *)malloc(sizeof(frame_t) + frame_payload_len);
  memcpy(pf, &vecChar[0], sizeof(frame_t) + frame_payload_len);

  std::vector<uint8_t>(vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len +
                           FRAME_CHECKSUM_SIZE + FRAME_END_SIZE,
                       vecChar.end())
      .swap(vecChar);
  return pf;

__finished:
  return NULL;
}