#include "rby1-sdk/net/real_time_control_protocol.h"

namespace rb {

int BuildRobotCommandRTPacket(unsigned char* packet, size_t N, const bool* mode, const double* target,
                              const unsigned int* feedback_gain, const double* feedforward_torque, bool finished) {
  unsigned short len = 1 +                  // N:uchar
                       N +                  // Mode:uchar[0...N-1]
                       8 * N * 2 + 4 * N +  // T,FBG,FFT:double[0...N-1]
                       1 +                  // Finished:uchar
                       1 +                  // CRC
                       2                    // footer
      ;

  int idx = 0;
  packet[idx++] = 0x24;
  packet[idx++] = 0x24;
  packet[idx++] = len & 0xff;
  packet[idx++] = ((len >> 8) & 0xff);
  packet[idx++] = (N & 0xff);
  for (int i = 0; i < N; i++) {
    packet[idx++] = (mode[i] ? 1 : 0);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&target[i]), sizeof(double));
    idx += sizeof(double);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&feedback_gain[i]), sizeof(unsigned int));
    idx += sizeof(unsigned int);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&feedforward_torque[i]), sizeof(double));
    idx += sizeof(double);
  }
  packet[idx++] = (finished ? 1 : 0);
  packet[idx] = CalculateCRC8(packet, idx);
  idx++;
  packet[idx++] = 0x25;
  packet[idx++] = 0x25;

  return idx;
}

int BuildRobotStateRTPacket(unsigned char* packet, size_t N, double t, const bool* is_ready, const double* position,
                            const double* velocity, const double* current, const double* torque) {
  unsigned short len = 1 +          // N:uchar // degree of freedom
                       8 +          // t:double // time
                       N +          // R:uchar[0...N-1] // is_ready
                       8 * N * 4 +  // P,V,C,T:double[0...N-1] // position, velocity, current, torque
                       1 +          // CRC
                       2            // footer
      ;

  int idx = 0;
  packet[idx++] = 0x24;
  packet[idx++] = 0x24;
  packet[idx++] = len & 0xff;
  packet[idx++] = ((len >> 8) & 0xff);
  packet[idx++] = (N & 0xff);
  memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&t), sizeof(double));
  idx += sizeof(double);
  for (int i = 0; i < N; i++) {
    packet[idx++] = (is_ready[i] ? 1 : 0);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&position[i]), sizeof(double));
    idx += sizeof(double);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&velocity[i]), sizeof(double));
    idx += sizeof(double);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&current[i]), sizeof(double));
    idx += sizeof(double);
  }
  for (int i = 0; i < N; i++) {
    memcpy(&packet[idx], reinterpret_cast<const unsigned char*>(&torque[i]), sizeof(double));
    idx += sizeof(double);
  }
  packet[idx] = CalculateCRC8(packet, idx);
  idx++;
  packet[idx++] = 0x25;
  packet[idx++] = 0x25;

  return idx;
}

std::pair<bool, int> ValidateRTProtocol(const unsigned char* packet, int packet_size) {
  if (packet_size < 2) {
    return {false, 0};
  }

  if (packet[0] == 0x24 && packet[1] == 0x24)
    ;
  else {
    return {false, 2};
  }

  if (packet_size < 4) {
    return {false, 0};
  }
  unsigned int len = (unsigned int)(packet[2]) | (unsigned int)(packet[3] << 8);

  if (packet_size < 2 + 2 + len) {
    return {false, 0};
  }

  if (packet[2 + 2 + len - 1] == 0x25 && packet[2 + 2 + len - 2] == 0x25)
    ;
  else {
    return {false, 2 + 2 + len};
  }

  if (packet[2 + 2 + len - 3] == CalculateCRC8((unsigned char*)packet, 2 + 2 + len - 3))
    ;
  else {
    return {false, 2 + 2 + len};
  }

  return {true, 2 + 2 + len};
}

// N
size_t GetDoFRobotStateRTProtocol(const unsigned char* packet) {
  return (size_t)(packet[4]);
}

// N
size_t GetDoFRobotCommandRTProtocol(const unsigned char* packet) {
  return (size_t)(packet[4]);
}

void ParseRobotStateRTProtocol(const unsigned char* packet, size_t* N, double* t, bool* is_ready, double* position,
                               double* velocity, double* current, double* torque) {
  int idx = 4;

  size_t dof = packet[idx];
  if (N != nullptr) {
    *N = dof;
  }
  idx++;

  if (t != nullptr) {
    memcpy(t, &packet[idx], sizeof(double));
  }
  idx += sizeof(double);

  for (int i = 0; i < dof; i++) {
    if (is_ready != nullptr) {
      is_ready[i] = packet[idx] == 1;
    }
    idx++;
  }
  for (int i = 0; i < dof; i++) {
    if (position != nullptr) {
      memcpy(&position[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
  for (int i = 0; i < dof; i++) {
    if (velocity != nullptr) {
      memcpy(&velocity[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
  for (int i = 0; i < dof; i++) {
    if (current != nullptr) {
      memcpy(&current[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
  for (int i = 0; i < dof; i++) {
    if (torque != nullptr) {
      memcpy(&torque[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
}

void ParseRobotCommandRTProtocol(const unsigned char* packet, size_t* N, bool* mode, double* target,
                                 unsigned int* feedback_gain, double* feedforward_torque, bool* finished) {
  int idx = 4;

  size_t dof = packet[idx];
  if (N != nullptr) {
    *N = dof;
  }
  idx++;

  for (int i = 0; i < dof; i++) {
    if (mode != nullptr) {
      mode[i] = packet[idx] == 1;
    }
    idx++;
  }
  for (int i = 0; i < dof; i++) {
    if (target != nullptr) {
      memcpy(&target[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
  for (int i = 0; i < dof; i++) {
    if (feedback_gain != nullptr) {
      memcpy(&feedback_gain[i], &packet[idx], sizeof(unsigned int));
    }
    idx += sizeof(unsigned int);
  }
  for (int i = 0; i < dof; i++) {
    if (feedforward_torque != nullptr) {
      memcpy(&feedforward_torque[i], &packet[idx], sizeof(double));
    }
    idx += sizeof(double);
  }
  if (finished != nullptr) {
    *finished = packet[idx] == 1;
  }
  idx++;
}

}  // namespace rb