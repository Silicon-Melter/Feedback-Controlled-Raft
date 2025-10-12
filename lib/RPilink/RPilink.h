#pragma once
#include <Arduino.h>

/**
 * Packet format (one line, ASCII, newline-terminated):
 *   S,v=<float>,w=<float>,roll=<float>,lpwm=<int>,rpwm=<int>,ts=<uint64>*<CS>\n
 * - CS is XOR checksum (two hex digits) of everything before '*'
 * - Missing fields default to: v=0, w=0, roll=0, lpwm=1500, rpwm=1500, ts=0
 * - Accepts CR, LF, or CRLF line endings.
 *
 * Usage:
 *   RPilink link;
 *   link.begin(Serial1);            // any Stream
 *   if (link.poll()) {
 *     const RPilink::Packet& p = link.last();
 *     if (p.fresh(300)) { }
 *   }
 */

class RPilink {
public:
  struct Packet {
    float    v     = 0.0f;     // m/s
    float    w     = 0.0f;     // rad/s
    float    roll  = 0.0f;     // deg
    int16_t  lpwm  = 1500;     // 1000..2000
    int16_t  rpwm  = 1500;     // 1000..2000
    uint32_t ts_ms = 0;        // sender timestamp (ms since epoch; truncated to 32-bit)
    bool     valid = false;    // parsed at least one known field
    bool     cs_ok = false;    // checksum verified if present
    uint32_t rx_ms = 0;        // local millis() when received

    bool fresh(uint32_t timeout_ms) const {
      return (millis() - rx_ms) <= timeout_ms;
    }
  };

  RPilink() = default;

  // Attach a Stream (Serial, Serial1, SoftwareSerial, etc.)
  void begin(Stream& s) { stream_ = &s; }

  // Call often; returns true when a NEW line was consumed (parsed or not).
  bool poll();

  // Access last parsed packet
  const Packet& last() const { return last_; }

  // Clear input buffer and state
  void reset();

  // Optional: cap maximum line length (default 240 chars)
  void setMaxLine(uint16_t n) { if (n >= 32) maxLine_ = n; }

private:
  // Helpers
  static uint8_t  xorChecksum_(const char* s, size_t n);
  static int      hexNibble_(char c);
  static bool     parseKV_(const char* key, const char* val, Packet& pkt);

  bool takeLine_(String& out); // extract one complete line if available

private:
  Stream*  stream_  = nullptr;
  String   buf_;                // accumulating input
  uint16_t maxLine_ = 240;
  Packet   last_;
};
