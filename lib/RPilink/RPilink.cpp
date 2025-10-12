#include "RPilink.h"
#include <stdlib.h>

static inline void trimString_(String& s) { s.trim(); }

uint8_t RPilink::xorChecksum_(const char* s, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; ++i) x ^= (uint8_t)s[i];
  return x;
}

int RPilink::hexNibble_(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool RPilink::parseKV_(const char* key, const char* val, Packet& pkt) {
  if (!key || !val) return false;

  auto toInt = [](const char* s, long& out)->bool {
    char* e=nullptr; long v=strtol(s, &e, 10); if (e==s) return false; out=v; return true;
  };
  auto toUInt = [](const char* s, unsigned long& out)->bool {
    char* e=nullptr; unsigned long v=strtoul(s, &e, 10); if (e==s) return false; out=v; return true;
  };
  auto toFloat = [](const char* s, float& out)->bool {
    char* e=nullptr; double v=strtod(s, &e); if (e==s) return false; out=(float)v; return true;
  };

  if (strcmp(key, "v") == 0)        { float f; if (toFloat(val, f)) { pkt.v = f; return true; } }
  else if (strcmp(key, "w") == 0)   { float f; if (toFloat(val, f)) { pkt.w = f; return true; } }
  else if (strcmp(key, "roll") == 0){ float f; if (toFloat(val, f)) { pkt.roll = f; return true; } }
  else if (strcmp(key, "lpwm") == 0){ long v;  if (toInt(val, v))   { pkt.lpwm = (int16_t)constrain(v, 1000, 2000); return true; } }
  else if (strcmp(key, "rpwm") == 0){ long v;  if (toInt(val, v))   { pkt.rpwm = (int16_t)constrain(v, 1000, 2000); return true; } }
  else if (strcmp(key, "ts") == 0)  { unsigned long u; if (toUInt(val, u)) { pkt.ts_ms = (uint32_t)u; return true; } }

  return false; // ignore unknown keys
}

bool RPilink::takeLine_(String& out) {
  if (stream_) {
    while (stream_->available()) {
      char ch = (char)stream_->read();
      if (ch == '\r' || ch == '\n') {
        if (ch == '\r' && stream_->peek() == '\n') (void)stream_->read(); // CRLF
        trimString_(buf_);
        if (buf_.length()) {
          out = buf_;
          buf_.remove(0);
          return true;
        }
        buf_.remove(0); // empty line; keep reading
      } else {
        if (buf_.length() < maxLine_) buf_ += ch;
      }
    }
  }
  return false;
}

bool RPilink::poll() {
  String line;
  if (!takeLine_(line)) return false;

  Packet pkt; // defaults
  pkt.valid = false;
  pkt.cs_ok = false;

  // Expect "S,..." with optional "*CS"
  if (!line.startsWith("S,")) {
    last_.valid = false;
    last_.cs_ok = false;
    last_.rx_ms = millis();
    return true; // consumed a line, even if not our packet
  }

  int star = line.lastIndexOf('*');
  String body = (star > 0) ? line.substring(0, star) : line;
  String cs   = (star > 0) ? line.substring(star + 1) : String();
  trimString_(body); trimString_(cs);

  // Checksum if present
  if (cs.length() >= 2) {
    int hi = hexNibble_(cs[0]), lo = hexNibble_(cs[1]);
    if (hi >= 0 && lo >= 0) {
      uint8_t want = (uint8_t)((hi << 4) | lo);
      uint8_t got  = xorChecksum_(body.c_str(), body.length());
      pkt.cs_ok = (want == got);
    }
  }

  // Parse key=value pairs (skip "S,")
  int pos = body.indexOf(',');
  while (pos >= 0 && pos < (int)body.length()) {
    int next = body.indexOf(',', pos + 1);
    String kv = (next < 0) ? body.substring(pos + 1) : body.substring(pos + 1, next);
    trimString_(kv);
    if (kv.length()) {
      int eq = kv.indexOf('=');
      if (eq > 0) {
        String key = kv.substring(0, eq); trimString_(key);
        String val = kv.substring(eq + 1); trimString_(val);
        pkt.valid |= parseKV_(key.c_str(), val.c_str(), pkt);
      }
    }
    if (next < 0) break;
    pos = next;
  }

  pkt.rx_ms = millis();
  last_ = pkt;
  return true;
}

void RPilink::reset() {
  buf_.remove(0);
  last_ = Packet();
}
