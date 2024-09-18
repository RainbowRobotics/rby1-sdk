#pragma once

#include <sys/socket.h>
#include <regex>
#include <string>

namespace rb {

inline std::string DecodeUrl(const std::string& encoded) {
  std::string decoded;
  char ch;
  int i, j;
  for (i = 0; i < encoded.length(); i++) {
    if (encoded[i] == '%') {
      sscanf(encoded.substr(i + 1, 2).c_str(), "%x", &j);
      ch = static_cast<char>(j);
      decoded += ch;
      i += 2;
    } else if (encoded[i] == '+') {
      decoded += ' ';
    } else {
      decoded += encoded[i];
    }
  }
  return decoded;
}

inline bool ParseIPAndFamily(std::string addr, std::string& ip, int& family) {
  addr = DecodeUrl(addr);

  std::regex ipv4_regex(R"(ipv4:([\d\.]+):\d+)");
  std::regex ipv6_regex(R"(ipv6:\[([0-9a-fA-F:]+)\]:\d+)");
  std::smatch match;

  if (std::regex_search(addr, match, ipv4_regex)) {
    ip = match[1];
    family = AF_INET;
    return true;
  } else if (std::regex_search(addr, match, ipv6_regex)) {
    ip = match[1];
    family = AF_INET6;
    return true;
  }
  return false;
}

}  // namespace rb