// serial_port.h
// Raw tty access for the relay. POSIX only.

#pragma once
#include <string>

namespace relay
{

// Opens a tty raw, 8N1, no flow control, nonblocking and exclusive (EBUSY when
// another relay has it), and flushes whatever the kernel had buffered.
// Returns the fd, or -1 with errno set.
int openSerial(const std::string& path, int baud);

// True for the baud rates openSerial accepts.
bool supportedBaud(int baud);

} // namespace relay
