#include <cstring>  // for std::strcpy
#include <stdint.h>

extern "C" void hello_from_cpp(char* buffer, int buffer_size) {
    const char* message = "Hello from C++!\n";
    // Make sure not to overflow the buffer
    if (std::strlen(message) < static_cast<size_t>(buffer_size)) {
        std::strcpy(buffer, message);
    } else if (buffer_size > 0) {
        buffer[0] = '\0';  // Write empty string if not enough space
    }
}