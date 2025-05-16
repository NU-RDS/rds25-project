#include <Arduino.h>

#if (defined(TX_EXAMPLE) || defined(RX_EXAMPLE))

#ifdef TX_EXAMPLE
#include "tx_example.hpp"
#define APP tx
#elif defined(RX_EXAMPLE)
#include "rx_example.hpp"
#define APP rx
#else
#define APP unknown
#endif

void setup() {
    APP::setup();
}

void loop() {
    APP::loop();
}

#endif