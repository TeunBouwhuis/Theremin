#include <Arduino.h>
#include <unity.h>
#include <main.cpp> 
// pio test -vvv
// bruuuh

void setUp(void) {}
void tearDown(void) {}


void testFilter(void) {
    FILTER_LIST f; ///bruh
    newFilter(&f);
    filterAdd(&f, 10);
}

void setup() {
    UNITY_BEGIN();
    testFilter();
    UNITY_END();
}

void loop() {}