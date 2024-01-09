#include <Arduino.h>
#include <unity.h>
#include <iostream>
#include <sstream>
#include <string>

// Deklarieren Sie die zu testende Funktion
uint64_t convertSerialNumber(const std::string& serialNumber);

void test_convertSerialNumber() {
    uint64_t fullExpected = 0x4433221101ULL; // Erwartetes Gesamtergebnis für "116111223344"

    uint64_t actualFull = convertSerialNumber("116111223344");

    TEST_ASSERT_EQUAL_UINT64(actualFull, fullExpected);
}

#ifdef UNIT_TEST
void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_convertSerialNumber);
    UNITY_END();
}

void loop() {
    // Nichts hier
}
#endif