#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);     // CS en D10 (ajusta si usas otro)
const int LED = 13;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {}

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);  // 125 kbps @ 8 MHz
  mcp2515.setNormalMode();

  Serial.println("MCP2515 Tx @125kbps, 8MHz, NORMAL");
}

void loop() {
  struct can_frame f;
  f.can_id  = 0x012;      // ID estándar (11 bits)
  f.can_dlc = 5;
  memcpy(f.data, "hello", 5);

  MCP2515::ERROR e = mcp2515.sendMessage(&f);

  // Parpadeo corto para marcar intento de TX
  digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);

  // Diagnóstico: contadores y flags
  uint8_t tec = mcp2515.errorCountTX();
  uint8_t rec = mcp2515.errorCountRX();
  uint8_t flags = mcp2515.getErrorFlags();

  Serial.print("TX: ");
  Serial.print(e == MCP2515::ERROR_OK ? "OK" : "ERR");
  Serial.print("  TEC="); Serial.print(tec);
  Serial.print(" REC=");  Serial.print(rec);
  Serial.print(" FLAGS=0x"); Serial.println(flags, HEX);

  delay(1000);
}
