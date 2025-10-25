/* Fragmenting CAN sender (MCP2515, 125kbps @ 8MHz)
   - CS on D10 (MCP2515)
   - Sends a text message split into fragments of 6 bytes (payload bytes 2..7)
   - ID = 0x123
*/

#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);
const uint32_t CAN_ID = 0x123;
const char MSG[] = "HOLA EQUIPO DEBIMOS CAMBIAR LA LIBRERIA ANTES";
const int PAYLOAD_PER_FRAME = 6; // bytes of text per frame (we use bytes 2..7)
const unsigned long SEND_INTERVAL_MS = 200; // intervalo entre frames

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Fragmenting TX started");
}

void sendFragment(const uint8_t* buf, uint8_t len, uint8_t seq, bool isFirst, bool isLast) {
  struct can_frame frame;
  frame.can_id  = CAN_ID;
  frame.can_dlc = 2 + len; // flags + seq + data
  frame.data[0] = (isFirst ? 0x01 : 0x00) | (isLast ? 0x02 : 0x00); // FLAGS
  frame.data[1] = seq;
  for (uint8_t i = 0; i < len; ++i) frame.data[2 + i] = buf[i];

  mcp2515.sendMessage(&frame);
}

void loop() {
  // Split MSG into fragments
  const uint8_t *ptr = (const uint8_t*)MSG;
  size_t total = strlen(MSG);
  uint8_t seq = 0;
  size_t off = 0;

  while (off < total) {
    size_t chunk = min((size_t)PAYLOAD_PER_FRAME, total - off);
    bool isFirst = (off == 0);
    bool isLast  = (off + chunk >= total);
    sendFragment(ptr + off, chunk, seq, isFirst, isLast);
    Serial.print("Sent frag seq="); Serial.print(seq);
    Serial.print(" len="); Serial.print(chunk);
    if (isLast) Serial.println(" (LAST)");
    else Serial.println();

    seq++;
    off += chunk;
    delay(SEND_INTERVAL_MS);
  }

  // Pause longer after sending full message
  delay(2000);
}
