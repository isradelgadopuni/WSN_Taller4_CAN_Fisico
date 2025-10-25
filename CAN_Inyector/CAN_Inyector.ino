#include <SPI.h>
#include <mcp2515.h>

// ======== CONFIG CAN / IDs ========
#define CAN_ID_STATUS  0x180  // Intermitentes + Velocidad (5 bytes)
#define CAN_ID_DOORS   0x181  // Estado de puertas (6 bytes)
MCP2515 mcp2515(10);         // CS en D10 (SPI: 11/12/13)

// ======== BOTONES (evitamos D2 y 10-13) ========
// Conexión: botón a GND, pin como INPUT_PULLUP (activo en LOW)
const uint8_t BTN_LEFT       = 3;  // intermitente izquierda (toggle)
const uint8_t BTN_RIGHT      = 4;  // intermitente derecha (toggle)
const uint8_t BTN_ACCEL      = 5;  // acelerar
const uint8_t BTN_DOORS_ALL  = 6;  // toggle todo cerrado/abierto
const uint8_t BTN_DOOR_FL    = 7;  // puerta delantera izquierda (toggle)
const uint8_t BTN_DOOR_FR    = 8;  // puerta delantera derecha (toggle)
const uint8_t BTN_DOOR_RL    = 9;  // puerta trasera izquierda (toggle)
const uint8_t BTN_DOOR_RR    = A0; // puerta trasera derecha (toggle)

// ======== ANTIRREBOTE ========
const uint16_t DEBOUNCE_MS = 30;
struct Debounce {
  uint8_t pin; bool lastStable; bool lastRead; unsigned long tEdge;
  void begin(uint8_t p){ pin=p; pinMode(pin, INPUT_PULLUP); lastRead=digitalRead(pin); lastStable=lastRead; tEdge=millis(); }
  bool fell(){ // flanco de bajada
    bool r = digitalRead(pin); unsigned long now=millis();
    if (r!=lastRead){ lastRead=r; tEdge=now; }
    if ((now - tEdge) > DEBOUNCE_MS && lastStable!=lastRead){ lastStable=lastRead; if (lastStable==LOW) return true; }
    return false;
  }
};
Debounce dbL, dbR, dbAcc, dbAll, dbFL, dbFR, dbRL, dbRR;

// ======== ESTADO ========
// Intermitentes: bit0=izq(0x01), bit1=der(0x02)
uint8_t blinkMask = 0x00;

// Velocidad 16-bit (BE) bytes [3]=hi, [4]=lo
uint16_t speedVal = 0x0000;
const uint16_t SPEED_STEP = 0x0010;
const uint16_t SPEED_MAX  = 0x3894;
const uint16_t SPEED_MIN  = 0x0000;

// Puertas (6B): [00][00][mask][00][00][00], mask en byte2: 01 FL, 02 FR, 04 RL, 08 RR
// 1 = cerrada, 0 = abierta
uint8_t doorMask = 0x0F; // arrancamos TODO CERRADO

// Reenvío periódico
const unsigned long PERIOD_MS = 100; // 10 Hz
unsigned long tLast = 0;

// ======== HELPERS CAN ========
void canSend(uint32_t id, const uint8_t *data, uint8_t len){
  struct can_frame f; f.can_id=id; f.can_dlc=len;
  for(uint8_t i=0;i<len;i++) f.data[i]=data[i];
  mcp2515.sendMessage(&f);
}

// 5B: [blink][XX][XX][speed_hi][speed_lo]
void sendStatus(){
  uint8_t p[5];
  p[0]=blinkMask;
  p[1]=0x00; // relleno (ajusta si necesitas valor distinto)
  p[2]=0x00; // relleno
  p[3]=(uint8_t)((speedVal>>8)&0xFF);
  p[4]=(uint8_t)(speedVal&0xFF);
  canSend(CAN_ID_STATUS, p, 5);
}

// 6B: [00][00][doorMask][00][00][00]
void sendDoors(){
  uint8_t p[6]={0x00,0x00,doorMask,0x00,0x00,0x00};
  canSend(CAN_ID_DOORS, p, 6);
}

void setup(){
  Serial.begin(115200);
  while(!Serial){}

  // Botones
  dbL.begin(BTN_LEFT); dbR.begin(BTN_RIGHT); dbAcc.begin(BTN_ACCEL); dbAll.begin(BTN_DOORS_ALL);
  dbFL.begin(BTN_DOOR_FL); dbFR.begin(BTN_DOOR_FR); dbRL.begin(BTN_DOOR_RL); dbRR.begin(BTN_DOOR_RR);

  // CAN
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ); // 125 kbps @ 8 MHz
  mcp2515.setNormalMode();

  Serial.println(F("MCP2515 listo (125kbps, 8MHz)."));
}

void loop(){
  // ---- Botones: intermitentes ----
  if (dbL.fell()){ blinkMask ^= 0x01; Serial.print(F("L ")); Serial.println((blinkMask&0x01)?F("ON"):F("OFF")); }
  if (dbR.fell()){ blinkMask ^= 0x02; Serial.print(F("R ")); Serial.println((blinkMask&0x02)?F("ON"):F("OFF")); }

  // ---- Botón acelerador (solo subir; ajusta si quieres freno) ----
  if (dbAcc.fell()){
    uint32_t next = (uint32_t)speedVal + SPEED_STEP;
    speedVal = (next > SPEED_MAX) ? SPEED_MAX : (uint16_t)next;
    Serial.print(F("Speed=0x")); Serial.println(speedVal, HEX);
  }

  // ---- Botón general puertas: toggle ALL ----
  if (dbAll.fell()){
    doorMask = (doorMask==0x0F) ? 0x00 : 0x0F; // 0x0F cerradas -> 0x00 abiertas
    Serial.print(F("Doors ALL -> ")); Serial.println((doorMask==0x0F)?F("CLOSED"):F("OPEN"));
  }

  // ---- Botones de puertas individuales (toggle bit) ----
  if (dbFL.fell()){ doorMask ^= 0x01; Serial.print(F("Door FL -> ")); Serial.println((doorMask&0x01)?F("CLOSED"):F("OPEN")); }
  if (dbFR.fell()){ doorMask ^= 0x02; Serial.print(F("Door FR -> ")); Serial.println((doorMask&0x02)?F("CLOSED"):F("OPEN")); }
  if (dbRL.fell()){ doorMask ^= 0x04; Serial.print(F("Door RL -> ")); Serial.println((doorMask&0x04)?F("CLOSED"):F("OPEN")); }
  if (dbRR.fell()){ doorMask ^= 0x08; Serial.print(F("Door RR -> ")); Serial.println((doorMask&0x08)?F("CLOSED"):F("OPEN")); }

  // ---- Envío periódico ----
  unsigned long now = millis();
  if (now - tLast >= PERIOD_MS){
    tLast = now;
    sendStatus(); // 5 bytes: intermitentes + velocidad (bytes 3,4)
    sendDoors();  // 6 bytes: estado de puertas (byte2 = máscara)
  }
}
