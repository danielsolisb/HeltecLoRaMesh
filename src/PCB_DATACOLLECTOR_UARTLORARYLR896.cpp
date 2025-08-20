// ==== ATmega328P / Arduino UNO (UART0 D0/D1) + RYLR896 ====
// Usa el UART hardware como LORA (D0 RX0 <= TX RYLR 3V3 ; D1 TX0 => RX RYLR 3V3 con divisor 4.7k/10k)
// NO usa Serial para logs (evitamos colisión). Si quieres debug por otro puerto, usa SoftSerial aparte.

// -------------------- Config HW --------------------
#define LORA      Serial
const int PIN_LED_RED   = 4;   // opcional para estatus
const int PIN_LED_GREEN = 3;

// -------------------- Parámetros RYLR --------------------
#define RY_BAUD   9600
const uint8_t MY_ID     = 20;
const uint8_t MASTER_ID = 1;

// -------------------- Timings --------------------
const uint16_t CMD_GAP_MS   = 90;   // pausa entre ATs (da aire al RYLR)
const uint16_t AT_TO_MS     = 900;  // timeout razonable para +OK
const uint16_t SEND_TO_MS   = 1500; // timeout para AT+SEND
const uint32_t TX_PERIOD_MS = 3000; // tu periodo original

// -------------------- Estado --------------------
uint32_t seq = 0;
unsigned long lastTx = 0;

// ------- Utils LED (opcionales: comenta si no usas LEDs) -------
void ledBoth(bool on){ digitalWrite(PIN_LED_RED,on); digitalWrite(PIN_LED_GREEN,on); }
void blinkBoth(uint8_t n){ for(uint8_t i=0;i<n;i++){ ledBoth(true); delay(150); ledBoth(false); delay(150);} }
void ticGreen(){ digitalWrite(PIN_LED_GREEN,1); delay(60); digitalWrite(PIN_LED_GREEN,0); }

// -------------------- WaitLine CR/LF --------------------
void clearInput(){ while (LORA.available()) (void)LORA.read(); }

// Acepta '\n' o '\r' como terminador. Devuelve true si obtuvo una línea.
bool waitLine(String& out, unsigned long to=AT_TO_MS) {
  unsigned long t0=millis(); out="";
  while (millis()-t0 < to) {
    while (LORA.available()) {
      char c = (char)LORA.read();
      if (c=='\n' || c=='\r') { out.trim(); if (out.length()) return true; else continue; }
      out += c;
    }
  }
  return out.length() > 0;
}

// -------------------- AT helpers --------------------
bool sendAT(const String& cmd, unsigned long to=AT_TO_MS) {
  clearInput();
  LORA.print(cmd); LORA.print("\r\n");
  String line; bool ok=false;
  unsigned long t0=millis();
  while (millis()-t0 < to) {
    if (waitLine(line, to)) {
      // Considera +OK / +READY como éxito; +ERR fracaso inmediato
      if (line.indexOf("+ERR") != -1) return false;
      if (line.indexOf("+OK")  != -1 || line.indexOf("+READY") != -1) ok = true;
      // Si es consulta, puede venir la data primero y +OK al final: seguimos hasta timeout
    }
  }
  return ok;
}

// -------------------- AUTObaud: detecta y fuerza 9600 --------------------
bool autobaud_fix_9600() {
  const long BAUDS[] = {115200, 57600, 38400, 19200, 9600};
  for (unsigned i=0; i<sizeof(BAUDS)/sizeof(BAUDS[0]); i++) {
    LORA.begin(BAUDS[i]); delay(200);
    for (uint8_t k=0; k<3; k++) {
      if (sendAT("AT", 600)) {
        delay(CMD_GAP_MS);
        (void)sendAT("AT+IPR=9600", 800);
        delay(150);
        LORA.end(); delay(50);
        LORA.begin(9600); delay(150);
        // confirma en 9600
        for (uint8_t j=0; j<3; j++) if (sendAT("AT", 800)) return true;
        break;
      }
      delay(120);
    }
  }
  return false;
}

// -------------------- Tu lógica mesh (igual que la original) --------------------
struct Seen { uint16_t key; unsigned long ms; };
Seen seenBuf[64]; uint8_t seenIdx=0;

bool seenRecently(uint8_t src, uint32_t s){
  uint16_t k = ((uint16_t)src<<8) ^ (uint16_t)(s & 0xFF);
  unsigned long now=millis();
  for (auto &e: seenBuf) if (e.key==k && (now-e.ms)<30000UL) return true;
  seenBuf[seenIdx] = {k, now}; seenIdx = (seenIdx+1) & 63; return false;
}

// +RCV=addr,len,data,RSSI,SNR
bool parseRCV(const String& line, String& outData, int& rssi, int& snr){
  if (!line.startsWith("+RCV=")) return false;
  int p1=line.indexOf(','), p2=line.indexOf(',',p1+1), p3=line.indexOf(',',p2+1), p4=line.indexOf(',',p3+1);
  if (p1<0||p2<0||p3<0||p4<0) return false;
  outData = line.substring(p2+1,p3);
  rssi    = line.substring(p3+1,p4).toInt();
  snr     = line.substring(p4+1).toInt();
  return true;
}

// M,src,dst,ttl,seq,payload
bool parseMesh(const String& s, uint8_t& src,uint8_t& dst,uint8_t& ttl,uint32_t& sq,String& pl){
  if (!s.startsWith("M,")) return false;
  int p1=s.indexOf(',',2), p2=s.indexOf(',',p1+1), p3=s.indexOf(',',p2+1), p4=s.indexOf(',',p3+1);
  if (p1<0||p2<0||p3<0||p4<0) return false;
  src=(uint8_t)s.substring(2,p1).toInt();
  dst=(uint8_t)s.substring(p1+1,p2).toInt();
  ttl=(uint8_t)s.substring(p2+1,p3).toInt();
  sq =(uint32_t)s.substring(p3+1,p4).toInt();
  pl = s.substring(p4+1);
  return true;
}

// -------------------- Config RYLR (igual a tu cfg, con pausas) --------------------
bool cfgRYLR(){
  bool ok=true;
  // ping
  uint8_t tries=0; while(tries<3 && !sendAT("AT", 600)){ tries++; delay(150); }
  if (tries>=3) return false;

  ok &= sendAT("AT+IPR=9600", 800);          delay(CMD_GAP_MS);
  ok &= sendAT("AT+BAND=915000000", 800);    delay(CMD_GAP_MS);
  ok &= sendAT("AT+PARAMETER=7,7,1,10",800); delay(CMD_GAP_MS);
  ok &= sendAT("AT+NETWORKID=0", 800);       delay(CMD_GAP_MS);
  ok &= sendAT("AT+ADDRESS=20", 800);        delay(CMD_GAP_MS);
  ok &= sendAT("AT+CRFOP=14", 800);          delay(CMD_GAP_MS);

  // Info (no crítico; confirma eco y +OK al final)
  (void)sendAT("AT+VER?", 800);        delay(40);
  (void)sendAT("AT+PARAMETER?", 800);  delay(40);
  (void)sendAT("AT+ADDRESS?", 800);    delay(40);
  return ok;
}

// -------------------- Setup --------------------
void setup(){
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  ledBoth(false);
  blinkBoth(2);

  // 1) Autobaud y fijar 9600
  if (!autobaud_fix_9600()) {
    // fallo duro: ambos fijos
    ledBoth(true);
    while(true){}
  }

  // 2) Config LoRa como en tu sketch
  if (!cfgRYLR()) {
    // error leve: rojo fijo (se puede seguir intentando si quieres)
    digitalWrite(PIN_LED_RED, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
  }
}

// -------------------- Loop (idéntico a tu original salvo LEDs) --------------------
void loop(){
  // TX periódico (cada 3 s)
  if (millis()-lastTx > TX_PERIOD_MS) {
    lastTx = millis();
    String payload = "M,"+String(MY_ID)+","+String(MASTER_ID)+",2,"+String(seq++)+",RYLR896";
    String cmd = "AT+SEND=0," + String(payload.length()) + "," + payload; // 0=broadcast
    (void)sendAT(cmd, SEND_TO_MS);
    ticGreen(); // “tic” por cada envío
  }

  // RX: imprime (internamente) y forward si aplica
  while (LORA.available()) {
    String line = LORA.readStringUntil('\n'); line.trim();
    if(!line.length()) continue;

    String data; int rssi=0, snr=0;
    if (parseRCV(line, data, rssi, snr)) {
      uint8_t src,dst,ttl; uint32_t sq; String pl;
      if (parseMesh(data, src, dst, ttl, sq, pl)) {
        if (!seenRecently(src,sq) && dst!=MY_ID && ttl>0) {
          String fwd = "M,"+String(src)+","+String(dst)+","+String((uint8_t)(ttl-1))+","+String(sq)+","+pl;
          String cmd = "AT+SEND=0," + String(fwd.length()) + "," + fwd;
          (void)sendAT(cmd, SEND_TO_MS);
        }
      }
    }
  }
}
