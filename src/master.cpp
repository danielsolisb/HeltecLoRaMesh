// ==== Heltec WiFi LoRa 32 V3 — MASTER Mesh (OLED + Serial + forward) ====
// UART0 (GPIO44 RX, 43 TX) -> mismo COM del boot
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Pines SX1262 (Heltec V3) ---
static const int PIN_LORA_NSS  = 8;
static const int PIN_LORA_DIO1 = 14;
static const int PIN_LORA_RST  = 12;
static const int PIN_LORA_BUSY = 13;

// --- OLED / I2C / Vext ---
#define I2C_SDA   17
#define I2C_SCL   18
#define OLED_RST  21
#define VEXT_CTRL 36

// --- UART0 (CDC boot COM) ---
#define UART0_RX 44
#define UART0_TX 43

// --- Instancias ---
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RST);

// --- LoRa PHY visto en sniffer ---
static const float   FREQ_MHZ  = 915.0;
static const float   BW_KHZ    = 125.0;
static const uint8_t SF        = 7;
static const uint8_t CR_DEN    = 5;       // 4/5
static const uint16_t PREAMBLE = 10;
static const uint8_t  SYNC_MSB = 0x12;    // ¡clave!
static const uint8_t  CTRLBITS = 0x44;    // ¡clave!
static const int8_t   TX_PWR   = 14;
static const uint8_t  CURR_LIM = 120;

// --- IDs ---
static const uint8_t MASTER_ID    = 1;
static const uint8_t BROADCAST_ID = 255;

// --- Dedup (src,seq bajos) ---
struct Seen { uint16_t key; uint32_t ts; };
static Seen seenBuf[128];
static uint8_t seenIdx = 0;
bool seenRecently(uint8_t src, uint32_t seq) {
  uint16_t k = ((uint16_t)src << 8) ^ (uint16_t)(seq & 0xFF);
  uint32_t now = millis();
  for (auto &e : seenBuf) if (e.key==k && (now-e.ts)<30000UL) return true;
  seenBuf[seenIdx] = {k, now}; seenIdx = (seenIdx+1) & 127;
  return false;
}

// --- IRQ ---
volatile bool rxFlag=false; volatile bool irqEn=true;
void IRAM_ATTR onDio1(){ if(irqEn) rxFlag=true; }

// --- Helpers UI ---
void vextOn(){ pinMode(VEXT_CTRL,OUTPUT); digitalWrite(VEXT_CTRL,LOW); }
int bars(float rssi){ int b=(int)round((rssi+120.0f)*(10.0f/90.0f)); return b<0?0:(b>10?10:b); }
void drawBars(int x,int y,int b){ for(int i=0;i<10;i++){ int bx=x+i*7, by=y-i; if(i<b) oled.fillRect(bx,by,6,3,SSD1306_WHITE); else oled.drawRect(bx,by,6,3,SSD1306_WHITE);} }
void showPkt(uint8_t src, uint32_t seq, float rssi, float snr, const String& note){
  oled.clearDisplay(); oled.setTextSize(1);
  oled.setCursor(0,0);  oled.println(F("MASTER mesh"));
  oled.setCursor(0,12); oled.print(F("src:")); oled.print(src);
  oled.setCursor(64,12);oled.print(F("seq:")); oled.println(seq);
  oled.setCursor(0,26); oled.print(F("RSSI: ")); oled.print(rssi,1); oled.println(F(" dBm"));
  oled.setCursor(0,38); oled.print(F("SNR : ")); oled.print(snr,1);  oled.println(F(" dB"));
  oled.setCursor(0,50); oled.print(note);
  drawBars(92,62,bars(rssi)); oled.display();
}

// --- ASCII extractor (salta encabezado binario si lo hay) ---
size_t findAsciiStart(const uint8_t* data, size_t len){
  for(size_t i=0;i<len;i++){
    int run=0; for(size_t j=i;j<len;j++){ uint8_t c=data[j]; if(c>=32 && c<=126) run++; else break; }
    if(run>=3) return i;
  }
  return 0;
}
String asciiFromRaw(const uint8_t* buf, size_t len){
  size_t off = findAsciiStart(buf,len);
  String s; s.reserve(len-off);
  for(size_t i=off;i<len;i++){ char c=(buf[i]>=32&&buf[i]<=126)?(char)buf[i]:0; if(c) s+=c; }
  return s;
}

// --- Parse "M,src,dst,ttl,seq,payload"
bool parseMesh(const String& s, uint8_t& src, uint8_t& dst, uint8_t& ttl, uint32_t& seq, String& payload){
  if(!s.startsWith("M,")) return false;
  int p1=s.indexOf(',',2), p2=s.indexOf(',',p1+1), p3=s.indexOf(',',p2+1), p4=s.indexOf(',',p3+1);
  if(p1<0||p2<0||p3<0||p4<0) return false;
  src=(uint8_t)s.substring(2,p1).toInt();
  dst=(uint8_t)s.substring(p1+1,p2).toInt();
  ttl=(uint8_t)s.substring(p2+1,p3).toInt();
  seq=(uint32_t)s.substring(p3+1,p4).toInt();
  payload=s.substring(p4+1);
  return true;
}

// --- Radio init ---
void radioInit(){
  SPI.begin(9,11,10,PIN_LORA_NSS);
  radio.setDio2AsRfSwitch(true);
  int st = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR_DEN, SYNC_MSB, TX_PWR, CURR_LIM);
  if(st!=RADIOLIB_ERR_NONE) Serial0.printf("ERR begin=%d\r\n",st);
  radio.setSyncWord(SYNC_MSB, CTRLBITS);
  radio.setPreambleLength(PREAMBLE);
  radio.setCRC(true);
  radio.setDio1Action(onDio1);
  radio.startReceive();
}

unsigned long lastHB=0;

void setup(){
  Serial0.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);
  delay(200); Serial0.println("\n[MASTER] UART0 listo");

  vextOn(); delay(15);
  Wire.begin(I2C_SDA,I2C_SCL);
  pinMode(OLED_RST,OUTPUT); digitalWrite(OLED_RST,LOW); delay(5); digitalWrite(OLED_RST,HIGH); delay(5);
  if(!oled.begin(SSD1306_SWITCHCAPVCC,0x3C)) Serial0.println("ERR OLED");
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(2);
  oled.setCursor(0,0); oled.println("MASTER"); oled.display();

  radioInit();
}

void loop(){
  if(millis()-lastHB>1000){ lastHB=millis(); Serial0.printf("HB0 %lu\r\n", lastHB); }

  if(rxFlag){
    irqEn=false; rxFlag=false;

    size_t L = radio.getPacketLength(); if(L==0||L>255) L=255;
    uint8_t raw[255];
    int rs = radio.readData(raw, L);
    radio.startReceive(); irqEn=true;

    if(rs==RADIOLIB_ERR_NONE){
      float rssi=radio.getRSSI(), snr=radio.getSNR();
      String s = asciiFromRaw(raw, L);

      uint8_t src,dst,ttl; uint32_t seq; String pl;
      if(parseMesh(s,src,dst,ttl,seq,pl) && !seenRecently(src,seq)){
        // Log al PC
        Serial0.printf("MEAS,%u,%u,%u,%u,%.1f,%.1f,%lu,%s\r\n",
          src,dst,ttl,seq,rssi,snr,(unsigned long)millis(), pl.c_str());

        // OLED
        String note = String("from ")+String(src);
        showPkt(src, seq, rssi, snr, note);

        // Forward si no soy el destino
        if(dst!=MASTER_ID && ttl>0){
          // decrementa ttl
          String fwd = "M,"+String(src)+","+String(dst)+","+String((uint8_t)(ttl-1))+","+String(seq)+","+pl;
          int ts = radio.transmit(fwd);
          Serial0.printf("FWD dst=%u ttl->%u rs=%d\r\n", dst, (uint8_t)(ttl-1), ts);
        }
      } else {
        // Si quieres ver lo que no parsea:
        // Serial0.printf("RAW \"%s\"\r\n", s.c_str());
      }
    }
  }
}
