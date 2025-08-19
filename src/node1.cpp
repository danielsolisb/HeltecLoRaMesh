// ==== Heltec WiFi LoRa 32 V3 — NODE1 (sensor + router TTL + OLED) ====
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const int PIN_LORA_NSS  = 8;
static const int PIN_LORA_DIO1 = 14;
static const int PIN_LORA_RST  = 12;
static const int PIN_LORA_BUSY = 13;

#define I2C_SDA   17
#define I2C_SCL   18
#define OLED_RST  21
#define VEXT_CTRL 36

#define UART0_RX 44
#define UART0_TX 43

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RST);

static const float   FREQ_MHZ  = 915.0;
static const float   BW_KHZ    = 125.0;
static const uint8_t SF        = 7;
static const uint8_t CR_DEN    = 5;
static const uint16_t PREAMBLE = 10;
static const uint8_t  SYNC_MSB = 0x12;
static const uint8_t  CTRLBITS = 0x44;

static const uint8_t MY_ID     = 2;
static const uint8_t MASTER_ID = 1;

struct Seen{ uint16_t key; uint32_t ts; };
Seen seenBuf[128]; uint8_t seenIdx=0;
bool seenRecently(uint8_t src, uint32_t seq){
  uint16_t k=((uint16_t)src<<8) ^ (uint16_t)(seq&0xFF);
  uint32_t now=millis(); for(auto&e:seenBuf) if(e.key==k&&(now-e.ts)<30000UL) return true;
  seenBuf[seenIdx]={k,now}; seenIdx=(seenIdx+1)&127; return false;
}

volatile bool rxFlag=false; volatile bool irqEn=true;
void IRAM_ATTR onDio1(){ if(irqEn) rxFlag=true; }

void vextOn(){ pinMode(VEXT_CTRL,OUTPUT); digitalWrite(VEXT_CTRL,LOW); }
int bars(float rssi){ int b=(int)round((rssi+120.0f)*(10.0f/90.0f)); return b<0?0:(b>10?10:b); }
void drawBars(int x,int y,int b){ for(int i=0;i<10;i++){ int bx=x+i*7, by=y-i; if(i<b) oled.fillRect(bx,by,6,3,SSD1306_WHITE); else oled.drawRect(bx,by,6,3,SSD1306_WHITE);} }

size_t findAsciiStart(const uint8_t* d,size_t L){ for(size_t i=0;i<L;i++){int run=0;for(size_t j=i;j<L;j++){uint8_t c=d[j];if(c>=32&&c<=126)run++;else break;} if(run>=3) return i;} return 0; }
String asciiFromRaw(const uint8_t* b,size_t L){ size_t off=findAsciiStart(b,L); String s; s.reserve(L-off); for(size_t i=off;i<L;i++){ char c=(b[i]>=32&&b[i]<=126)?(char)b[i]:0; if(c)s+=c; } return s; }

bool parseMesh(const String& s, uint8_t& src,uint8_t& dst,uint8_t& ttl,uint32_t& seq,String& pl){
  if(!s.startsWith("M,")) return false;
  int p1=s.indexOf(',',2), p2=s.indexOf(',',p1+1), p3=s.indexOf(',',p2+1), p4=s.indexOf(',',p3+1);
  if(p1<0||p2<0||p3<0||p4<0) return false;
  src=(uint8_t)s.substring(2,p1).toInt(); dst=(uint8_t)s.substring(p1+1,p2).toInt();
  ttl=(uint8_t)s.substring(p2+1,p3).toInt(); seq=(uint32_t)s.substring(p3+1,p4).toInt();
  pl=s.substring(p4+1); return true;
}

void radioInit(){
  SPI.begin(9,11,10,PIN_LORA_NSS);
  radio.setDio2AsRfSwitch(true);
  int st=radio.begin(FREQ_MHZ,BW_KHZ,SF,CR_DEN,SYNC_MSB,14,120);
  radio.setSyncWord(SYNC_MSB,CTRLBITS);
  radio.setPreambleLength(PREAMBLE);
  radio.setCRC(true);
  radio.setDio1Action(onDio1);
  radio.startReceive();
}

uint32_t seq=0;
unsigned long lastTx=0, lastHB=0;

void setup(){
  Serial0.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);
  delay(200); Serial0.println("\n[NODE1] UART0 listo");

  vextOn(); delay(15);
  Wire.begin(I2C_SDA,I2C_SCL);
  pinMode(OLED_RST,OUTPUT); digitalWrite(OLED_RST,LOW); delay(5); digitalWrite(OLED_RST,HIGH); delay(5);
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C); oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2); oled.setCursor(0,0); oled.println("NODE1"); oled.display();

  radioInit();
}

void loop(){
  if(millis()-lastHB>1000){ lastHB=millis(); Serial0.printf("HB1 %lu\r\n", lastHB); }

  // TX periódico a MASTER
  if(millis()-lastTx>3000){
    lastTx=millis();
    String s = "M,"+String(MY_ID)+","+String(MASTER_ID)+",2,"+String(seq++)+",HELTEC1";
    int ts = radio.transmit(s);
    Serial0.printf("TX %s | rs=%d\r\n", s.c_str(), ts);

    // OLED quick status
    oled.clearDisplay(); oled.setTextSize(1);
    oled.setCursor(0,0); oled.println(F("NODE1 TX->MASTER"));
    oled.setCursor(0,12); oled.println(s);
    oled.display();
    radio.startReceive();
  }

  if(rxFlag){
    irqEn=false; rxFlag=false;
    size_t L=radio.getPacketLength(); if(L==0||L>255)L=255; uint8_t raw[255];
    int rs=radio.readData(raw,L); radio.startReceive(); irqEn=true;
    if(rs==RADIOLIB_ERR_NONE){
      float rssi=radio.getRSSI(), snr=radio.getSNR();
      String s = asciiFromRaw(raw,L);
      uint8_t src,dst,ttl; uint32_t sq; String pl;
      if(parseMesh(s,src,dst,ttl,sq,pl) && !seenRecently(src,sq)){
        Serial0.printf("RX MESH src=%u dst=%u ttl=%u seq=%u | RSSI=%.1f SNR=%.1f | %s\r\n",
                       src,dst,ttl,sq,rssi,snr,pl.c_str());
        // OLED RX status
        oled.clearDisplay(); oled.setTextSize(1);
        oled.setCursor(0,0); oled.println(F("NODE1 RX"));
        oled.setCursor(0,12); oled.print(F("src:")); oled.print(src);
        oled.setCursor(64,12); oled.print(F("seq:")); oled.println(sq);
        oled.setCursor(0,26); oled.print(F("RSSI: ")); oled.print(rssi,1);
        oled.setCursor(0,38); oled.print(F("SNR : ")); oled.print(snr,1); oled.display();

        // Forward si no es para mí
        if(dst!=MY_ID && ttl>0){
          String fwd = "M,"+String(src)+","+String(dst)+","+String((uint8_t)(ttl-1))+","+String(sq)+","+pl;
          int ts=radio.transmit(fwd);
          Serial0.printf("FWD -> dst=%u ttl->%u rs=%d\r\n", dst, (uint8_t)(ttl-1), ts);
          radio.startReceive();
        }
      }
    }
  }
}
