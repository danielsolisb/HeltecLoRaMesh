// ==== ATmega328P / Arduino UNO + RYLR896 (MODO: Receptor Inteligente y Tolerante) ====
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define LORA Serial
const int PIN_LED_RED   = 4;
const int PIN_LED_GREEN = 3;

#ifndef NETWORK_ID
#define NETWORK_ID 0
#endif

#define RY_BAUD 9600
const uint8_t MY_ID     = 20;
const uint8_t MASTER_ID = 1;

// Timings
const uint16_t CMD_GAP_MS    = 90;
const uint16_t AT_TO_MS      = 900;
const uint16_t SEND_TO_MS    = 1000;
const uint32_t TX_PERIOD_MS  = 3000;

// Conectividad
const uint32_t RX_ALIVE_MS     = 12000;
const uint32_t ACK_WAIT_MS     = 2000;
const uint8_t  ACK_FAIL_THRESH = 3;

// Estado
uint32_t seq = 0, lastSeqTx = 0;
unsigned long lastTx = 0, lastRxMs = 0, ackDeadline = 0;
bool cfg_ok=false, waitingAck=false;
uint8_t ackFailStreak=0;

Adafruit_ADS1115 ads;

void ticGreen(){ digitalWrite(PIN_LED_GREEN,1); delay(60); digitalWrite(PIN_LED_GREEN,0); }
void clearInput(){ while (LORA.available()) (void)LORA.read(); }
bool waitLine(String& out, unsigned long to=AT_TO_MS){ unsigned long t0=millis(); out=""; while(millis()-t0<to){ while(LORA.available()){ char c=(char)LORA.read(); if(c=='\n'||c=='\r'){out.trim();if(out.length())return true;} out+=c;}} return out.length()>0; }
bool parseMesh(const String& s, uint8_t& src, uint8_t& dst, uint8_t& ttl, uint32_t& seq, String& pl){ if(!s.startsWith("M,")) return false; int p1=s.indexOf(',',2), p2=s.indexOf(',',p1+1), p3=s.indexOf(',',p2+1), p4=s.indexOf(',',p3+1); if(p1<0||p2<0||p3<0||p4<0) return false; src=(uint8_t)s.substring(2,p1).toInt(); dst=(uint8_t)s.substring(p1+1,p2).toInt(); ttl=(uint8_t)s.substring(p2+1,p3).toInt(); seq=(uint32_t)s.substring(p3+1,p4).toInt(); pl=s.substring(p4+1); return true; }

// --- INICIO DE LA CORRECCIÓN: Lógica de recepción tolerante ---
void handleRCV(const String& line) {
  // Ya no nos importa si el inicio "+RCV=..." es perfecto.
  // Buscamos directamente el inicio de nuestro protocolo "M,".
  int mesh_start_index = line.indexOf("M,");

  if (mesh_start_index != -1) {
    // Si encontramos "M,", extraemos el resto del string desde ese punto.
    String mesh_data = line.substring(mesh_start_index);
    
    uint8_t src, dst, ttl;
    uint32_t sq;
    String pl; // Payload

    // Intentamos analizar el paquete de mesh que hemos extraído.
    if (parseMesh(mesh_data, src, dst, ttl, sq, pl)) {
      
      // Si el paquete es para nosotros (destino == 20), lo procesamos.
      if (dst == MY_ID) {
        lastRxMs = millis(); // Actualizar el timestamp de la última recepción válida

        // 1. Manejar ACKs para nuestros envíos de datos
        if (waitingAck && sq == lastSeqTx && pl.startsWith("ACK")) {
          waitingAck = false;
          ackFailStreak = 0;
          ticGreen();
          delay(50);
          ticGreen();
        }
        
        // 2. Manejar comandos del máster (¡la parte importante!)
        if (pl.equalsIgnoreCase("LED1_ON")) {
          digitalWrite(PIN_LED_RED, HIGH);
        } else if (pl.equalsIgnoreCase("LED1_OFF")) {
          digitalWrite(PIN_LED_RED, LOW);
        }
      }
    }
  }
}
// --- FIN DE LA CORRECCIÓN ---

bool sendAT(const String& cmd, unsigned long to=AT_TO_MS){ 
  clearInput(); 
  LORA.print(cmd); LORA.print("\r\n"); 
  String line; 
  bool ok=false; 
  unsigned long t0=millis(); 
  while (millis()-t0 < to){ 
    if(waitLine(line, to)) { 
      if(line.startsWith("+RCV=")){ handleRCV(line); continue; } 
      if(line.indexOf("+ERR")!=-1) return false; 
      if(line.indexOf("+OK")!=-1||line.indexOf("+READY")!=-1) ok=true;
    }
  } 
  return ok; 
}

bool cfgRYLR(){ uint8_t tries=0; while(tries<3 && !sendAT("AT",600)){tries++;delay(150);} if(tries>=3)return false; bool ok=true; ok&=sendAT("AT+IPR=9600",800);delay(CMD_GAP_MS); ok&=sendAT("AT+BAND=915000000",800);delay(CMD_GAP_MS); ok&=sendAT("AT+PARAMETER=7,7,1,10",800);delay(CMD_GAP_MS); ok&=sendAT(String("AT+NETWORKID=0"),800);delay(CMD_GAP_MS); ok&=sendAT("AT+ADDRESS=20",800);delay(CMD_GAP_MS); ok&=sendAT("AT+CRFOP=14",800);delay(CMD_GAP_MS); return ok; }

void setup(){
  pinMode(PIN_LED_RED,OUTPUT);
  pinMode(PIN_LED_GREEN,OUTPUT);
  Wire.begin();
  if (!ads.begin()) {
    while (1) { digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED)); delay(200); }
  }
  LORA.begin(RY_BAUD);
  delay(1000);
  cfg_ok = cfgRYLR();
  if(!cfg_ok) digitalWrite(PIN_LED_RED,HIGH);
}

void loop(){
  if(cfg_ok && (millis()-lastTx > TX_PERIOD_MS)){
    lastTx = millis();
    lastSeqTx = seq++;
    int16_t val_a0 = ads.readADC_SingleEnded(0);
    int16_t val_a1 = ads.readADC_SingleEnded(1);
    String payload_data = "DATA," + String(val_a0) + "," + String(val_a1);
    String payload = "M,"+String(MY_ID)+","+String(MASTER_ID)+",2,"+String(lastSeqTx)+","+payload_data;
    String cmd = "AT+SEND=0," + String(payload.length()) + "," + payload;
    sendAT(cmd, SEND_TO_MS);
    waitingAck = true;
    ackDeadline = millis() + ACK_WAIT_MS;
    ticGreen();
  }
  
  while(LORA.available()){
    String line = LORA.readStringUntil('\n'); line.trim();
    if(line.startsWith("+RCV=")) {
        handleRCV(line);
    }
  }

  if(waitingAck && (long)(millis()-ackDeadline) >= 0){
    waitingAck=false;
  }
}