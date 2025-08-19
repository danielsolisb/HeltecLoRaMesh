// ==== Arduino UNO + RYLR896 — NODE2 (emisor + router TTL con AT) ====
// AltSoftSerial: RX=D8 (desde TX del RYLR 3V3), TX=D9 (-> RX del RYLR con divisor 4.7k/10k)
#include <AltSoftSerial.h>
AltSoftSerial LORA;

#define PC_BAUD 115200
#define RY_BAUD 9600

const uint8_t MY_ID     = 20;
const uint8_t MASTER_ID = 1;

uint32_t seq = 0;
unsigned long lastTx = 0;

struct Seen { uint16_t key; unsigned long ms; };
Seen seenBuf[64]; uint8_t seenIdx=0;
bool seenRecently(uint8_t src, uint32_t s){
  uint16_t k = ((uint16_t)src<<8) ^ (uint16_t)(s & 0xFF);
  unsigned long now=millis(); for(auto &e:seenBuf) if(e.key==k && (now-e.ms)<30000UL) return true;
  seenBuf[seenIdx]={k,now}; seenIdx=(seenIdx+1)&63; return false;
}

bool waitLine(String& out, unsigned long to=800){
  unsigned long t0=millis(); out="";
  while(millis()-t0<to){
    while(LORA.available()){
      char c=LORA.read();
      if(c=='\n'){ out.trim(); return true; }
      if(c!='\r') out+=c;
    }
  }
  return out.length()>0;
}

bool sendAT(const String& cmd, unsigned long to=1000){
  Serial.print("TX-> "); Serial.println(cmd);
  LORA.print(cmd); LORA.print("\r\n");
  String line; bool ok=false;
  unsigned long t0=millis();
  while(millis()-t0<to){
    if(waitLine(line,to)){
      if(line.length()) { Serial.print("RYLR> "); Serial.println(line); }
      if(line.indexOf("+OK")!=-1 || line.indexOf("+READY")!=-1) ok=true;
    }
  }
  return ok;
}

void cfgRYLR(){
  sendAT("AT");
  sendAT("AT+IPR=9600");
  sendAT("AT+BAND=915000000");
  sendAT("AT+PARAMETER=7,7,1,10");   // SF7, BW125k, CR4/5, PREAMBLE 10
  sendAT("AT+NETWORKID=0");         // pública
  sendAT("AT+ADDRESS=20");
  sendAT("AT+CRFOP=14");
  // Info
  LORA.print("AT+VER?\r\n"); LORA.print("AT+BAND?\r\n"); LORA.print("AT+PARAMETER?\r\n");
  LORA.print("AT+NETWORKID?\r\n"); LORA.print("AT+ADDRESS?\r\n");
}

// Parse '+RCV=addr,len,data,RSSI,SNR' y devuelve 'data' en outData
bool parseRCV(const String& line, String& outData, int& rssi, int& snr){
  if(!line.startsWith("+RCV=")) return false;
  int p1=line.indexOf(','), p2=line.indexOf(',',p1+1), p3=line.indexOf(',',p2+1), p4=line.indexOf(',',p3+1);
  if(p1<0||p2<0||p3<0||p4<0) return false;
  // addr = line.substring(5,p1) -> no usado aquí
  // len  = line.substring(p1+1,p2)
  outData = line.substring(p2+1,p3); // data
  rssi = line.substring(p3+1,p4).toInt();
  snr  = line.substring(p4+1).toInt();
  return true;
}

// Parse 'M,src,dst,ttl,seq,payload'
bool parseMesh(const String& s, uint8_t& src,uint8_t& dst,uint8_t& ttl,uint32_t& sq,String& pl){
  if(!s.startsWith("M,")) return false;
  int p1=s.indexOf(',',2), p2=s.indexOf(',',p1+1), p3=s.indexOf(',',p2+1), p4=s.indexOf(',',p3+1);
  if(p1<0||p2<0||p3<0||p4<0) return false;
  src=(uint8_t)s.substring(2,p1).toInt();
  dst=(uint8_t)s.substring(p1+1,p2).toInt();
  ttl=(uint8_t)s.substring(p2+1,p3).toInt();
  sq =(uint32_t)s.substring(p3+1,p4).toInt();
  pl = s.substring(p4+1);
  return true;
}

void setup(){
  Serial.begin(PC_BAUD);
  delay(200);
  Serial.println("\n[NODE2] UNO+RYLR896 (sensor/router)");
  LORA.begin(RY_BAUD); delay(100);
  cfgRYLR();
  Serial.println("[NODE2] Config lista.");
}

void loop(){
  // TX periódico (cada 3 s) hacia MASTER (broadcast aire)
  if(millis()-lastTx>3000){
    lastTx=millis();
    String payload = "M,"+String(MY_ID)+","+String(MASTER_ID)+",2,"+String(seq++)+",RYLR896";
    String cmd = "AT+SEND=0," + String(payload.length()) + "," + payload; // 0=broadcast
    sendAT(cmd, 1500);
    Serial.print("[TX] "); Serial.println(payload);
  }

  // RX: imprime y forward si aplica
  while(LORA.available()){
    String line = LORA.readStringUntil('\n'); line.trim();
    if(!line.length()) continue;
    Serial.print("RYLR> "); Serial.println(line);

    String data; int rssi=0,snr=0;
    if(parseRCV(line, data, rssi, snr)){
      uint8_t src,dst,ttl; uint32_t sq; String pl;
      if(parseMesh(data,src,dst,ttl,sq,pl)){
        Serial.print("[RX] "); Serial.print(data);
        Serial.print(" | RSSI="); Serial.print(rssi); Serial.print(" SNR="); Serial.println(snr);

        if(!seenRecently(src,sq) && dst!=MY_ID && ttl>0){
          String fwd = "M,"+String(src)+","+String(dst)+","+String((uint8_t)(ttl-1))+","+String(sq)+","+pl;
          String cmd = "AT+SEND=0," + String(fwd.length()) + "," + fwd;
          sendAT(cmd, 1500);
          Serial.print("[FWD] "); Serial.println(fwd);
        }
      }
    }
  }
}
