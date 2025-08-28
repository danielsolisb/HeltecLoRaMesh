#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

// --- Definiciones ---
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define LED_PIN  35
#define UART1_RX 5
#define UART1_TX 6

// --- Objetos ---
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
HardwareSerial Serial1(1); // Usamos el UART de hardware número 1

// --- Variables ---
unsigned long lastSendTime = 0;

void oledDisplay(String line1, String line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- PLACA A ---");
  display.setCursor(0, 12);
  display.println(line1);
  display.setCursor(0, 22);
  display.println(line2);
  display.display();
}

void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200); // Monitor USB
  pinMode(LED_PIN, OUTPUT);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false);

  // Inicializar UART1 en los pines que definimos
  Serial1.begin(9600, SERIAL_8N1, UART1_RX, UART1_TX);
  
  oledDisplay("Iniciando...");
  delay(1000);
  oledDisplay("Esperando...");
}

void loop() {
  // Enviar un mensaje cada 3 segundos
  if (millis() - lastSendTime > 3000) {
    String message = "Hola desde A!";
    Serial.printf("Enviando: %s\n", message.c_str());
    oledDisplay("Enviando:", message);
    Serial1.println(message);
    blinkLED();
    lastSendTime = millis();
  }

  // Escuchar si llegan mensajes
  if (Serial1.available()) {
    String receivedMessage = Serial1.readStringUntil('\n');
    receivedMessage.trim();
    if (receivedMessage.length() > 0) {
      Serial.printf("Recibido: %s\n", receivedMessage.c_str());
      oledDisplay("Recibido:", receivedMessage);
      blinkLED();
    }
  }
}