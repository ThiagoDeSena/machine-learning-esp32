#include <Arduino.h>

const int PINO_LDR = 1;
const int INTERVALO_MS = 20; // 50 Hz (1000 ms / 50 amostras = 20 ms)
unsigned long tempoAnterior = 0;

void setup() {
  // O Edge Impulse geralmente trabalha bem com 115200 de baud rate
  Serial.begin(115200);
  
  // Aguarda a porta serial estabilizar
  while (!Serial); 
}

void loop() {
  unsigned long tempoAtual = millis();

  // Verifica se já se passaram 20 milissegundos desde a última leitura
  if (tempoAtual - tempoAnterior >= INTERVALO_MS) {
    tempoAnterior = tempoAtual;

    int valorLDR = analogRead(PINO_LDR);

    // O Data Forwarder espera apenas o valor seguido de uma quebra de linha
    Serial.println(valorLDR);
  }
}