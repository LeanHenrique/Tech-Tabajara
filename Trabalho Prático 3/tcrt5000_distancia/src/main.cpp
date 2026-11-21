#include <Arduino.h>
#include <math.h>

// TCRT5000 como sensor de distância (Aula 13 - PSR 2026/2, equipe RTech Tabajara)
// LED infravermelho no D8 (via 100 ohm), coletor do fototransistor no A0 (pull-up de 2 x 2 kohm).
// Imprime na serial, a cada ciclo: sinal,mm
//
// Modelo inverso: d = G0 + G1 / sqrt(S), com S = max(0, adc_off - adc_on).
// Ganhos gravados no ensaio (ajuste de 0 a 200 mm, planilha calibracao_tcrt5000.xlsx, aba Ajustes, linha 6).
// Para usar o reajuste de 20 a 180 mm (linha 7), troque por G0 = 4.03 e G1 = 318.56.

const int PIN_LED = 8;
const int PIN_ADC = A0;

unsigned long t_on_ms = 1000;   // tempos de acionamento comparados: 1000, 100, 10 e 1 ms
unsigned long t_off_ms = 1000;

const float G0 = 3.2236;        // mm
const float G1 = 306.18;        // mm * contagem^(1/2)

// Sentinela para "fora de alcance": com S = 0 a reflexao nao se separa do fundo
// (sem alvo, a serial mostrou S = 0 ou 1). Isso so ocorreu alem de ~200 mm, onde o
// modelo nao foi calibrado. Uma distancia negativa e impossivel, entao -1 nao se
// confunde com uma medida e o plot_serial.py a descarta.
const float FORA_DE_ALCANCE = -1.0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(PIN_LED, HIGH);
  delay(t_on_ms);
  int adc_on = analogRead(PIN_ADC);

  digitalWrite(PIN_LED, LOW);      // o LED nao fica parado em HIGH (cerca de 38 mA)
  delay(t_off_ms);
  int adc_off = analogRead(PIN_ADC);

  int sinal = adc_off - adc_on;
  if (sinal < 0) sinal = 0;

  float mm = (sinal > 0) ? G0 + G1 / sqrt((float)sinal) : FORA_DE_ALCANCE;

  Serial.print(sinal);
  Serial.print(',');
  Serial.println(mm, 1);
}
