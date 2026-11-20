// TP4 - Encoder e controle PID de velocidade da roda (Arduino Mega)
//
// Servo SG90 de giro contínuo no D9, encoder (A, B) em D2 e D3.
// Cada linha de dados sai em CSV pela serial (115200 baud); linhas que
// começam com '#' são eventos/condições. Use plot_serial.py para gravar
// e gerar_graficos.py para refazer os gráficos a partir dos arquivos.
//
// Comandos (terminar com Enter):
//   b            busca do neutro: 1440..1560 us em passos de 10 us
//   a            curva em malha aberta: 1000..2000 us em passos de 20 us
//   g [us]       degrau em malha aberta (neutro + us, padrão 150) p/ comparar filtros
//   e            ensaio 1: cinco patamares
//   i            ensaio 2: inversão de sentido
//   r            ensaios 1 e 2 em sequência, no mesmo registro
//   x            para tudo (pulso = neutro)
//   v <mm/s>     setpoint manual em malha fechada
//   p <us>       pulso fixo (malha aberta)
//   kp|ki|kd <x> muda ganho
//   n <us>       muda o neutro (o valor definitivo vai em NEUTRO_MEDIDO)
//   f <0|1>      filtro usado no PID: 0 = passa-baixa (EMA), 1 = média móvel
//   h            mostra a condição atual

#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

// ===================== Valores medidos na bancada =====================
// D: diâmetro externo da roda montada, na banda de contato.
// N: ticks contados pelo programa (decodificação X4) em uma volta da roda.
const float D = 46.2;                 // mm
const float N = 288.0;                // ticks / volta da roda
const float k = (PI * D) / N;         // mm/tick (= 0,5040)

// Neutro: pulso em que a roda para. Obtido com o comando "b" (passos de
// 10 us em torno de 1500 us) e confirmado com "a".
// >>> SUBSTITUIR pelo valor encontrado na bancada <<<
const int NEUTRO_MEDIDO = 1500;

// Se a velocidade medida tiver sinal oposto ao do comando (pulso > neutro
// deve dar v > 0), troque os fios A e B (D2 <-> D3) e registre no relatório.
// Esta constante faz a mesma troca em software.
const bool INVERTER_ENCODER = false;

// ===================== Hardware =====================
Encoder encoder(2, 3);
Servo meuServo;
const int pinoServo = 9;
const int pinoBotao = 4;

long lerTicks() {
  long t = encoder.read();
  return INVERTER_ENCODER ? -t : t;
}

// ===================== Pulso e controle =====================
int neutro = NEUTRO_MEDIDO;
const int PULSO_MIN = 1000;
const int PULSO_MAX = 2000;
const float U_MAX = 250.0;            // correção limitada a ±250 us em torno do neutro

// u = kp*e + ki*∫e dt - kd*dv/dt
// e em mm/s, u em us  ->  kp [us/(mm/s)], ki [us/mm], kd [us·s²/mm]
// Ganhos iniciais; os finais saem da sintonia na bancada (tabela no relatório).
float kp = 0.1;
float ki = 3.0;
float kd = 0.0;

float setpoint = 0.0;
float termoI = 0.0;
float erro = 0.0, termoP = 0.0, termoD = 0.0;
int pulso = NEUTRO_MEDIDO;

enum Modo { MALHA_FECHADA, MALHA_ABERTA };
Modo modo = MALHA_FECHADA;

// ===================== Amostragem e filtros =====================
const unsigned long PERIODO = 50;     // ms (nominal); a conta usa o Δt medido
unsigned long tempoAnterior = 0;
long ticksAnteriores = 0;

// Filtro 1: passa-baixa de primeira ordem (média móvel exponencial)
const float ALFA = 0.2;
float velEma = 0.0;

// Filtro 2: média móvel simples
const int JANELA_MM = 5;
float bufMM[JANELA_MM];
int idxMM = 0;
float somaMM = 0.0;
float velMM = 0.0;

enum Filtro { FILTRO_EMA = 0, FILTRO_MM = 1 };
Filtro filtroPID = FILTRO_EMA;        // filtro escolhido para a malha
float velPID = 0.0;

void zerarFiltros() {
  velEma = 0.0;
  for (int j = 0; j < JANELA_MM; j++) bufMM[j] = 0.0;
  idxMM = 0;
  somaMM = 0.0;
  velMM = 0.0;
  velPID = 0.0;
}

// ===================== Ensaios por patamares =====================
// aberto = false: valor é setpoint em mm/s (malha fechada)
// aberto = true : valor é a correção em us somada ao neutro (malha aberta)
struct Patamar { float valor; unsigned long duracao; const char* fase; bool aberto; };

// Patamares escolhidos dentro da curva medida em malha aberta: com ±250 us
// a roda não passa de ~80 mm/s, por isso o último patamar fica abaixo disso.
// Ajustar após levantar a curva com o comando "a".
const unsigned long T_PATAMAR = 6000;
const float V_INVERSAO = 60.0;

const Patamar ESCADA[] = {
  {0,  2000,      "repouso", false},
  {20, T_PATAMAR, "p1",      false},
  {35, T_PATAMAR, "p2",      false},
  {50, T_PATAMAR, "p3",      false},
  {60, T_PATAMAR, "p4",      false},
  {70, T_PATAMAR, "p5",      false},
  {0,  3000,      "fim",     false},
};

// Inversão direta: +V e em seguida -V, sem patamar em zero no meio,
// para a roda passar pela banda morta.
const Patamar INVERSAO[] = {
  {0,           2000,      "repouso",  false},
  {V_INVERSAO,  T_PATAMAR, "positivo", false},
  {-V_INVERSAO, T_PATAMAR, "negativo", false},
  {0,           3000,      "fim",      false},
};

const Patamar RELATORIO[] = {
  {0,           2000,      "repouso",  false},
  {20,          T_PATAMAR, "p1",       false},
  {35,          T_PATAMAR, "p2",       false},
  {50,          T_PATAMAR, "p3",       false},
  {60,          T_PATAMAR, "p4",       false},
  {70,          T_PATAMAR, "p5",       false},
  {0,           3000,      "repouso",  false},
  {V_INVERSAO,  T_PATAMAR, "positivo", false},
  {-V_INVERSAO, T_PATAMAR, "negativo", false},
  {0,           3000,      "fim",      false},
};

// Degrau em malha aberta para comparar os dois filtros no mesmo registro.
Patamar DEGRAU[] = {
  {0,   2000, "repouso", true},
  {150, 6000, "degrau",  true},
  {0,   3000, "fim",     true},
};

#define TAMANHO(a) (sizeof(a) / sizeof(a[0]))

const Patamar* ensaio = nullptr;
int ensaioTam = 0;
int ensaioIdx = 0;
unsigned long ensaioInicio = 0;
unsigned long ensaioT0 = 0;
const char* ensaioNome = "manual";
const char* fase = "manual";

// ===================== Varredura em malha aberta =====================
// Usada na busca do neutro (b) e na curva pulso x velocidade (a).
// Em cada pulso: espera o transitório (descarte) e mede a velocidade média
// pela diferença de ticks no restante da janela.
struct Varredura {
  bool ativa;
  int atual, fim, passo;
  unsigned long duracao, descarte;
  unsigned long inicio;
  bool medindo;
  long ticksIni;
  unsigned long tIni;
};
Varredura varr = {false, 0, 0, 0, 0, 0, 0, false, 0, 0};

// ===================== Botão e serial =====================
const float SETPOINTS_BOTAO[] = {0, 30, 60, -30};
const int NUM_SETPOINTS_BOTAO = TAMANHO(SETPOINTS_BOTAO);
int idxBotao = 0;
int estadoBotaoAnterior = HIGH;
unsigned long ultimoDebounce = 0;
const unsigned long atrasoDebounce = 50;

char bufferCmd[24];
int tamCmd = 0;

// =====================================================================

void definirSetpoint(float sp) {
  // A integral é zerada sempre que o setpoint muda, para não carregar o erro antigo.
  if (sp != setpoint || modo != MALHA_FECHADA) termoI = 0.0;
  setpoint = sp;
  modo = MALHA_FECHADA;
}

void imprimirCabecalho() {
  Serial.println("t_s,dt_ms,ticks,dist_mm,v_bruta,v_ema,v_mm,v_pid,setpoint,erro,P,I,D,pulso,modo,ensaio,fase");
}

void imprimirCondicao() {
  Serial.print("# cond ensaio="); Serial.print(ensaioNome);
  Serial.print(" neutro="); Serial.print(neutro);
  Serial.print(" D="); Serial.print(D, 2);
  Serial.print(" N="); Serial.print(N, 0);
  Serial.print(" k="); Serial.print(k, 4);
  Serial.print(" dt_ms="); Serial.print(PERIODO);
  Serial.print(" kp="); Serial.print(kp, 3);
  Serial.print(" ki="); Serial.print(ki, 3);
  Serial.print(" kd="); Serial.print(kd, 3);
  Serial.print(" filtro="); Serial.print(filtroPID == FILTRO_EMA ? "EMA" : "MM");
  Serial.print(" alfa="); Serial.print(ALFA, 2);
  Serial.print(" janela="); Serial.print(JANELA_MM);
  Serial.print(" U_MAX="); Serial.print(U_MAX, 0);
  if (varr.ativa) {
    Serial.print(" duracao_ms="); Serial.print(varr.duracao);
    Serial.print(" descarte_ms="); Serial.print(varr.descarte);
  }
  Serial.println();
}

void logEvento(const char* msg) {
  Serial.print("# ");
  Serial.print((millis() - ensaioT0) / 1000.0, 2);
  Serial.print(" s: ");
  Serial.print(msg);
  Serial.print(" | ensaio=");
  Serial.print(ensaioNome);
  Serial.print(" fase=");
  Serial.print(fase);
  Serial.print(" setpoint=");
  Serial.print(setpoint);
  Serial.print(" pulso=");
  Serial.println(pulso);
}

// Começa um registro novo: zera encoder, tempo e filtros (roda em repouso).
void iniciarRegistro(const char* nome, unsigned long agora) {
  encoder.write(0);
  ticksAnteriores = 0;
  ensaioT0 = agora;
  ensaioNome = nome;
  zerarFiltros();
  Serial.print("# INICIO ensaio=");
  Serial.println(nome);
  imprimirCondicao();
  imprimirCabecalho();
}

void aplicarPatamar(const Patamar& p) {
  fase = p.fase;
  if (p.aberto) {
    modo = MALHA_ABERTA;
    setpoint = 0.0;
    termoI = 0.0;
    pulso = constrain(neutro + (int)p.valor, PULSO_MIN, PULSO_MAX);
  } else {
    definirSetpoint(p.valor);
  }
}

void iniciarEnsaio(const char* nome, const Patamar* p, int tam, unsigned long agora) {
  varr.ativa = false;
  iniciarRegistro(nome, agora);
  ensaio = p;
  ensaioTam = tam;
  ensaioIdx = 0;
  ensaioInicio = agora;
  aplicarPatamar(p[0]);
  logEvento("PATAMAR");
}

void pararTudo() {
  bool estavaRodando = (ensaio != nullptr) || varr.ativa;
  ensaio = nullptr;
  varr.ativa = false;
  if (estavaRodando) {
    Serial.print("# FIM ensaio=");
    Serial.println(ensaioNome);
  }
  ensaioNome = "manual";
  fase = "manual";
  definirSetpoint(0);
  pulso = neutro;
  logEvento("PARADO");
}

void atualizarEnsaio(unsigned long agora) {
  if (ensaio == nullptr) return;
  if (agora - ensaioInicio < ensaio[ensaioIdx].duracao) return;

  ensaioInicio = agora;
  ensaioIdx++;
  if (ensaioIdx >= ensaioTam) {
    pararTudo();
  } else {
    aplicarPatamar(ensaio[ensaioIdx]);
    logEvento("PATAMAR");
  }
}

void iniciarVarredura(const char* nome, int ini, int fim, int passo,
                      unsigned long duracao, unsigned long descarte, unsigned long agora) {
  ensaio = nullptr;
  varr.ativa = true;
  varr.atual = ini;
  varr.fim = fim;
  varr.passo = passo;
  varr.duracao = duracao;
  varr.descarte = descarte;
  varr.inicio = agora;
  varr.medindo = false;
  iniciarRegistro(nome, agora);
  modo = MALHA_ABERTA;
  setpoint = 0.0;
  termoI = 0.0;
  fase = "varredura";
  pulso = constrain(ini, PULSO_MIN, PULSO_MAX);
  logEvento("PULSO");
}

void atualizarVarredura(unsigned long agora) {
  if (!varr.ativa) return;
  unsigned long decorrido = agora - varr.inicio;

  if (!varr.medindo && decorrido >= varr.descarte) {
    varr.medindo = true;
    varr.ticksIni = lerTicks();
    varr.tIni = agora;
  }
  if (decorrido < varr.duracao) return;

  long dTicks = lerTicks() - varr.ticksIni;
  float janela = (agora - varr.tIni) / 1000.0;
  Serial.print("# MEDIDA pulso="); Serial.print(pulso);
  Serial.print(" v_mm_s="); Serial.print(k * dTicks / janela, 2);
  Serial.print(" ticks="); Serial.print(dTicks);
  Serial.print(" janela_s="); Serial.println(janela, 3);

  varr.atual += varr.passo;
  if ((varr.passo > 0 && varr.atual > varr.fim) || (varr.passo < 0 && varr.atual < varr.fim)) {
    pararTudo();
    return;
  }
  varr.inicio = agora;
  varr.medindo = false;
  pulso = constrain(varr.atual, PULSO_MIN, PULSO_MAX);
  logEvento("PULSO");
}

void executarComando(char* cmd) {
  char* arg = strchr(cmd, ' ');
  bool temArg = (arg != nullptr);
  float valor = temArg ? atof(arg + 1) : 0.0;
  unsigned long agora = millis();

  if (strncmp(cmd, "kp", 2) == 0)      kp = valor;
  else if (strncmp(cmd, "ki", 2) == 0) { ki = valor; termoI = 0.0; }
  else if (strncmp(cmd, "kd", 2) == 0) kd = valor;
  else if (cmd[0] == 'v') { ensaio = nullptr; varr.ativa = false; fase = "manual"; definirSetpoint(valor); }
  else if (cmd[0] == 'n') { neutro = (int)valor; if (modo == MALHA_FECHADA && setpoint == 0) pulso = neutro; }
  else if (cmd[0] == 'p') {
    ensaio = nullptr; varr.ativa = false; fase = "malha_aberta";
    modo = MALHA_ABERTA; setpoint = 0.0; termoI = 0.0;
    pulso = constrain((int)valor, PULSO_MIN, PULSO_MAX);
  }
  else if (cmd[0] == 'f') filtroPID = (valor >= 0.5) ? FILTRO_MM : FILTRO_EMA;
  else if (cmd[0] == 'b') iniciarVarredura("neutro", 1440, 1560, 10, 3000, 1000, agora);
  else if (cmd[0] == 'a') iniciarVarredura("aberta", PULSO_MIN, PULSO_MAX, 20, 2500, 1500, agora);
  else if (cmd[0] == 'g') {
    DEGRAU[1].valor = temArg ? valor : 150.0;
    iniciarEnsaio("degrau", DEGRAU, TAMANHO(DEGRAU), agora);
  }
  else if (cmd[0] == 'e') iniciarEnsaio("escada", ESCADA, TAMANHO(ESCADA), agora);
  else if (cmd[0] == 'i') iniciarEnsaio("inversao", INVERSAO, TAMANHO(INVERSAO), agora);
  else if (cmd[0] == 'r') iniciarEnsaio("relatorio", RELATORIO, TAMANHO(RELATORIO), agora);
  else if (cmd[0] == 'x') pararTudo();
  else if (cmd[0] != 'h') { Serial.print("# comando desconhecido: "); Serial.println(cmd); return; }

  imprimirCondicao();
}

void lerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (tamCmd > 0) {
        bufferCmd[tamCmd] = '\0';
        executarComando(bufferCmd);
        tamCmd = 0;
      }
    } else if (tamCmd < (int)sizeof(bufferCmd) - 1) {
      bufferCmd[tamCmd++] = c;
    }
  }
}

void lerBotao(unsigned long agora) {
  static int estadoEstavel = HIGH;
  int leituraBotao = digitalRead(pinoBotao);

  if (leituraBotao != estadoBotaoAnterior) ultimoDebounce = agora;

  if ((agora - ultimoDebounce) > atrasoDebounce && leituraBotao != estadoEstavel) {
    estadoEstavel = leituraBotao;
    if (estadoEstavel == LOW) {
      idxBotao = (idxBotao + 1) % NUM_SETPOINTS_BOTAO;
      ensaio = nullptr;
      varr.ativa = false;
      fase = "manual";
      definirSetpoint(SETPOINTS_BOTAO[idxBotao]);
    }
  }
  estadoBotaoAnterior = leituraBotao;
}

void imprimirLinha(float t, unsigned long dtMs, long ticks, float dist, float vBruta) {
  Serial.print(t, 3);        Serial.print(',');
  Serial.print(dtMs);        Serial.print(',');
  Serial.print(ticks);       Serial.print(',');
  Serial.print(dist, 2);     Serial.print(',');
  Serial.print(vBruta, 2);   Serial.print(',');
  Serial.print(velEma, 2);   Serial.print(',');
  Serial.print(velMM, 2);    Serial.print(',');
  Serial.print(velPID, 2);   Serial.print(',');
  Serial.print(setpoint, 1); Serial.print(',');
  Serial.print(erro, 2);     Serial.print(',');
  Serial.print(termoP, 2);   Serial.print(',');
  Serial.print(termoI, 2);   Serial.print(',');
  Serial.print(termoD, 2);   Serial.print(',');
  Serial.print(pulso);       Serial.print(',');
  Serial.print(modo == MALHA_FECHADA ? "fechada" : "aberta"); Serial.print(',');
  Serial.print(ensaioNome);  Serial.print(',');
  Serial.println(fase);
}

void setup() {
  Serial.begin(115200);
  pinMode(pinoBotao, INPUT_PULLUP);

  meuServo.attach(pinoServo, PULSO_MIN, PULSO_MAX);
  pulso = neutro;
  meuServo.writeMicroseconds(pulso);

  zerarFiltros();
  tempoAnterior = millis();
  Serial.println("# comandos: b (busca neutro), a (malha aberta), g [us] (degrau filtros), e (escada), i (inversao), r (e+i), x (parar), v <mm/s>, p <us>, kp/ki/kd <x>, n <us>, f <0|1>, h");
  imprimirCondicao();
  imprimirCabecalho();
}

void loop() {
  unsigned long tempoAtual = millis();

  lerSerial();
  lerBotao(tempoAtual);
  atualizarEnsaio(tempoAtual);
  atualizarVarredura(tempoAtual);

  if (tempoAtual - tempoAnterior < PERIODO) return;

  // Δt real entre as duas leituras (não o nominal)
  unsigned long dtMs = tempoAtual - tempoAnterior;
  float dt = dtMs / 1000.0;
  tempoAnterior = tempoAtual;

  long ticksAtuais = lerTicks();
  float distancia = k * ticksAtuais;                         // s = k·ticks (sem filtro)
  float velBruta = k * (ticksAtuais - ticksAnteriores) / dt; // v = Δs/Δt
  ticksAnteriores = ticksAtuais;

  // Os dois filtros rodam sempre, para poderem ser comparados no mesmo registro.
  velEma += ALFA * (velBruta - velEma);
  somaMM += velBruta - bufMM[idxMM];
  bufMM[idxMM] = velBruta;
  idxMM = (idxMM + 1) % JANELA_MM;
  velMM = somaMM / JANELA_MM;

  float velPIDAnterior = velPID;
  velPID = (filtroPID == FILTRO_EMA) ? velEma : velMM;

  erro = setpoint - velPID;
  termoP = 0.0;
  termoD = 0.0;
  if (modo == MALHA_FECHADA) {
    float u = 0.0;
    if (setpoint == 0.0) {
      termoI = 0.0;
    } else {
      termoP = kp * erro;
      // Derivativo sobre a velocidade filtrada (não sobre o erro)
      termoD = -kd * (velPID - velPIDAnterior) / dt;

      // Integral limitada e com integração condicional (anti-windup):
      // não acumula quando a saída já está saturada no sentido do erro.
      float iNovo = constrain(termoI + ki * erro * dt, -U_MAX, U_MAX);
      float uTeste = termoP + iNovo + termoD;
      bool saturaCima  = (uTeste >  U_MAX) && (erro > 0);
      bool saturaBaixo = (uTeste < -U_MAX) && (erro < 0);
      if (!saturaCima && !saturaBaixo) termoI = iNovo;

      u = constrain(termoP + termoI + termoD, -U_MAX, U_MAX);
    }
    pulso = constrain(neutro + (int)round(u), PULSO_MIN, PULSO_MAX);
  }
  meuServo.writeMicroseconds(pulso);

  imprimirLinha((tempoAtual - ensaioT0) / 1000.0, dtMs, ticksAtuais, distancia, velBruta);
}
