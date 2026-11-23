#include <stdlib.h>
#include <atomic>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP não está disponível. Selecione uma placa ESP32 compatível.
#endif

BluetoothSerial SerialBT;
const char *NOME_BLUETOOTH = "Estoura-Balao";
std::atomic<bool> bluetoothDesconectou(false);
bool bluetoothConectado = false;

// A biblioteca chama este callback em outra tarefa; a parada ocorre no loop.
void eventoBluetooth(esp_spp_cb_event_t evento, esp_spp_cb_param_t *parametro) {
  (void)parametro;
  if (evento == ESP_SPP_CLOSE_EVT) bluetoothDesconectou.store(true);
}

//velocidade (auto explicativo né meu fi)
int velocidade = 180; // Escala de 0 a 255

// Controle de conexão e segurança
unsigned long ultimoSinalControle = 0;

const unsigned long TIMEOUT_CONTROLE = 1500;

//estado atual do robô
bool roboLigado = false;

// Buffer para receber comandos do controle
char bufferComando[24];
int indiceBuffer = 0;
bool descartandoLinha = false;

//motor esquerdo
const int AIN1 = 25;
const int AIN2 = 26;

//motor direito
const int BIN1 = 27;
const int BIN2 = 14;

//acordar o driver 8833 (pino sleep)
const int SLEEP_PIN = 33;

//pino reservado para o buzzer... vai ficar legal, confia!
const int BUZZER_PIN = 18;

void setup() { 

  Serial.begin(115200);

  //config inicial
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(SLEEP_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // Mantém os motores desligados ao iniciar
  digitalWrite(SLEEP_PIN, LOW);

  pararMotores();

  SerialBT.register_callback(eventoBluetooth);
  if (!SerialBT.begin(NOME_BLUETOOTH)) {
    Serial.println("ERRO: nao foi possivel iniciar Bluetooth.");
  } else {
    Serial.println("Bluetooth pronto: Estoura-Balao");
  }

}



// Descartar dados pendentes e parte de linha após uma desconexão evita que
// uma sessão anterior seja interpretada na próxima conexão.
void verificarBluetooth() {
  bool conectado = SerialBT.hasClient();
  if (bluetoothDesconectou.exchange(false) || (bluetoothConectado && !conectado)) {
    emergencia();
    indiceBuffer = 0;
    descartandoLinha = false;
    int pendentes = SerialBT.available();
    while (pendentes-- > 0) SerialBT.read();
    Serial.println("Bluetooth desconectado: motores parados.");
  }
  bluetoothConectado = conectado;
}

void loop() {

  verificarBluetooth();

  int caracteresProcessados = 0;
  while (SerialBT.hasClient() && !bluetoothDesconectou.load() &&
         SerialBT.available() > 0 && caracteresProcessados++ < 64) {

    char caractere = SerialBT.read();

    if (caractere == '\r') {
      continue;
    }

    // Se a mensagem anterior ultrapassou o limite,
    // ignora tudo até encontrar uma quebra de linha.
    if (descartandoLinha) {

      if (caractere == '\n') {
        descartandoLinha = false;
        indiceBuffer = 0;
      }

      continue;
    }

    // Mensagem completa
    if (caractere == '\n') {

      bufferComando[indiceBuffer] = '\0';

      if (indiceBuffer > 0) {
        processarLinha(bufferComando);
      }

      indiceBuffer = 0;
    }

    // Armazena caracteres
    else if (indiceBuffer < sizeof(bufferComando) - 1) {

      bufferComando[indiceBuffer++] = caractere;

    }

    // Mensagem ultrapassou o limite
    else {

      indiceBuffer = 0;
      descartandoLinha = true;

      Serial.println("ERRO: comando muito longo");
    }
  }

  verificarBluetooth();

  // Verifica perda de comunicação
  if (roboLigado &&
      millis() - ultimoSinalControle >= TIMEOUT_CONTROLE) {

    Serial.println("FAILSAFE: conexao perdida! Parando robo.");

    emergencia();
  }
}


void tocarBip(int frequencia, int duracao) {
  tone(BUZZER_PIN, frequencia, duracao);
  delay(duracao);

  noTone(BUZZER_PIN);
  delay(80);
}

void somLigar() {
  tocarBip(500, 120);   // Grave
  tocarBip(800, 120);   // Médio
  tocarBip(1200, 180);  // Agudo
}

void somDesligar() {
  tocarBip(1200, 180);  // Agudo
  tocarBip(800, 120);   // Médio
  tocarBip(500, 120);   // Grave
}

void ligarRobo() {
  if (roboLigado) return;

  digitalWrite(SLEEP_PIN, HIGH);
  delay(1);

  roboLigado = true;
  ultimoSinalControle = millis();
  somLigar();
}

void desligarRobo() {
  if (!roboLigado) return;

  pararMotores();
  roboLigado = false;

  digitalWrite(SLEEP_PIN, LOW);

  somDesligar();
}

// Para os dois motores
void pararMotores() {

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

}


void controlarMotor(int pino1, int pino2, int valor) {
  valor = constrain(valor, -255, 255);

  if (valor > 0) {
    digitalWrite(pino2, LOW);
    analogWrite(pino1, valor);
  }
  else if (valor < 0) {
    digitalWrite(pino1, LOW);
    analogWrite(pino2, -valor);
  }
  else {
    digitalWrite(pino1, LOW);
    digitalWrite(pino2, LOW);
  }
}

void controleMotores(int esquerdo, int direito) {
  if (!roboLigado) return;

  controlarMotor(AIN1, AIN2, esquerdo);
  controlarMotor(BIN1, BIN2, direito);
}

// Avançar
void frente() {
  controleMotores(velocidade, velocidade);
}

// Recuar
void re() {
  controleMotores(-velocidade, -velocidade);
}

// Girar para a esquerda
void esquerda() {
  controleMotores(-velocidade, velocidade);
}

// Girar para a direita
void direita() {
  controleMotores(velocidade, -velocidade);
}

void definirVelocidade(int valor) {
  valor = constrain(valor, 0, 255);
  velocidade = valor;
}

//para tudo sem tocar o beep (assim n tem delay)
void emergencia() {
  pararMotores();
  roboLigado = false;
  digitalWrite(SLEEP_PIN, LOW);
}


void executarComando(char comando) {
  switch (comando) {

    case 'L':
      ligarRobo();
      break;

    case 'D':
      desligarRobo();
      break;

    case 'F':
      frente();
      break;

    case 'B':
      re();
      break;

    case 'E':
      esquerda();
      break;

    case 'R':
      direita();
      break;

    case 'S':
      pararMotores();
      break;

    case 'X':
      emergencia();
      break;

    case 'H':
      ultimoSinalControle = millis();
      break;
  }
}



void processarLinha(char *linha) {

  if (linha[0] == 'M') {

    char *cursor = linha + 1;

    if (*cursor != ',') {
      Serial.println("ERRO: comando invalido");
      return;
    }

    cursor++;

    char *fim;

    long esquerdo = strtol(cursor, &fim, 10);

    if (fim == cursor || *fim != ',') {
      Serial.println("ERRO: velocidade esquerda invalida");
      return;
    }

    cursor = fim + 1;

    long direito = strtol(cursor, &fim, 10);

    if (fim == cursor || *fim != '\0') {
      Serial.println("ERRO: velocidade direita invalida");
      return;
    }

    if (esquerdo < -255 || esquerdo > 255 ||
        direito < -255 || direito > 255) {

      Serial.println("ERRO: velocidade fora do limite");
      return;
    }

    controleMotores((int)esquerdo, (int)direito);
  }

  else if (linha[0] != '\0' && linha[1] == '\0') {
    executarComando(linha[0]);
  }
}

