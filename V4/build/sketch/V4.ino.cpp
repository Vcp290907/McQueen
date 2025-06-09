#include <Arduino.h>
#line 1 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
#include <EEPROM.h>
#include "Servo.h"
#include "Adafruit_TCS34725softi2c.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <SharpIR.h>

// Endereços na EEPROM para os parâmetros
#define EEPROM_MIN_LUX_VERDE 0
#define EEPROM_MAX_LUX_VERDE 2
#define EEPROM_MIN_C_VERDE 4
#define EEPROM_MAX_C_VERDE 6
#define EEPROM_DIFERENCA_CORES 8

#define EEPROM_C_BRANCO 10

#define EEPROM_maxLuxPreto 12
#define EEPROM_maxCPreto 14

#define EEPROM_minLuxCinza 16
#define EEPROM_maxLuxCinza 18
#define EEPROM_minCNoCinza 20
#define EEPROM_maxCNoCinza 22

#define EEPROM_minLuxVermelho 24
#define EEPROM_maxLuxVermelho 26
#define EEPROM_minCVermelho 28
#define EEPROM_maxCVermelho 30
#define EEPROM_diferencaDasCoresVermelho 32

boolean modoConfig = false;

#define SDApin1 2 //Sensor Esquerda
#define SCLpin1 3 //Sensor Esquerda
#define SDApin2 4 //Sensor Direita
#define SCLpin2 5 //Sensor Direita 

#define motorEpin 6 //Servo Esquerdo
#define motorDpin 7 //Servo Direito

#define motorGpin 8 //Servo Garra
#define motorEsqGpin 10 //Servo Garra Esquerdo
#define motorDirGpin 12 //Servo Garra Direito

#define SE 22 //Sensor Esquerdo
#define SME 23 //Sensor Meio Esquerdo
#define SM 24 //Sensor Meio
#define SMD 25 //Sensor Meio Direito
#define SD 26 //Sensor Direito

#define buzzer 32

#define SIF A4
#define SID A0
#define SIE A2

#define model1 1080
#define model2 1080
#define model3 1080

SharpIR SI_Frente(SIF, model1);
SharpIR SI_Direita(SID, model2);
SharpIR SI_Esquerda(SIE, model3);

// Inicializar duas instâncias do sensor TC34725
Adafruit_TCS34725softi2c tcs1 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, SDApin2, SCLpin2);
Adafruit_TCS34725softi2c tcs2 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, SDApin1, SCLpin1);

//Cria obejetos servos
Servo motorE;
Servo motorD;
Servo motorG;
Servo motorEsqG;
Servo motorDirG;

//Cria o objeto do giroscópio
MPU6050 giro(Wire);
unsigned long timer = 0;

//Matriz para os valores dos sensores de linha
int sensores[] = {SE,SME,SM,SMD,SD};
static int valores[5];

// Variáveis para os sensores de cor
uint16_t r1, g1, b1, c1, lux1, r2, g2, b2, c2, lux2;
long duration1, distance1, duration2, distance2;

int erro = 1;
int anguloRampaSubida, anguloRampaDescida, anguloDoReto, anguloReto;
int* sl;
boolean trava = false;

// Parâmetros gerais

int veloBaseEsq = 120;
int veloBaseDir = 60;
int pequenaCurvaLadoC = 10;
int pequenaCurvaLadoR = 5;
int veloCurva90 = 40;

int grausCurva90 = 90;
int graqusCurva180 = 175;
int grausCurva45 = 45;

int anguloAtual = 0;

int verificacaoCurvaVerde = 150; //Pulinho para ver se é curva verde
int erroGiro = 0;
int tempoDepoisDoVerde90 = 2000;
int delayCurvasverde = 0; //Verificar esse valor e onde ele é usado
int tempoAntesCurva90 = 0;
int tempoDepoisCurva90 = 1250;
int tempoDepoisDoVerde180 = 2000;
int tempoDepoisDoVerdeFalso = 500;

//Branco
int valorCnoBranco = 1000; 

// Verde mais claro
int minLuxVerde;
int maxLuxVerde;
int minCVerde;
int maxCVerde;

//Verde mais escuro
// minLuxVerde: 40
// maxLuxVerde: 100
// minCVerde: 130
// maxCVerde: 250
// diferencaDasCores: 2

// Preto
int maxLuxPreto = 150;
int maxCPreto = 400;

// Cinza
int minLuxCinza = 110;
int maxLuxCinza = 200;
int minCNoCinza = 550;
int maxCNoCinza = 650;

// Vermelho
int minLuxVermelho = 100;
int maxLuxVermelho = 150;
int minCVermelho = 400;
int maxCVermelho = 650;
int diferencaDasCoresVermelho = 40;

// Outros
int diferencaDasCores = 15;
int subtracaoSensoresCor = 100;

//PID

float Kp = 1.5;
float Ki = 0.05;
float Kd = 1.0;
int erroP = 0;
int erroAnterior = 0;
float erroI = 0;
float erroD = 0;
int ajuste = 0;

//Desvio OBJETO
int distanciaDesvio = 7;
int delayCurva1 = 3;
int delayCurva2 = 7;
int delayMeio = 1;
//************************************************************************
//*                                                                      *
//*                              Funções                                 *
//*                                                                      *
//************************************************************************

#line 175 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void tocar_buzzer(int freque, int unidades, int espera);
#line 185 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int * lerSensoresLinha();
#line 201 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void verificaVermelho();
#line 230 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
bool lerVerde1();
#line 299 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void lerCinza();
#line 326 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void testarAmplitudeSensores();
#line 429 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void resgate();
#line 509 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoAnguloZ();
#line 514 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoAnguloY();
#line 519 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void giroVerde();
#line 638 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void correcao();
#line 665 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void correcaoObjeto();
#line 684 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desvioObjeto();
#line 777 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void andarReto();
#line 1056 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void ligarGarra();
#line 1062 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarGarra();
#line 1069 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarMotorPrincipal();
#line 1073 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarMotoresGarra();
#line 1078 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void descerGarra();
#line 1084 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void garraMeio();
#line 1090 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void subirGarra();
#line 1096 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void abrirGarra();
#line 1103 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void fecharGarra();
#line 1116 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoSensoresCor();
#line 1149 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void retornoGiroscopio();
#line 1170 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void retornoSensoresLinha();
#line 1178 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void lerInfravermelho();
#line 1191 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuPrincipal();
#line 1206 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuCalculos();
#line 1215 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteVerde();
#line 1229 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteVermelho();
#line 1243 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteCinza();
#line 1256 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarVerdeMedia();
#line 1331 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarVermelhoMedia();
#line 1406 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarBrancoMedia();
#line 1440 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarCinzaMedia();
#line 1501 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void imprimirValoresEEPROM();
#line 1548 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosVerde();
#line 1585 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosVermelho();
#line 1626 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosCinza();
#line 1657 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void alterarValorEEPROM(const char* nome, int endereco, int &variavel);
#line 1670 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void processarComandoSerial();
#line 1762 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void setup();
#line 1828 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void loop();
#line 175 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void tocar_buzzer(int freque, int unidades, int espera){
  int i = 0;
  for (i; i < unidades; i++) {
    tone(buzzer, freque);
    delay(espera);
    noTone(buzzer);
    delay(espera);
  }
}

int* lerSensoresLinha() {
  for (byte i = 0; i < 5; i++) {
    valores[i] = digitalRead(sensores[i]);
  }
  return valores;
}

int mediaInfravermelhoFrente(int numLeituras = 5) {
  long soma = 0;
  for (int i = 0; i < numLeituras; i++) {
    soma += SI_Frente.distance();
    delay(10); // Pequeno delay para estabilidade
  }
  return soma / numLeituras;
}

void verificaVermelho() {
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;

  bool vermelhoDireita = (r1 > g1 && r1 > b1 && r1 - g1 > diferencaDasCoresVermelho && 
                          cc1 >= minCVermelho && cc1 <= maxCVermelho && luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho);
  bool vermelhoEsquerda = (r2 > g2 && r2 > b2 && r2 - g2 > diferencaDasCoresVermelho && 
                          cc2 >= minCVermelho && cc2 <= maxCVermelho && luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho);

  if (vermelhoDireita || vermelhoEsquerda) {
    Serial.println("VERMELHO DETECTADO!");
    while (true) {
      motorE.write(90);
      motorD.write(90);
      Serial.println("Parado por vermelho");
    }
  } else {
    Serial.println("Não é vermelho");
    lerCinza();
  }
}

bool lerVerde1() {
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;

  // Teste para preto
  if (luu1 < maxLuxPreto && cc1 < maxCPreto && luu2 < maxLuxPreto && cc2 < maxCPreto) {
    Serial.println("Preto detectado, não é verde!");
    return false;
  }

  // Teste para verde
  if (cc1 <= valorCnoBranco || cc2 <= valorCnoBranco) {
    bool verdeDireita = (g1 > r1 && g1 > b1 && g1 - r1 > diferencaDasCores && cc1 >= minCVerde && cc1 <= maxCVerde && luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    bool verdeEsquerda = (g2 > r2 && g2 > b2 && g2 - r2 > diferencaDasCores && cc2 >= minCVerde && cc2 <= maxCVerde && luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    bool verdeAmbos = (abs(cc1 - cc2) < subtracaoSensoresCor) && verdeDireita && verdeEsquerda;

    Serial.print("Verde Direita: "); Serial.println(verdeDireita);
    Serial.print("Verde Esquerda: "); Serial.println(verdeEsquerda);
    Serial.print("Verde Ambos: "); Serial.println(verdeAmbos);
    Serial.print("cc1: "); Serial.print(cc1); Serial.print(" | cc2: "); Serial.println(cc2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("R1: "); Serial.print(r1); Serial.print(" | R2: "); Serial.println(r2);
    Serial.print("G1: "); Serial.print(g1); Serial.print(" | G2: "); Serial.println(g2);
    Serial.print("B1: "); Serial.print(b1); Serial.print(" | B2: "); Serial.println(b2);
    Serial.print("C1: "); Serial.print(c1); Serial.print(" | C2: "); Serial.println(c2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    Serial.print("g1 > r1 && g1 > b1: "); Serial.println(g1 > r1 && g1 > b1);
    Serial.print("g1 - r1 > diferencaDasCores: "); Serial.println(g1 - r1 > diferencaDasCores);
    Serial.print("g2 > r2 && g2 > b2: "); Serial.println(g2 > r2 && g2 > b2);
    Serial.print("g2 - r2 > diferencaDasCores: "); Serial.println(g2 - r2 > diferencaDasCores);
    Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    Serial.print("Abs(cc1 - cc2) < subtracaoSensoresCor: "); Serial.println(abs(cc1 - cc2) < subtracaoSensoresCor);

    if (verdeAmbos) {
      Serial.println("Verde detectado nos dois sensores!");
      return true;
    } else if (verdeDireita) {
      Serial.println("Verde na Direita!");
      Serial.print("Lux: "); Serial.print(lux1); Serial.print(" | R: "); Serial.print(r1);
      Serial.print(" | G: "); Serial.print(g1); Serial.print(" | B: "); Serial.print(b1);
      Serial.print(" | C: "); Serial.print(c1); Serial.println();
      return true;
    } else if (verdeEsquerda) {
      Serial.println("Verde na Esquerda!");
      Serial.print("Lux: "); Serial.print(lux2); Serial.print(" | R: "); Serial.print(r2);
      Serial.print(" | G: "); Serial.print(g2); Serial.print(" | B: "); Serial.print(b2);
      Serial.print(" | C: "); Serial.print(c2); Serial.println();
      return true;
    }
  }

  Serial.println("Não é verde!");
  return false;
}

void lerCinza() { //
  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;

  if((cc1 >= minCNoCinza && cc1 <= maxCNoCinza && luu1 >= minLuxCinza && luu1 <= maxLuxCinza) ||
     (cc2 >= minCNoCinza && cc2 <= maxCNoCinza && luu2 >= minLuxCinza && luu2 <= maxLuxCinza)) {
    Serial.println("Cinza detectado!");
    resgate();
  } else {
    Serial.println("Não é cinza!");
  }
}

int amplitudeSensor(SharpIR &sensor, int numLeituras = 20) {
  int minValor = 10000;
  int maxValor = 0;
  for (int i = 0; i < numLeituras; i++) {
    int leitura = sensor.distance(); 
    if (leitura < minValor) minValor = leitura;
    if (leitura > maxValor) maxValor = leitura;
    delay(10); // Pequeno delay para estabilidade
  }
  return maxValor - minValor;
}

void testarAmplitudeSensores() {
  int ampFrente = amplitudeSensor(SI_Frente);
  int ampEsquerda = amplitudeSensor(SI_Esquerda);
  int ampDireita = amplitudeSensor(SI_Direita);

  Serial.print("Amplitude Frente: "); Serial.println(ampFrente);
  Serial.print("Amplitude Esquerda: "); Serial.println(ampEsquerda);
  Serial.print("Amplitude Direita: "); Serial.println(ampDireita);

  // Se amplitude for maior que um limiar (ex: 20 cm), provavelmente está "no vazio"
  if (ampFrente > 17) Serial.println("Sensor da frente está no vazio!");
  if (ampEsquerda > 17) Serial.println("Sensor da esquerda está no vazio!");
  if (ampDireita > 17) Serial.println("Sensor da direita está no vazio!");

  Serial.println("---------------------");
}

bool vazioFrente = false;
bool vazioEsquerda = false;
bool vazioDireita = false;

void sensoresNoVazio (int limiar = 17) {
  vazioFrente   = amplitudeSensor(SI_Frente)   > limiar;
  vazioEsquerda = amplitudeSensor(SI_Esquerda) > limiar;
  vazioDireita  = amplitudeSensor(SI_Direita)  > limiar;
}

bool detectaMudancaBrusca(SharpIR &sensor, int limiar = 10, int numLeituras = 7, const char* nomeSensor = "") {
  int leituras[numLeituras];
  int soma = 0;
  int validas = 0;

  // Coleta leituras válidas (< 50)
  for (int i = 0; i < numLeituras; i++) {
    int leitura = sensor.distance();
    if (leitura < 50) {
      leituras[i] = leitura;
      soma += leitura;
      validas++;
    } else {
      leituras[i] = 50; // Marca como inválida
    }
    delay(10);
  }

  if (validas < 3) return false; // Não tem leituras suficientes

  float media = (float)soma / validas;
  int consecutivas = 0;

  for (int i = 0; i < numLeituras; i++) {
    Serial.print("[");
    Serial.print(nomeSensor);
    Serial.print("] Leitura ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(leituras[i]);
    Serial.print(" | Média: ");
    Serial.print(media);
    Serial.print(" | Diferença: ");
    Serial.println(media - leituras[i]);

    if (leituras[i] < 50 && (media - leituras[i]) > limiar) {
      consecutivas++;
      if (consecutivas >= 2) {
        Serial.print("Bolinha detectada no sensor ");
        Serial.println(nomeSensor);
        return true;
      }
    } else {
      consecutivas = 0;
    }
  }
  return false;
}

void monitorarLateraisComCorrecao(unsigned long tempo_ms = 3000, int limiar = 5, int numLeituras = 1) {
  unsigned long inicio = millis();
  bool detectouBrusca = false;

  while (millis() - inicio < tempo_ms) {
    correcaoObjeto();

    // Detecta mudança brusca na esquerda
    if (detectaMudancaBrusca(SI_Esquerda, limiar, 5, "Esquerda")) {
      Serial.println("Mudança brusca na ESQUERDA (possível bolinha)!");
      tocar_buzzer(1000, 1, 500);
      detectouBrusca = true;
      break;
    }
    if (detectaMudancaBrusca(SI_Direita, limiar, 5, "Direita")) {
      Serial.println("Mudança brusca na DIREITA (possível bolinha)!");
      tocar_buzzer(1000, 1, 500);
      detectouBrusca = true;
      break;
    }
  }

  if (!detectouBrusca) {
    Serial.println("Nenhuma mudança brusca detectada nas laterais durante o período.");
  }
}

void resgate(){
  Serial.println("Resgate iniciado!");
  motorE.write(90);
  motorD.write(90);
  tocar_buzzer(500, 2, 100);

  ligarGarra();
  abrirGarra();
  descerGarra();
  delay(500);

  Serial.print("Angulo Reto: "); Serial.print(anguloReto); Serial.print(" | Angulo Atual: "); Serial.println(retornoAnguloZ());

  unsigned long startTime = millis();
  while(startTime + 6 * 1000 > millis()) {
    correcaoObjeto();
    giro.update();
  }

  motorD.write(90);
  motorE.write(90);

  sensoresNoVazio();

  if (!vazioEsquerda && !vazioDireita) {
    int distEsq = SI_Esquerda.distance();
    int distDir = SI_Direita.distance();

    Serial.print("Distância Esquerda: "); Serial.println(distEsq);
    Serial.print("Distância Direita: "); Serial.println(distDir);

    if (distEsq < distDir) {
      motorE.write(veloBaseDir);
      motorD.write(veloBaseDir);
      while (((anguloReto - grausCurva90) > retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
      }
      anguloReto = anguloReto - grausCurva90;

    } else {
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseEsq);
      while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
      }
      anguloReto = anguloReto + grausCurva90;

    }
  } else {
    Serial.println("Um dos lados está no vazio, lógica a definir...");
    while (true)
    {
      motorD.write(90);
      motorE.write(90);
      tocar_buzzer(1000, 1, 500);
    }
    
  }

  motorD.write(veloBaseEsq);
  motorE.write(veloBaseDir);
  delay(2000);

  anguloReto = retornoAnguloZ();
  Serial.print("Angulo Reto atualizado: "); Serial.println(anguloReto);
  
  monitorarLateraisComCorrecao(10000, 4, 10);

  while (true)
  {
    motorD.write(90);
    motorE.write(90);
  }
  
}

int retornoAnguloZ(){
  giro.update();
  return giro.getAngleZ();
}

int retornoAnguloY(){
  giro.update();
  return giro.getAngleY();
}

void giroVerde() {
  Serial.println("Giro Verde");
  motorE.write(veloBaseEsq);
  motorD.write(veloBaseDir);
  giro.update();

  delay(verificacaoCurvaVerde);

  motorE.write(90);
  motorD.write(90);

  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;
  // Teste para verde

  if (cc1 <= valorCnoBranco || cc2 <= valorCnoBranco) {
    bool verdeDireita = (g1 > r1 && g1 > b1 && g1 - r1 > diferencaDasCores && cc1 >= minCVerde && cc1 <= maxCVerde && luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    bool verdeEsquerda = (g2 > r2 && g2 > b2 && g2 - r2 > diferencaDasCores && cc2 >= minCVerde && cc2 <= maxCVerde && luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    bool verdeAmbos = (abs(cc1 - cc2) < subtracaoSensoresCor) && verdeDireita && verdeEsquerda;

    // Serial.print("Verde Direita: "); Serial.println(verdeDireita);
    // Serial.print("Verde Esquerda: "); Serial.println(verdeEsquerda);
    // Serial.print("Verde Ambos: "); Serial.println(verdeAmbos);
    // Serial.print("cc1: "); Serial.print(cc1); Serial.print(" | cc2: "); Serial.println(cc2);
    // Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    // Serial.print("R1: "); Serial.print(r1); Serial.print(" | R2: "); Serial.println(r2);
    // Serial.print("G1: "); Serial.print(g1); Serial.print(" | G2: "); Serial.println(g2);
    // Serial.print("B1: "); Serial.print(b1); Serial.print(" | B2: "); Serial.println(b2);
    // Serial.print("C1: "); Serial.print(c1); Serial.print(" | C2: "); Serial.println(c2);
    // Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    // Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    // Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    // Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    // Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    // Serial.print("g1 > r1 && g1 > b1: "); Serial.println(g1 > r1 && g1 > b1);
    // Serial.print("g1 - r1 > diferencaDasCores: "); Serial.println(g1 - r1 > diferencaDasCores);
    // Serial.print("g2 > r2 && g2 > b2: "); Serial.println(g2 > r2 && g2 > b2);
    // Serial.print("g2 - r2 > diferencaDasCores: "); Serial.println(g2 - r2 > diferencaDasCores);
    // Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    // Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    // Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    // Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    // Serial.print("Abs(cc1 - cc2) < subtracaoSensoresCor: "); Serial.println(abs(cc1 - cc2) < subtracaoSensoresCor);

    if (verdeAmbos) {

      Serial.print("VERDE!! Curva 180°"); Serial.print(" | Angulo Reto: "); Serial.print(anguloReto); Serial.print(" | Angulo Atual: "); Serial.println(retornoAnguloZ());
      motorE.write(veloBaseDir - pequenaCurvaLadoC);
      motorD.write(veloBaseDir - pequenaCurvaLadoC);
      tocar_buzzer(1000, 3, 100);
      while (anguloReto - graqusCurva180 < retornoAnguloZ()) {
        giro.update();
        Serial.print("VERDE!! Fazendo curva 180° | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - graqusCurva180);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisDoVerde180);

      anguloReto = anguloReto - graqusCurva180;
      erroI = 0;

    } else if (verdeDireita) {
      Serial.println("Verde na Direita!");
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(delayCurvasverde);

      motorE.write(veloBaseDir);
      motorD.write(veloBaseDir);
      while (anguloReto - grausCurva90 < retornoAnguloZ()) {
        giro.update();
        Serial.print("VERDE!! Fazendo curva para a direita | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisDoVerde90);

      anguloReto = anguloReto - grausCurva90;
      erroI = 0;

    } else if (verdeEsquerda) {
      Serial.println("Verde na Esquerda!");
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(delayCurvasverde);

      anguloReto = retornoAnguloZ();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseEsq);
      while (anguloReto + grausCurva90 > retornoAnguloZ()) {
        giro.update();
        Serial.print("VERDE!! Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisDoVerde90);

      anguloReto = anguloReto + grausCurva90;
      erroI = 0;

    }
  }
}

void correcao() {
  sl = lerSensoresLinha();
  anguloAtual = retornoAnguloZ();
  if (sl[0] == 0 || sl[1] == 0 || sl[2] == 0 || sl[3] == 0 || sl[4] == 0) {
    return;
  } else {
    if (anguloReto - erro > anguloAtual) {
      verificaVermelho();
      motorE.write(100);
      motorD.write(90);
      Serial.println("Correção1");
    }
    else if (anguloReto + erro < anguloAtual) {
      verificaVermelho();
      motorE.write(90);
      motorD.write(80);
      Serial.println("Correção2");
    }
    else if (abs(anguloReto - anguloAtual) <= erro) {
      verificaVermelho();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      Serial.println("Correção3");
    }
  }
}

void correcaoObjeto() {
  anguloAtual = retornoAnguloZ();
  if (anguloReto - erro > anguloAtual) {
    motorE.write(100);
    motorD.write(90);
    Serial.println("Objeto Correção1");
  }
  else if (anguloReto + erro < anguloAtual) {
    motorE.write(90);
    motorD.write(80);
    Serial.println("Objeto Correção2");
  }
  else if (abs(anguloReto - anguloAtual) <= erro) {
    motorE.write(veloBaseEsq + veloCurva90);
    motorD.write(veloBaseDir - veloCurva90);
    Serial.println("Objeto Correção3");
  }
}

void desvioObjeto() {
  if (SI_Frente.distance() < distanciaDesvio) {

    //*************************/
    // Curva para a esquerda 1
    //*************************/

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);

    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      giro.update();
      Serial.print("OBJETO 1 Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
    }
    motorE.write(veloBaseEsq + veloCurva90);
    motorD.write(veloBaseDir - veloCurva90);

    anguloReto = anguloReto + grausCurva90;
    
    unsigned long startTime = millis();
    while (millis() - startTime < delayCurva1 * 1000) {
      Serial.print("Aguardando curva 1 | Tempo decorrido: "); Serial.println(millis() - startTime);
      Serial.print("Tempo restante: "); Serial.println((delayCurva1 * 1000) - (millis() - startTime));
      correcaoObjeto();
    }

    //************************/
    // Curva para a direita 1
    //************************/

    Serial.println("Curva para a direita 1");

    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      giro.update();
      Serial.print("OBJETO 2 Fazendo curva para a direita | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
    }
    motorE.write(veloBaseEsq + veloCurva90);
    motorD.write(veloBaseDir - veloCurva90);

    anguloReto = anguloReto - grausCurva90;

    startTime = millis();
    while (millis() - startTime < delayCurva2 * 1000) {
      correcaoObjeto();
    }

    //************************/
    // Curva para a direita 2
    //************************/

    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      giro.update();
      Serial.print("OBJETO 3 Fazendo curva para a direita | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
    }
    motorE.write(veloBaseEsq + veloCurva90);
    motorD.write(veloBaseDir - veloCurva90);

    anguloReto = anguloReto - grausCurva90;
    
    sl = lerSensoresLinha();
    while (sl[2] == 1) {
      correcaoObjeto();
      sl = lerSensoresLinha();
    }
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);

    delay(delayMeio*1000);

    //************************/
    // Curva para a esquerda 2
    //************************/

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);

    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      giro.update();
      Serial.print("OBJETO 4 Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
    }
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);

    anguloReto = anguloReto + grausCurva90;
  }
}

static bool estavaDesalinhado = true;

void andarReto() {
  giro.update();
  sl = lerSensoresLinha();

  int combinacaoSensores = sl[0] * 16 + sl[1] * 8 + sl[2] * 4 + sl[3] * 2 + sl[4];

  switch (combinacaoSensores) {

    case 0b11011:
      anguloAtual = retornoAnguloZ();
      erroP = anguloAtual - anguloReto;

      // Integral
      erroI += erroP;
      erroI = constrain(erroI, -100, 100);

      // Derivativo
      erroD = erroP - erroAnterior;
      erroAnterior = erroP;

      // PID
      ajuste = Kp * erroP + Ki * erroI + Kd * erroD;
      ajuste = abs(ajuste);
      ajuste = constrain(ajuste, 0, 40);

      if (erroP > 0) {
        motorE.write(veloBaseEsq - ajuste);
        motorD.write(veloBaseDir + (ajuste / 3));
      } else if (erroP < 0) {
        motorE.write(veloBaseEsq - (ajuste / 3));
        motorD.write(veloBaseDir + ajuste);
      } else {
        motorE.write(veloBaseEsq);
        motorD.write(veloBaseDir);
      }

      Serial.print("Andando reto | Angulo Atual: "); Serial.print(anguloAtual);
      Serial.print(" | Angulo Reto: "); Serial.print(anguloReto);
      Serial.print(" | Ajuste: "); Serial.print(ajuste);
      Serial.print(" | P: "); Serial.print(Kp * erroP);
      Serial.print(" | I: "); Serial.print(Ki * erroI);
      Serial.print(" | D: "); Serial.println(Kd * erroD);

      if (estavaDesalinhado) {
        anguloReto = (anguloAtual + anguloReto) / 2;
        erroI = 0;
        Serial.print("Novo angulo RETO (centralizado): "); Serial.println(anguloReto);
        estavaDesalinhado = false;
      }
      
      desvioObjeto();

      break;

    case 0b10011: // Pequena curva esquerda
      motorE.write(veloBaseEsq + pequenaCurvaLadoC);
      motorD.write(veloBaseDir - pequenaCurvaLadoR);
      Serial.println("Pequena curva esquerda");
      estavaDesalinhado = true;
      desvioObjeto();
      break;

    case 0b00011: // Curva falsa ou verde
      Serial.println("Curva falsa OU verde");
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      delay(tempoDepoisDoVerdeFalso);
      break;

    case 0b00111: // Curva esquerda
      Serial.println("Curva esquerda");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseEsq);
      while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);
      anguloReto = anguloReto + grausCurva90;
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);
      break;

    case 0b11001: // Pequena curva direita
      Serial.println("Pequena curva direita");
      motorE.write(veloBaseEsq + pequenaCurvaLadoR);
      motorD.write(veloBaseDir - pequenaCurvaLadoC);
      estavaDesalinhado = true;
      desvioObjeto();
      break;

    case 0b11000: // Curva falsa ou verde
      Serial.println("Curva falsa OU verde");
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      delay(tempoDepoisDoVerdeFalso);
      break;

    case 0b11100: // Curva direita
      Serial.println("Curva direita");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      motorE.write(veloBaseDir);
      motorD.write(veloBaseDir);
      while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a direita | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = anguloReto - grausCurva90;
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);

      break;

    case 0b00000: // Início da pista ou encruzilhada REVER 
      motorE.write(90);
      motorD.write(90);
      Serial.println("Início da pista, ou encruzilhada");
      giroVerde();
      break;

    case 0b10111: // Saiu do principal, esquerda
      Serial.println("Saiu do principal, esquerda");
      sl = lerSensoresLinha();
      motorE.write(veloBaseEsq);
      motorD.write(80);
      sl = lerSensoresLinha();
      desvioObjeto();
      break;

    case 0b11101: // Saiu do principal, direita
      Serial.println("Saiu do principal, direita");
      motorE.write(100);
      motorD.write(veloBaseDir);
      sl = lerSensoresLinha();
      desvioObjeto();
      break;

    case 0b11111: // Branco, final ou resgate
      Serial.println("Branco");
      correcao();
      break;

    case 0b00100: // T
      Serial.println("T");
      giroVerde();
      break;

    case 0b01111: // Recuperando a linha, esquerda
      Serial.println("Recuperando a linha, esquerda");
      while(sl[2] == 1 && sl[1] == 1 && sl[0] == 0){
        sl = lerSensoresLinha();
        motorE.write(110);
        motorD.write(90);
      }
      anguloAtual = retornoAnguloZ();
      while(anguloAtual >= anguloReto){
        anguloAtual = retornoAnguloZ();
        Serial.print("Angulo Atual: "); Serial.println(anguloAtual);
        Serial.print("Angulo reto"); Serial.println(anguloReto);
        motorE.write(90);
        motorD.write(70);
      }
      break;

    case 0b11110: // Recuperando a linha, direita
      Serial.println("Recuperando a linha, direita");
      while(sl[4] == 0 && sl[3] == 1 && sl[2] == 1){
        sl = lerSensoresLinha();
        motorE.write(90);
        motorD.write(70);
      }
      anguloAtual = retornoAnguloZ();
      while(anguloAtual <= anguloReto){
        anguloAtual = retornoAnguloZ();
        Serial.print("Angulo Atual: "); Serial.println(anguloAtual);
        Serial.print("Angulo reto"); Serial.println(anguloReto);
        motorE.write(110);
        motorD.write(90);
      }
      break;

    case 0b10100:
      Serial.println("Curva direita 2");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      motorE.write(veloBaseDir);
      motorD.write(veloBaseDir);
      while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a direita 2 | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = anguloReto - grausCurva90;
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);
      break;

    case 0b00101:
      Serial.println("Curva esquerda 2");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseEsq);
      while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
        giro.update();
        sl = lerSensoresLinha();
        Serial.print("Fazendo curva para a esquerda 2 | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
      }

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = anguloReto + grausCurva90;
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);
      break;

    default:
      giro.update();
      Serial.println("Bugou ai kkk");
      tocar_buzzer(1000, 1, 100);
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      retornoSensoresLinha();
      break;
  }
}

//******************************************************************************
//*                                                                            *
//*                              Funções GARRA                                 *
//*                                                                            *
//******************************************************************************

void ligarGarra(){
  motorG.attach(motorGpin);
  motorEsqG.attach(motorEsqGpin);
  motorDirG.attach(motorDirGpin);
}

void desligarGarra(){
  motorG.detach();
  motorEsqG.detach();
  motorDirG.detach();
  delay(250);
}

void desligarMotorPrincipal() {
  motorG.detach();
}

void desligarMotoresGarra() {
  motorEsqG.detach();
  motorDirG.detach();
}

void descerGarra() {
  Serial.println("Descendo Garra");
  motorG.write(175); // Posição de descida
  delay(2000); // Tempo para descer
}

void garraMeio(){
  Serial.println("Garra Meio");
  motorG.write(90); // Posição de meio
  delay(2000); // Tempo para meio
}

void subirGarra() {
  Serial.println("Subindo Garra");
  motorG.write(15); // Posição de subida
  delay(2000); // Tempo para subir
}

void abrirGarra() {
  Serial.println("Abrindo Garra");
  motorEsqG.write(0); // Posição de abertura
  motorDirG.write(180); // Posição de abertura
  delay(1000); // Tempo para abrir
}

void fecharGarra() {
  Serial.println("Fechando Garra");
  motorEsqG.write(110); // Posição de fechamento
  motorDirG.write(70); // Posição de fechamento
  delay(1000); // Tempo para fechar
}

//******************************************************************************
//*                                                                            *
//*                              Funções DEBUG                                 *
//*                                                                            *
//******************************************************************************

int retornoSensoresCor(){ //Função de retorno para analizar apenas
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  Serial.println("*Sensor 01 Dirieta*");
  Serial.print("Lux: "); Serial.print(lux1, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r1, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g1, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b1, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c1, DEC); Serial.print(" ");
  Serial.println(" ");

  //Serial.print("Sensor 2: Color Temp: "); Serial.print(colorTemp2, DEC); Serial.print(" K - "); da bug nem tenta kkk
  Serial.println("*Sensor 02 Esquerda*");
  Serial.print("Lux: "); Serial.print(lux2, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r2, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g2, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b2, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c2, DEC); Serial.print(" ");
  Serial.println(" ");

  Serial.println("---Diferença Esquerda---");
  Serial.print("G - R = ");Serial.println(g2 - r2);
  Serial.print("G - B = ");Serial.println(g2 - b2);

  Serial.println("---Diferença Direita---");
  Serial.print("G - R = ");Serial.println(g1 - r1);
  Serial.print("G - B = ");Serial.println(g1 - b1);
  delay(250);
}

void retornoGiroscopio(){
  static int valores[3];
  giro.update();
  valores[0] = giro.getAngleX();
  valores[1] = giro.getAngleY();
  valores[2] = giro.getAngleZ();

  for(int i=0;i<3;i++){
    if(i==0){
      Serial.print("angleX : ");
      Serial.print(valores[i]);
    }else if(i==1){
      Serial.print("\tangleY : ");
      Serial.print(valores[i]);
    }else if(i==2){
      Serial.print("\tangleZ : ");
      Serial.println(valores[i]);
    }
  }
}

void retornoSensoresLinha(){
  sl = lerSensoresLinha();
  for(byte i = 0; i < 5; i++){
    Serial.print(sl[i]);
  }
  Serial.println(" ");
}

void lerInfravermelho(){
  int d1 = SI_Frente.distance();
  int d2 = SI_Esquerda.distance();
  int d3 = SI_Direita.distance();

  Serial.print("Distancia Frente: "); Serial.print(d1); Serial.print("cm - ");
  Serial.print("Distancia Esquerda: "); Serial.print(d2); Serial.print("cm - ");
  Serial.print("Distancia Direita: "); Serial.print(d3); Serial.print("cm - ");
  Serial.println(" ");
}

int menuAtual = 1;

void exibirMenuPrincipal() {
  Serial.println("=== Menu 01 ===");
  Serial.println("1 - Ler sensores de cor");
  Serial.println("2 - Calculos");
  Serial.println("3 - Calibrar Verde");
  Serial.println("4 - Calibrar Vermelho");
  Serial.println("5 - Calibrar Cinza");
  Serial.println("6 - Ajustar Branco Manual");
  Serial.println("7 - Ajustar Verde Manual");
  Serial.println("8 - Ajustar Vermelho Manual");
  Serial.println("9 - Exibir valores da EEPROM");
  Serial.println("0 - Sair");
  Serial.println("================");
}

void exibirMenuCalculos() {
  Serial.println("=== Menu 02 - Calculos ===");
  Serial.println("1 - Calculos do Verde");
  Serial.println("2 - Calculos do Vermelho");
  Serial.println("3 - Calculos do Cinza");
  Serial.println("4 - Voltar");
  Serial.println("================");
}

void exibirMenuAjusteVerde() {
  Serial.println("=== Menu 03 - Ajustar verde manual ===");
  Serial.println("1 - Ler sensores de cor");
  Serial.println("2 - Calculo do verde");
  Serial.println("3 - maxLuxVerde");
  Serial.println("4 - minLuxVerde");
  Serial.println("5 - maxCVerde");
  Serial.println("6 - minCVerde");
  Serial.println("7 - Valores EEPROM Verde");
  Serial.println("8 - Alterar diferenca cores verde");
  Serial.println("9 - Voltar");
  Serial.println("================");
}

void exibirMenuAjusteVermelho() {
  Serial.println("=== Menu 04 - Ajustar vermelho manual ===");
  Serial.println("1 - Ler sensores de cor");
  Serial.println("2 - Calculo do Vermelho");
  Serial.println("3 - maxLuxVermelho");
  Serial.println("4 - minLuxVermelho");
  Serial.println("5 - maxCVermelho");
  Serial.println("6 - minCVermelho");
  Serial.println("7 - Valores EEPROM Vermelho");
  Serial.println("8 - DiferencaDasCoresVermelho");
  Serial.println("9 - Voltar");
  Serial.println("================");
}

void exibirMenuAjusteCinza() {
  Serial.println("=== Menu 05 - Ajustar cinza manual ===");
  Serial.println("1 - Ler sensores de cor");
  Serial.println("2 - Calculo do Cinza");
  Serial.println("3 - maxLuxCinza");
  Serial.println("4 - minLuxCinza");
  Serial.println("5 - maxCCinza");
  Serial.println("6 - minCCinza");
  Serial.println("7 - Valores EEPROM Cinza");
  Serial.println("8 - Voltar");
  Serial.println("================");
}

void calibrarVerdeMedia() {
  Serial.println("=== Calibração do Verde (Média) ===");
  Serial.println("Coloque o sensor sobre o VERDE e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read(); // Limpa o buffer

  const int amostras = 20;
  long somaLux = 0, somaC = 0, somaDif = 0;
  int luxArray[amostras], cArray[amostras], difArray[amostras];

  for (int i = 0; i < amostras; i++) {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);

    int cVerde = (int)c1;
    int luxVerde = (int)lux1;
    int difVerde = (int)(g1 - r1);

    somaLux += luxVerde;
    somaC += cVerde;
    somaDif += difVerde;
    luxArray[i] = luxVerde;
    cArray[i] = cVerde;
    difArray[i] = difVerde;

    Serial.print("Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxVerde);
    Serial.print(" | C: "); Serial.print(cVerde);
    Serial.print(" | G-R: "); Serial.println(difVerde);

    delay(150);
  }

  int mediaLux = somaLux / amostras;
  int mediaC = somaC / amostras;
  int mediaDif = somaDif / amostras;

  // Calcula min/max com margem de 20% (ajuste conforme necessário)
  int minLux = mediaLux * 0.7;
  int maxLux = mediaLux * 1.3;
  int minC = mediaC * 0.7;
  int maxC = mediaC * 1.3;

  // Ajuste para diferencaDasCores ser menor que a real
  int margem = 15; // ajuste conforme necessário
  int diferencaAjustada = mediaDif - margem;
  if (diferencaAjustada < 1) diferencaAjustada = 1;

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxVerde: "); Serial.println(mediaLux);
  Serial.print("mediaCVerde: "); Serial.println(mediaC);
  Serial.print("mediaDifVerde (G-R): "); Serial.println(mediaDif);
  Serial.print("diferencaDasCores usada: "); Serial.println(diferencaAjustada);
  Serial.print("minLuxVerde (80%): "); Serial.println(minLux);
  Serial.print("maxLuxVerde (120%): "); Serial.println(maxLux);
  Serial.print("minCVerde (80%): "); Serial.println(minC);
  Serial.print("maxCVerde (120%): "); Serial.println(maxC);

  // Salva na EEPROM
  EEPROM.put(EEPROM_MIN_LUX_VERDE, minLux);
  EEPROM.put(EEPROM_MAX_LUX_VERDE, maxLux);
  EEPROM.put(EEPROM_MIN_C_VERDE, minC);
  EEPROM.put(EEPROM_MAX_C_VERDE, maxC);
  EEPROM.put(EEPROM_DIFERENCA_CORES, diferencaAjustada);

  // Atualiza variáveis globais
  minLuxVerde = minLux;
  maxLuxVerde = maxLux;
  minCVerde = minC;
  maxCVerde = maxC;
  diferencaDasCores = diferencaAjustada;

  Serial.println("Valores salvos na EEPROM!");
}

void calibrarVermelhoMedia() {
  Serial.println("=== Calibração do Vermelho (Média) ===");
  Serial.println("Coloque o sensor sobre o VERMELHO e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read(); // Limpa o buffer

  const int amostras = 20;
  long somaLux = 0, somaC = 0, somaDif = 0;
  int luxArray[amostras], cArray[amostras], difArray[amostras];

  for (int i = 0; i < amostras; i++) {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);

    int cVermelho = (int)c1;
    int luxVermelho = (int)lux1;
    int difVermelho = (int)(r1 - g1);

    somaLux += luxVermelho;
    somaC += cVermelho;
    somaDif += difVermelho;
    luxArray[i] = luxVermelho;
    cArray[i] = cVermelho;
    difArray[i] = difVermelho;

    Serial.print("Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxVermelho);
    Serial.print(" | C: "); Serial.print(cVermelho);
    Serial.print(" | R-G: "); Serial.println(difVermelho);

    delay(150);
  }

  int mediaLux = somaLux / amostras;
  int mediaC = somaC / amostras;
  int mediaDif = somaDif / amostras;

  // Calcula min/max com margem de 20%
  int minLux = mediaLux * 0.6;
  int maxLux = mediaLux * 1.4;
  int minC = mediaC * 0.6;
  int maxC = mediaC * 1.4;

  // Ajuste para diferencaDasCoresVermelho ser menor que a real
  int margem = 15; // ajuste conforme necessário
  int diferencaAjustada = mediaDif - margem;
  if (diferencaAjustada < 1) diferencaAjustada = 1;

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxVermelho: "); Serial.println(mediaLux);
  Serial.print("mediaCVermelho: "); Serial.println(mediaC);
  Serial.print("mediaDifVermelho (R-G): "); Serial.println(mediaDif);
  Serial.print("diferencaDasCoresVermelho usada: "); Serial.println(diferencaAjustada);
  Serial.print("minLuxVermelho (80%): "); Serial.println(minLux);
  Serial.print("maxLuxVermelho (120%): "); Serial.println(maxLux);
  Serial.print("minCVermelho (80%): "); Serial.println(minC);
  Serial.print("maxCVermelho (120%): "); Serial.println(maxC);

  // Salva na EEPROM
  EEPROM.put(EEPROM_minLuxVermelho, minLux);
  EEPROM.put(EEPROM_maxLuxVermelho, maxLux);
  EEPROM.put(EEPROM_minCVermelho, minC);
  EEPROM.put(EEPROM_maxCVermelho, maxC);
  EEPROM.put(EEPROM_diferencaDasCoresVermelho, diferencaAjustada);

  // Atualiza variáveis globais
  minLuxVermelho = minLux;
  maxLuxVermelho = maxLux;
  minCVermelho = minC;
  maxCVermelho = maxC;
  diferencaDasCoresVermelho = diferencaAjustada;

  Serial.println("Valores salvos na EEPROM!");
}

void calibrarBrancoMedia() {
  Serial.println("=== Calibração do Branco (Média) ===");
  Serial.println("Coloque o sensor sobre o BRANCO e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read(); // Limpa o buffer

  const int amostras = 20;
  long somaC = 0;
  int cArray[amostras];

  for (int i = 0; i < amostras; i++) {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    int cBranco = (int)c1;
    somaC += cBranco;
    cArray[i] = cBranco;

    Serial.print("Amostra "); Serial.print(i+1);
    Serial.print(" | C: "); Serial.println(cBranco);

    delay(150);
  }

  int mediaC = somaC / amostras;

  Serial.println("Calibração concluída!");
  Serial.print("mediaCBranco: "); Serial.println(mediaC);

  EEPROM.put(EEPROM_C_BRANCO, mediaC);

  valorCnoBranco = mediaC;

  Serial.println("Valor salvo na EEPROM!");
}

void calibrarCinzaMedia() {
  Serial.println("=== Calibração do Cinza (Média) ===");
  Serial.println("Coloque o sensor sobre o CINZA e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read(); // Limpa o buffer

  const int amostras = 20;
  long somaLux = 0, somaC = 0;
  int luxArray[amostras], cArray[amostras];

  for (int i = 0; i < amostras; i++) {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);

    int cCinza = (int)c1;
    int luxCinza = (int)lux1;

    somaLux += luxCinza;
    somaC += cCinza;
    luxArray[i] = luxCinza;
    cArray[i] = cCinza;

    Serial.print("Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxCinza);
    Serial.print(" | C: "); Serial.println(cCinza);

    delay(150);
  }

  int mediaLux = somaLux / amostras;
  int mediaC = somaC / amostras;

  // Calcula min/max com margem de 20%
  int minLux = mediaLux * 0.8;
  int maxLux = mediaLux * 1.2;
  int minC = mediaC * 0.8;
  int maxC = mediaC * 1.2;

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxCinza: "); Serial.println(mediaLux);
  Serial.print("mediaCCinza: "); Serial.println(mediaC);
  Serial.print("minLuxCinza (80%): "); Serial.println(minLux);
  Serial.print("maxLuxCinza (120%): "); Serial.println(maxLux);
  Serial.print("minCCinza (80%): "); Serial.println(minC);
  Serial.print("maxCCinza (120%): "); Serial.println(maxC);

  // Salva na EEPROM
  EEPROM.put(EEPROM_minLuxCinza, minLux);
  EEPROM.put(EEPROM_maxLuxCinza, maxLux);
  EEPROM.put(EEPROM_minCNoCinza, minC);
  EEPROM.put(EEPROM_maxCNoCinza, maxC);

  // Atualiza variáveis globais
  minLuxCinza = minLux;
  maxLuxCinza = maxLux;
  minCNoCinza = minC;
  maxCNoCinza = maxC;

  Serial.println("Valores salvos na EEPROM!");
}

void imprimirValoresEEPROM() {
  int valor;

  Serial.println("=== Valores armazenados na EEPROM ===");

  EEPROM.get(EEPROM_MIN_LUX_VERDE, valor);
  Serial.print("minLuxVerde: "); Serial.println(valor);
  EEPROM.get(EEPROM_MAX_LUX_VERDE, valor);
  Serial.print("maxLuxVerde: "); Serial.println(valor);
  EEPROM.get(EEPROM_MIN_C_VERDE, valor);
  Serial.print("minCVerde: "); Serial.println(valor);
  EEPROM.get(EEPROM_MAX_C_VERDE, valor);
  Serial.print("maxCVerde: "); Serial.println(valor);
  EEPROM.get(EEPROM_DIFERENCA_CORES, valor);
  Serial.print("diferencaDasCores (Verde): "); Serial.println(valor);

  EEPROM.get(EEPROM_C_BRANCO, valor);
  Serial.print("valorCnoBranco: "); Serial.println(valor);

  EEPROM.get(EEPROM_maxLuxPreto, valor);
  Serial.print("maxLuxPreto: "); Serial.println(valor);
  EEPROM.get(EEPROM_maxCPreto, valor);
  Serial.print("maxCPreto: "); Serial.println(valor);

  EEPROM.get(EEPROM_minLuxCinza, valor);
  Serial.print("minLuxCinza: "); Serial.println(valor);
  EEPROM.get(EEPROM_maxLuxCinza, valor);
  Serial.print("maxLuxCinza: "); Serial.println(valor);
  EEPROM.get(EEPROM_minCNoCinza, valor);
  Serial.print("minCNoCinza: "); Serial.println(valor);
  EEPROM.get(EEPROM_maxCNoCinza, valor);
  Serial.print("maxCNoCinza: "); Serial.println(valor);

  EEPROM.get(EEPROM_minLuxVermelho, valor);
  Serial.print("minLuxVermelho: "); Serial.println(valor);
  EEPROM.get(EEPROM_maxLuxVermelho, valor);
  Serial.print("maxLuxVermelho: "); Serial.println(valor);
  EEPROM.get(EEPROM_minCVermelho, valor);
  Serial.print("minCVermelho: "); Serial.println(valor);
  EEPROM.get(EEPROM_maxCVermelho, valor);
  Serial.print("maxCVermelho: "); Serial.println(valor);
  EEPROM.get(EEPROM_diferencaDasCoresVermelho, valor);
  Serial.print("diferencaDasCoresVermelho: "); Serial.println(valor);

  Serial.println("======================================");
}

void calculosVerde(){
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux2 = tcs2.calculateLux(r2, g2, b2);
    int cc1 = (int)c1;
    int cc2 = (int)c2;
    int luu1 = (int)lux1;
    int luu2 = (int)lux2;
    bool verdeDireita = (g1 > r1 && g1 > b1 && g1 - r1 > diferencaDasCores && cc1 >= minCVerde && cc1 <= maxCVerde && luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    bool verdeEsquerda = (g2 > r2 && g2 > b2 && g2 - r2 > diferencaDasCores && cc2 >= minCVerde && cc2 <= maxCVerde && luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    bool verdeAmbos = (abs(cc1 - cc2) < subtracaoSensoresCor) && verdeDireita && verdeEsquerda;
    Serial.print("Verde Direita: "); Serial.println(verdeDireita);
    Serial.print("Verde Esquerda: "); Serial.println(verdeEsquerda);
    Serial.print("Verde Ambos: "); Serial.println(verdeAmbos);
    Serial.print("cc1: "); Serial.print(cc1); Serial.print(" | cc2: "); Serial.println(cc2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("R1: "); Serial.print(r1); Serial.print(" | R2: "); Serial.println(r2);
    Serial.print("G1: "); Serial.print(g1); Serial.print(" | G2: "); Serial.println(g2);
    Serial.print("B1: "); Serial.print(b1); Serial.print(" | B2: "); Serial.println(b2);
    Serial.print("C1: "); Serial.print(c1); Serial.print(" | C2: "); Serial.println(c2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    Serial.print("g1 > r1 && g1 > b1: "); Serial.println(g1 > r1 && g1 > b1);
    Serial.print("g1 - r1 > diferencaDasCores: "); Serial.println(g1 - r1 > diferencaDasCores);
    Serial.print("g2 > r2 && g2 > b2: "); Serial.println(g2 > r2 && g2 > b2);
    Serial.print("g2 - r2 > diferencaDasCores: "); Serial.println(g2 - r2 > diferencaDasCores);
    Serial.print("cc1 >= minCVerde && cc1 <= maxCVerde: "); Serial.println(cc1 >= minCVerde && cc1 <= maxCVerde);
    Serial.print("cc2 >= minCVerde && cc2 <= maxCVerde: "); Serial.println(cc2 >= minCVerde && cc2 <= maxCVerde);
    Serial.print("luu1 >= minLuxVerde && luu1 <= maxLuxVerde: "); Serial.println(luu1 >= minLuxVerde && luu1 <= maxLuxVerde);
    Serial.print("luu2 >= minLuxVerde && luu2 <= maxLuxVerde: "); Serial.println(luu2 >= minLuxVerde && luu2 <= maxLuxVerde);
    Serial.print("Abs(cc1 - cc2) < subtracaoSensoresCor: "); Serial.println(abs(cc1 - cc2) < subtracaoSensoresCor);
}

void calculosVermelho() {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux2 = tcs2.calculateLux(r2, g2, b2);

    int cc1 = (int)c1;
    int cc2 = (int)c2;
    int luu1 = (int)lux1;
    int luu2 = (int)lux2;

    bool vermelhoDireita = (r1 > g1 && r1 > b1 && r1 - g1 > diferencaDasCoresVermelho &&
                            cc1 >= minCVermelho && cc1 <= maxCVermelho &&
                            luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho);

    bool vermelhoEsquerda = (r2 > g2 && r2 > b2 && r2 - g2 > diferencaDasCoresVermelho &&
                             cc2 >= minCVermelho && cc2 <= maxCVermelho &&
                             luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho);

    bool vermelhoAmbos = vermelhoDireita && vermelhoEsquerda;

    Serial.print("Vermelho Direita: "); Serial.println(vermelhoDireita);
    Serial.print("Vermelho Esquerda: "); Serial.println(vermelhoEsquerda);
    Serial.print("Vermelho Ambos: "); Serial.println(vermelhoAmbos);
    Serial.print("cc1: "); Serial.print(cc1); Serial.print(" | cc2: "); Serial.println(cc2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("R1: "); Serial.print(r1); Serial.print(" | R2: "); Serial.println(r2);
    Serial.print("G1: "); Serial.print(g1); Serial.print(" | G2: "); Serial.println(g2);
    Serial.print("B1: "); Serial.print(b1); Serial.print(" | B2: "); Serial.println(b2);
    Serial.print("C1: "); Serial.print(c1); Serial.print(" | C2: "); Serial.println(c2);

    Serial.print("cc1 >= minCVermelho && cc1 <= maxCVermelho: "); Serial.println(cc1 >= minCVermelho && cc1 <= maxCVermelho);
    Serial.print("cc2 >= minCVermelho && cc2 <= maxCVermelho: "); Serial.println(cc2 >= minCVermelho && cc2 <= maxCVermelho);
    Serial.print("luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho: "); Serial.println(luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho);
    Serial.print("luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho: "); Serial.println(luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho);
    Serial.print("r1 > g1 && r1 > b1: "); Serial.println(r1 > g1 && r1 > b1);
    Serial.print("r1 - g1 > diferencaDasCoresVermelho: "); Serial.println(r1 - g1 > diferencaDasCoresVermelho);
    Serial.print("r2 > g2 && r2 > b2: "); Serial.println(r2 > g2 && r2 > b2);
    Serial.print("r2 - g2 > diferencaDasCoresVermelho: "); Serial.println(r2 - g2 > diferencaDasCoresVermelho);
}

void calculosCinza() {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux2 = tcs2.calculateLux(r2, g2, b2);

    int cc1 = (int)c1;
    int cc2 = (int)c2;
    int luu1 = (int)lux1;
    int luu2 = (int)lux2;

    bool cinzaDireita = (cc1 >= minCNoCinza && cc1 <= maxCNoCinza && luu1 >= minLuxCinza && luu1 <= maxLuxCinza);
    bool cinzaEsquerda = (cc2 >= minCNoCinza && cc2 <= maxCNoCinza && luu2 >= minLuxCinza && luu2 <= maxLuxCinza);
    bool cinzaAmbos = cinzaDireita && cinzaEsquerda;

    Serial.print("Cinza Direita: "); Serial.println(cinzaDireita);
    Serial.print("Cinza Esquerda: "); Serial.println(cinzaEsquerda);
    Serial.print("Cinza Ambos: "); Serial.println(cinzaAmbos);
    Serial.print("cc1: "); Serial.print(cc1); Serial.print(" | cc2: "); Serial.println(cc2);
    Serial.print("Lux1: "); Serial.print(lux1); Serial.print(" | Lux2: "); Serial.println(lux2);
    Serial.print("R1: "); Serial.print(r1); Serial.print(" | R2: "); Serial.println(r2);
    Serial.print("G1: "); Serial.print(g1); Serial.print(" | G2: "); Serial.println(g2);
    Serial.print("B1: "); Serial.print(b1); Serial.print(" | B2: "); Serial.println(b2);
    Serial.print("C1: "); Serial.print(c1); Serial.print(" | C2: "); Serial.println(c2);

    Serial.print("cc1 >= minCNoCinza && cc1 <= maxCNoCinza: "); Serial.println(cc1 >= minCNoCinza && cc1 <= maxCNoCinza);
    Serial.print("cc2 >= minCNoCinza && cc2 <= maxCNoCinza: "); Serial.println(cc2 >= minCNoCinza && cc2 <= maxCNoCinza);
    Serial.print("luu1 >= minLuxCinza && luu1 <= maxLuxCinza: "); Serial.println(luu1 >= minLuxCinza && luu1 <= maxLuxCinza);
    Serial.print("luu2 >= minLuxCinza && luu2 <= maxLuxCinza: "); Serial.println(luu2 >= minLuxCinza && luu2 <= maxLuxCinza);
}

void alterarValorEEPROM(const char* nome, int endereco, int &variavel) {
  Serial.print("Digite o novo valor para ");
  Serial.print(nome);
  Serial.println(":");
  while (!Serial.available()) { delay(10); }
  int novoValor = Serial.parseInt();
  variavel = novoValor;
  EEPROM.put(endereco, novoValor);
  Serial.print(nome);
  Serial.print(" atualizado para: ");
  Serial.println(novoValor);
}

void processarComandoSerial() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.length() == 0) return;

    int opcao = comando.toInt();

    exibirMenuPrincipal();
    modoConfig = true;

    switch (menuAtual) {
      case 1: // Menu Principal
        switch (opcao) {
          case 1: retornoSensoresCor(); break;
          case 2: menuAtual = 2; exibirMenuCalculos(); break;
          case 3: calibrarVerdeMedia(); break;
          case 4: calibrarVermelhoMedia(); break;
          case 5: calibrarCinzaMedia(); break;
          case 6: calibrarBrancoMedia(); break;
          case 7: menuAtual = 3; exibirMenuAjusteVerde(); break;
          case 8: menuAtual = 4; exibirMenuAjusteVermelho(); break;
          case 9: imprimirValoresEEPROM(); break;
          case 0: Serial.println("Saindo do modo de configuração."); modoConfig = false; break;
          default: Serial.println("Opção inválida!"); exibirMenuPrincipal(); break;
        }
        break;

      case 2: // Menu Calculos
        switch (opcao) {
          case 1: calculosVerde(); break;
          case 2: calculosVermelho(); break;
          case 3: calculosCinza(); break;
          case 0: menuAtual = 1; exibirMenuPrincipal(); break;
          default: Serial.println("Opção inválida!"); exibirMenuCalculos(); break;
        }
        break;

      case 3: // Menu Ajuste Verde Manual
        switch (opcao) {
          case 1: retornoSensoresCor(); break;
          case 2: calculosVerde(); break;
          case 3: alterarValorEEPROM("maxLuxVerde", EEPROM_MAX_LUX_VERDE, maxLuxVerde); break;
          case 4: alterarValorEEPROM("minLuxVerde", EEPROM_MIN_LUX_VERDE, minLuxVerde); break;
          case 5: alterarValorEEPROM("maxCVerde", EEPROM_MAX_C_VERDE, maxCVerde); break;
          case 6: alterarValorEEPROM("minCVerde", EEPROM_MIN_C_VERDE, minCVerde); break;
          case 7: imprimirValoresEEPROM(); break;
          case 8: alterarValorEEPROM("diferencaDasCores", EEPROM_DIFERENCA_CORES, diferencaDasCores); break;
          case 0: menuAtual = 1; exibirMenuPrincipal(); break;
          default: Serial.println("Opção inválida!"); exibirMenuAjusteVerde(); break;
        }
        break;

      case 4: // Menu Ajuste Vermelho Manual
        switch (opcao) {
          case 1: retornoSensoresCor(); break;
          case 2: calculosVermelho(); break;
          case 3: alterarValorEEPROM("maxLuxVermelho", EEPROM_maxLuxVermelho, maxLuxVermelho); break;
          case 4: alterarValorEEPROM("minLuxVermelho", EEPROM_minLuxVermelho, minLuxVermelho); break;
          case 5: alterarValorEEPROM("maxCVermelho", EEPROM_maxCVermelho, maxCVermelho); break;
          case 6: alterarValorEEPROM("minCVermelho", EEPROM_minCVermelho, minCVermelho); break;
          case 7: imprimirValoresEEPROM(); break;
          case 8: alterarValorEEPROM("diferencaDasCoresVermelho", EEPROM_diferencaDasCoresVermelho, diferencaDasCoresVermelho); break;
          case 0: menuAtual = 1; exibirMenuPrincipal(); break;
          default: Serial.println("Opção inválida!"); exibirMenuAjusteVermelho(); break;
        }
        break;

      case 5: // Menu Ajuste Cinza Manual
        switch (opcao) {
          case 1: retornoSensoresCor(); break;
          case 2: calculosCinza(); break;
          case 3: alterarValorEEPROM("maxLuxCinza", EEPROM_maxLuxCinza, maxLuxCinza); break;
          case 4: alterarValorEEPROM("minLuxCinza", EEPROM_minLuxCinza, minLuxCinza); break;
          case 5: alterarValorEEPROM("maxCCinza", EEPROM_maxCNoCinza, maxCNoCinza); break;
          case 6: alterarValorEEPROM("minCCinza", EEPROM_minCNoCinza, minCNoCinza); break;
          case 7: imprimirValoresEEPROM(); break;
          case 0: menuAtual = 1; exibirMenuPrincipal(); break;
          default: Serial.println("Opção inválida!"); exibirMenuAjusteCinza(); break;
        }
        break;
    }
  }
}

//******************************************************************************
//*                                                                            *
//*                                Void Setup                                  *
//*                                                                            *
//******************************************************************************

void setup() {
  Serial.begin(115200);

  // Wire.begin();
  // Wire.setClock(400000);

  byte status = giro.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ }
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  giro.calcOffsets();
  Serial.println("Done!\n");

  for (int i = 0; i < 5; i++) {
    pinMode(sensores[i], INPUT);
  }

  pinMode(buzzer, OUTPUT);

  motorE.attach(motorEpin);
  motorD.attach(motorDpin);
  
  motorE.write(90);
  motorD.write(90);

  anguloDoReto = retornoAnguloY();
  anguloRampaSubida = anguloDoReto - 10;
  anguloRampaDescida = anguloDoReto + 10;

  retornoSensoresCor();

  EEPROM.get(EEPROM_MIN_LUX_VERDE, minLuxVerde);
  EEPROM.get(EEPROM_MAX_LUX_VERDE, maxLuxVerde);
  EEPROM.get(EEPROM_MIN_C_VERDE, minCVerde);
  EEPROM.get(EEPROM_MAX_C_VERDE, maxCVerde);
  EEPROM.get(EEPROM_DIFERENCA_CORES, diferencaDasCores);
  EEPROM.get(EEPROM_C_BRANCO, valorCnoBranco);
  EEPROM.get(EEPROM_maxLuxPreto, maxLuxPreto);
  EEPROM.get(EEPROM_maxCPreto, maxCPreto);
  EEPROM.get(EEPROM_minLuxCinza, minLuxCinza);
  EEPROM.get(EEPROM_maxLuxCinza, maxLuxCinza);
  EEPROM.get(EEPROM_minCNoCinza, minCNoCinza);
  EEPROM.get(EEPROM_maxCNoCinza, maxCNoCinza);
  EEPROM.get(EEPROM_minLuxVermelho, minLuxVermelho);
  EEPROM.get(EEPROM_maxLuxVermelho, maxLuxVermelho);
  EEPROM.get(EEPROM_minCVermelho, minCVermelho);
  EEPROM.get(EEPROM_maxCVermelho, maxCVermelho);
  EEPROM.get(EEPROM_diferencaDasCoresVermelho, diferencaDasCoresVermelho);

  ligarGarra();
  fecharGarra();
  subirGarra();
  desligarGarra();

  tocar_buzzer(750, 3, 125);
}

//******************************************************************************
//*                                                                            *
//*                                Void Loop                                   *
//*                                                                            *
//******************************************************'************************

void loop() {
  //verificaVermelho();
  //retornoSensoresCor();
  // lerInfravermelho();
  // delay(1000);
  //andarReto();

  // testarAmplitudeSensores();

  // Serial.print("Garra: "); Serial.println(motorG.read());

  processarComandoSerial(); // Sempre verifica comandos seriais
  if (!modoConfig) {
    andarReto(); // Executa lógica normal do robô apenas se não estiver no modo de configuração
  }
}
