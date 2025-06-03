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
int verificacaoCurva = 10; //Usado para verificar se é curva 90 ou correção
boolean pulinho = false; //Usado para verificar se é curva 90 ou correção

int veloBaseEsq = 120;
int veloBaseDir = 60;
int pequenaCurvaLadoC = 10;
int pequenaCurvaLadoR = 5;
int veloCurva90 = 40;

int grausCurva90 = 90;
int graqusCurva180 = 175;

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
int minLuxVerde = 60;
int maxLuxVerde = 140;
int minCVerde = 150;
int maxCVerde = 250;

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
int distanciaDesvio = 10;
int delayCurva1 = 4;
int delayCurva2 = 8;
int delayMeio = 1;
//************************************************************************
//*                                                                      *
//*                              Funções                                 *
//*                                                                      *
//************************************************************************

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

void varreduraResgate() {
  
}

void resgate(){
  Serial.println("Resgate iniciado!");
  motorE.write(90);
  motorD.write(90);
  tocar_buzzer(500, 2, 100);
  
  int d1 = mediaInfravermelhoFrente(10);
  int meio = d1/2;

  ligarGarra();
  abrirGarra();
  descerGarra();
  delay(1500);

  Serial.print("Angulo Reto: "); Serial.print(anguloReto); Serial.print(" | Angulo Atual: "); Serial.println(retornoAnguloZ());

  while(d1 >= meio){
    d1 = mediaInfravermelhoFrente();
    Serial.print("Distância atual: "); Serial.print(d1); Serial.print(" | Meio: "); Serial.println(meio); 
    correcaoObjeto();
    giro.update();
  }

  motorD.write(90);
  motorE.write(90);

  int d2 = SI_Esquerda.distance();
  int d3 = SI_Direita.distance();
  
  int altura = 0; 
  
  bool virarEsquerda = d2 > d3;
  bool virarDireita = d3 > d2;

  if(virarEsquerda){
    altura = d2;
    Serial.println("Esquerda!");
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      Serial.print("Fazendo curva para a esquerda | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto + 90);
    }
    anguloReto = anguloReto + grausCurva90;
    motorD.write(90);
    motorE.write(90);

  } else if(virarDireita){
    altura = d3;
    Serial.println("Direita!");
    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      Serial.print("Fazendo curva para a direita | Angulo Atual: "); Serial.print(retornoAnguloZ()); Serial.print(" Objetivo: "); Serial.println(anguloReto - 90);
    }
    anguloReto = anguloReto - grausCurva90;
    motorD.write(90);
    motorE.write(90);
  }

  fecharGarra();
  delay(1000);
  garraMeio();
  delay(1000);
  desligarMotoresGarra();
  
  //Alinhando pra trás

  motorD.write(veloBaseEsq);
  motorE.write(veloBaseDir);

  delay(2000);

  anguloReto = retornoAnguloZ();

  varreduraResgate();
  
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
      while (anguloReto - graqusCurva180 <= retornoAnguloZ()) {
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
  if (anguloReto - erro > anguloReto) {
    motorE.write(100);
    motorD.write(90);
    Serial.println("Objeto Correção1");
  }
  else if (anguloReto + erro < anguloReto) {
    motorE.write(90);
    motorD.write(80);
    Serial.println("Objeto Correção2");
  }
  else if (abs(anguloReto - anguloReto) <= erro) {
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

    while (((anguloReto + grausCurva90) >= retornoAnguloZ())) {
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
    while (((anguloReto - grausCurva90) <= retornoAnguloZ())) {
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
    while (((anguloReto - grausCurva90) <= retornoAnguloZ())) {
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

    while (((anguloReto + grausCurva90) >= retornoAnguloZ())) {
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
      while (((anguloReto + grausCurva90) >= retornoAnguloZ())) {
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
      while (((anguloReto - grausCurva90) <= retornoAnguloZ())) {
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
      break;

    case 0b11101: // Saiu do principal, direita
      Serial.println("Saiu do principal, direita");
      motorE.write(100);
      motorD.write(veloBaseDir);
      sl = lerSensoresLinha();
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
      while (((anguloReto - grausCurva90) <= retornoAnguloZ())) {
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
      while (((anguloReto + grausCurva90) >= retornoAnguloZ())) {
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
  motorG.write(180); // Posição de descida
  delay(2000); // Tempo para descer
}

void garraMeio(){
  Serial.println("Garra Meio");
  motorG.write(90); // Posição de meio
  delay(2000); // Tempo para meio
}

void subirGarra() {
  Serial.println("Subindo Garra");
  motorG.write(0); // Posição de subida
  delay(1000); // Tempo para subir
}

void abrirGarra() {
  Serial.println("Abrindo Garra");
  motorEsqG.write(0); // Posição de abertura
  motorDirG.write(180); // Posição de abertura
  delay(1000); // Tempo para abrir
}

void fecharGarra() {
  Serial.println("Fechando Garra");
  motorEsqG.write(85); // Posição de fechamento
  motorDirG.write(85); // Posição de fechamento
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
  delay(500);
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

void processarComandoSerial() {
  static boolean esperandoValor = false; // Controla se está esperando um valor após selecionar uma opção
  static int opcaoSelecionada = 0; // Armazena a opção escolhida (2 a 6) para alteração

  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    if (comando.length() > 0) {
      if (!modoConfig && comando == "1") {
        modoConfig = true;
        Serial.println("Modo de configuracao ATIVADO.");
        exibirMenu();
        esperandoValor = false;
      } else if (modoConfig) {
        if (esperandoValor) {
          int valor = comando.toInt();
          switch (opcaoSelecionada) {
            case 2:
              minLuxVerde = valor;
              EEPROM.put(EEPROM_MIN_LUX_VERDE, minLuxVerde);
              Serial.print("minLuxVerde atualizado para: "); Serial.println(minLuxVerde);
              break;
            case 3:
              maxLuxVerde = valor;
              EEPROM.put(EEPROM_MAX_LUX_VERDE, maxLuxVerde);
              Serial.print("maxLuxVerde atualizado para: "); Serial.println(maxLuxVerde);
              break;
            case 4:
              minCVerde = valor;
              EEPROM.put(EEPROM_MIN_C_VERDE, minCVerde);
              Serial.print("minCVerde atualizado para: "); Serial.println(minCVerde);
              break;
            case 5:
              maxCVerde = valor;
              EEPROM.put(EEPROM_MAX_C_VERDE, maxCVerde);
              Serial.print("maxCVerde atualizado para: "); Serial.println(maxCVerde);
              break;
            case 6:
              diferencaDasCores = valor;
              EEPROM.put(EEPROM_DIFERENCA_CORES, diferencaDasCores);
              Serial.print("diferencaDasCores atualizado para: "); Serial.println(diferencaDasCores);
              break;
            default:
              Serial.println("Erro: opcao invalida!");
              break;
          }
          esperandoValor = false;
          exibirMenu(); // Volta ao menu após alterar o valor
        } else {
          int opcao = comando.toInt();
          switch (opcao) {
            case 0:
              modoConfig = false;
              Serial.println("Modo de configuracao DESATIVADO. Robô em modo normal.");
              break;
            case 1:
              retornoSensoresCor();
              exibirMenu();
              break;
            case 2:
              Serial.println("Digite o novo valor para minLuxVerde:");
              esperandoValor = true;
              opcaoSelecionada = 2;
              break;
            case 3:
              Serial.println("Digite o novo valor para maxLuxVerde:");
              esperandoValor = true;
              opcaoSelecionada = 3;
              break;
            case 4:
              Serial.println("Digite o novo valor para minCVerde:");
              esperandoValor = true;
              opcaoSelecionada = 4;
              break;
            case 5:
              Serial.println("Digite o novo valor para maxCVerde:");
              esperandoValor = true;
              opcaoSelecionada = 5;
              break;
            case 6:
              Serial.println("Digite o novo valor para diferencaDasCores:");
              esperandoValor = true;
              opcaoSelecionada = 6;
              break;
            case 7:
              printEEPROMValues();
              exibirMenu();
              break;
            case 8:
              Serial.println("Fazendo calculos do verde...");
              lerVerde1();
            default:
              Serial.println("Opcao invalida! Digite um numero de 0 a 7.");
              exibirMenu();
              break;
          }
        }
      } else {
        Serial.println("Digite '1' para entrar no modo de configuracao.");
      }
    }
  }
}

void printEEPROMValues() {
  Serial.println("Valores armazenados na EEPROM:");
  Serial.print("minLuxVerde: "); Serial.println(minLuxVerde);
  Serial.print("maxLuxVerde: "); Serial.println(maxLuxVerde);
  Serial.print("minCVerde: "); Serial.println(minCVerde);
  Serial.print("maxCVerde: "); Serial.println(maxCVerde);
  Serial.print("diferencaDasCores: "); Serial.println(diferencaDasCores);
}

void exibirMenu() {
  Serial.println("=== Menu de Configuracao ===");
  Serial.println("1 - Ler sensores de cor");
  Serial.println("2 - Alterar minLuxVerde");
  Serial.println("3 - Alterar maxLuxVerde");
  Serial.println("4 - Alterar minCVerde");
  Serial.println("5 - Alterar maxCVerde");
  Serial.println("6 - Alterar diferencaDasCores");
  Serial.println("7 - Exibir valores da EEPROM");
  Serial.println("8 - Calculos do verde");
  Serial.println("0 - Sair do modo de configuracao");
  Serial.println("============================");
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

  // motorG.write(0);
  // motorEsqG.write(90);
  // motorDirG.write(90);

  anguloDoReto = retornoAnguloY();
  anguloRampaSubida = anguloDoReto - 10;
  anguloRampaDescida = anguloDoReto + 10;

  retornoSensoresCor();

  EEPROM.get(EEPROM_MIN_LUX_VERDE, minLuxVerde);
  EEPROM.get(EEPROM_MAX_LUX_VERDE, maxLuxVerde);
  EEPROM.get(EEPROM_MIN_C_VERDE, minCVerde);
  EEPROM.get(EEPROM_MAX_C_VERDE, maxCVerde);
  EEPROM.get(EEPROM_DIFERENCA_CORES, diferencaDasCores);
  
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
  //lerInfravermelho();
  //delay(1000);
  //andarReto();

  processarComandoSerial(); // Sempre verifica comandos seriais
  if (!modoConfig) {
    andarReto(); // Executa lógica normal do robô apenas se não estiver no modo de configuração
  }
}