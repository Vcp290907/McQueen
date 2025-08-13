#include <Arduino.h>
#line 1 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
#include <EEPROM.h>
#include "Servo.h"
#include "Adafruit_TCS34725softi2c.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <SharpIR.h>
#include <Ultrasonic.h>
#include <math.h>

// Endereços na EEPROM para os parâmetros
#define EEPROM_MIN_LUX_VERDE_DIR 0
#define EEPROM_MAX_LUX_VERDE_DIR 2
#define EEPROM_MIN_C_VERDE_DIR 4
#define EEPROM_MAX_C_VERDE_DIR 6
#define EEPROM_DIFERENCA_CORES_DIR 8

#define EEPROM_MIN_LUX_VERDE_ESQ 34
#define EEPROM_MAX_LUX_VERDE_ESQ 36
#define EEPROM_MIN_C_VERDE_ESQ 38
#define EEPROM_MAX_C_VERDE_ESQ 40
#define EEPROM_DIFERENCA_CORES_ESQ 42

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

bool modoConfig = false;

#define SDApin1 2 //Sensor Esquerda
#define SCLpin1 3 //Sensor Esquerda
#define SDApin2 4 //Sensor Direita
#define SCLpin2 5 //Sensor Direita 

#define motorEpin 7 //Servo Esquerdo
#define motorDpin 6 //Servo Direito

#define motorGpin 8 //Servo Garra
#define motorEsqGpin 12 //Servo Garra Esquerdo
#define motorDirGpin 10 //Servo Garra Direito

#define no 11
#define trig 15
#define echo 13

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

#define pinoR1 27
#define pinoG1 29
#define pinoB1 31
#define pinoR2 28
#define pinoG2 30
#define pinoB2 18

#define Hall_Esquerda A8
#define Hall_Direita A6

Ultrasonic ultrasonic(trig, echo);

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
const int sensores[] PROGMEM = {SE,SME,SM,SMD,SD};
static int valores[5];

// Variáveis para os sensores de cor - otimizadas
uint16_t r1, g1, b1, c1, lux1, r2, g2, b2, c2, lux2;

int erro = 1;
int anguloRampaSubida, anguloRampaDescida, anguloDoReto, anguloReto;
int* sl;
bool trava = false;

// Parâmetros gerais
const int veloBaseEsq = 40; //40
const int veloBaseDir = 140; //140
const int pequenaCurvaLadoC = 15; //15
const int pequenaCurvaLadoR = 5; //5
const int veloCurva90 = 40; //40

const int grausCurva90 = 90;
const int graqusCurva180 = 180;
const int grausCurva45 = 45;

int anguloAtual = 0;

const int verificacaoCurvaVerde = 350; //Pulinho para ver se é curva verde
int erroGiro = 0;
const int erroRampa = 3; // 3
const int erroRampaDescida = 12; // 12
const int tempoDepoisDoVerde90 = 500;
const int delayCurvasverde = 450;
const int tempoAntesCurva90 = 400;
const int tempoDepoisCurva90 = 150; //1000
const int tempoDepoisDoVerde180 = 150; //1000
const int tempoDepoisDoVerdeFalso = 500; //1000
const int paredeResgate = 2000; //2000
const int paredeResgateSaida = 1000; //1000

//Branco
int valorCnoBranco = 1000; 

// Verde - variáveis de calibração (mantidas como int)
int minLuxVerdeDir, minLuxVerdeEsq;
int maxLuxVerdeDir, maxLuxVerdeEsq;
int minCVerdeDir, minCVerdeEsq;
int maxCVerdeDir, maxCVerdeEsq;
int diferencaDasCoresDir, diferencaDasCoresEsq;

// Preto - otimizado
int maxLuxPreto = 150;
int maxCPreto = 400;

// Cinza - otimizado  
int minLuxCinza = 110;
int maxLuxCinza = 200;
int minCNoCinza = 550;
int maxCNoCinza = 650;

// Vermelho - otimizado
int minLuxVermelho = 100;
int maxLuxVermelho = 150;
int minCVermelho = 400;
int maxCVermelho = 650;
int diferencaDasCoresVermelho = 40;

// Outros
const int subtracaoSensoresCor = 100;

//PID - otimizado com tipos menores
const float Kp = 2.0;
const float Ki = 0.2;
const float Kd = 1.5;
int erroP = 0;
int erroAnterior = 0;
float erroI = 0;
float erroD = 0;
int ajuste;

// PID específico para função correcao (independente do andarReto para não interferir)
int erroP_cor = 0;
int erroAnterior_cor = 0;
float erroI_cor = 0;
float erroD_cor = 0;
const int maxAjusteCorrecao = 45; // limite de ajuste (era pequenaCurvaLadoC*3 na lógica antiga)

//Desvio OBJETO - otimizado
const int distanciaDesvio = 0; //6
const int delayCurva1 = 3;
const int delayCurva2 = 7;
const int delayMeio = 1;
//************************************************************************
//*                                                                      *
//*                              Funções                                 *
//*                                                                      *
//************************************************************************

#line 200 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void tocar_buzzer(int freque, int unidades, int espera);
#line 210 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int * lerSensoresLinha();
#line 217 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void setCorEsquerda(int vermelho,int verde, int azul);
#line 223 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void setCorDireita(int vermelho,int verde, int azul);
#line 229 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoAnguloZ();
#line 252 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void verificaCores();
#line 287 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void piscaLeds(int intervalo);
#line 299 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void sairFrente();
#line 308 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void sairMeioDireita();
#line 331 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void sairMeioEsquerda();
#line 354 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void andarRetoPorTempo(int tempo);
#line 364 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void andarPraTrasPorTempo(int tempo);
#line 374 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void curvaComHall(int direcao, int graus);
#line 477 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void curva90Direita();
#line 481 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void curva90Esquerda();
#line 485 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void curva180Direita();
#line 672 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void resgate();
#line 878 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoAnguloY();
#line 1027 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void correcao();
#line 1064 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void correcaoObjeto();
#line 1080 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void correcaoRe();
#line 1096 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desvioObjeto();
#line 1194 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void andarReto();
#line 1505 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void ligarGarra();
#line 1512 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarGarra();
#line 1519 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarMotorPrincipal();
#line 1524 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void desligarMotoresGarra();
#line 1530 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void descerGarra();
#line 1536 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void garraMeio();
#line 1542 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void garra90();
#line 1547 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void subirGarra();
#line 1553 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void abrirGarra();
#line 1560 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void fecharGarra();
#line 1567 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void fechar_bolinha();
#line 1600 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
int retornoSensoresCor();
#line 1633 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void retornoGiroscopio();
#line 1654 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void retornoSensoresLinha();
#line 1662 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void lerInfravermelho();
#line 1675 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuPrincipal();
#line 1690 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuCalculos();
#line 1699 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteVerde();
#line 1713 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteVermelho();
#line 1727 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void exibirMenuAjusteCinza();
#line 1740 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarVerdeMedia();
#line 1876 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarVermelhoMedia();
#line 2032 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarBrancoMedia();
#line 2066 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calibrarCinzaMedia();
#line 2143 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void imprimirValoresEEPROM();
#line 2201 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosVerde();
#line 2244 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosVermelho();
#line 2285 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void calculosCinza();
#line 2311 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void alterarValorEEPROM(const char* nome, int endereco, int &variavel);
#line 2324 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void processarComandoSerial();
#line 2451 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void setup();
#line 2539 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
void loop();
#line 200 "C:\\Users\\VCP2909\\Desktop\\Carrinho_OBR\\Programação OBR_Arduino\\2025\\V4\\McQueen\\V4\\V4.ino"
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
    valores[i] = digitalRead(pgm_read_word(&sensores[i]));
  }
  return valores;
}

void setCorEsquerda(int vermelho,int verde, int azul){
  digitalWrite(pinoR1, vermelho);
  digitalWrite(pinoG1, verde);
  digitalWrite(pinoB1, azul);
}

void setCorDireita(int vermelho,int verde, int azul){
  digitalWrite(pinoR2, vermelho);
  digitalWrite(pinoG2, verde);
  digitalWrite(pinoB2, azul);
}

int retornoAnguloZ(){
  giro.update();
  return (int)round(giro.getAngleZ());
}

int mediaInfravermelho(int sensor, int numLeituras = 5) { // 1 pra esquerda, 2 pra frente, 3 pra direita
  long soma = 0;
  SharpIR* sensorPtr;
  if (sensor == 1) sensorPtr = &SI_Esquerda;
  else if (sensor == 2) sensorPtr = &SI_Frente;
  else if (sensor == 3) sensorPtr = &SI_Direita;
  else return -1; 

  for (int i = 0; i < numLeituras; i++) {
    int medida = sensorPtr->distance();
    soma += medida;
    Serial.print(F("Distancia medida: "));Serial.println(medida);
    delay(5);
  }
  Serial.print(F("Media Sensor ")); Serial.print(sensor); Serial.print(": "); Serial.println(soma / numLeituras);
  return soma / numLeituras;
}

void verificaCores() {
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;

  bool vermelho = ((r1 > g1 && r1 > b1 && r1 - g1 > diferencaDasCoresVermelho && cc1 >= minCVermelho 
                    && cc1 <= maxCVermelho && luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho) || 
                    (r2 > g2 && r2 > b2 && r2 - g2 > diferencaDasCoresVermelho && cc2 >= minCVermelho 
                      && cc2 <= maxCVermelho && luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho));

  bool cinza = ((cc1 >= minCNoCinza && cc1 <= maxCNoCinza && luu1 >= minLuxCinza && luu1 <= maxLuxCinza) ||
              (cc2 >= minCNoCinza && cc2 <= maxCNoCinza && luu2 >= minLuxCinza && luu2 <= maxLuxCinza));

  if (vermelho) {
    Serial.println(F("VERMELHO DETECTADO!"));
    while (true) {
      motorE.write(90);
      motorD.write(90);
    }
  }else if (cinza) {
    Serial.println("Cinza detectado!");
    resgate();
    return;
  }else{
    Serial.println(F("Não é vermelho nem cinza"));
    return;
  }
}

void piscaLeds(int intervalo){
  setCorDireita(1,1,0);
  setCorEsquerda(1,1,0);
  delay(intervalo);
  setCorDireita(1,0,1);
  setCorEsquerda(1,0,1);
  delay(intervalo);
  setCorDireita(0,1,1);
  setCorEsquerda(0,1,1);
  delay(intervalo);
}

void sairFrente(){
  sl = lerSensoresLinha();
  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1){
    sl = lerSensoresLinha();
    correcaoObjeto();
  }
  return;
}

void sairMeioDireita(){
  motorE.write(veloBaseDir);
  motorD.write(veloBaseDir);
  while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
    giro.update();
  }
  anguloReto = anguloReto - grausCurva90;
  motorE.write(90);
  motorD.write(90);

  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1){
    sl = lerSensoresLinha();
    correcaoObjeto();
  }

  while(true) {
    motorE.write(90);
    motorD.write(90);
    piscaLeds(250);
  }
  return;
}

void sairMeioEsquerda(){
  motorE.write(veloBaseEsq);
  motorD.write(veloBaseEsq);
  while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
    giro.update();
  }
  anguloReto = anguloReto + grausCurva90;
  motorE.write(90);
  motorD.write(90);

  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1){
    sl = lerSensoresLinha();
    correcaoObjeto();
  }

  while(true) {
    motorE.write(90);
    motorD.write(90);
    piscaLeds(250);
  }
  return;
}

void andarRetoPorTempo(int tempo){
  Serial.print(F("Andando reto por ")); Serial.print(tempo); Serial.println(" ms");
  unsigned long startTime = millis();
  while (millis() - startTime < tempo) {
    correcaoObjeto();
  }
  motorE.write(90);
  motorD.write(90);
}

void andarPraTrasPorTempo(int tempo){
  Serial.print(F("Andando reto pra tras por ")); Serial.print(tempo); Serial.println(" ms");
  unsigned long startTime = millis();
  while (millis() - startTime < tempo) {
    correcaoRe();
  }
  motorE.write(90);
  motorD.write(90);
}

void curvaComHall(int direcao, int graus) {
  Serial.print("Executando curva de "); Serial.print(graus);
  Serial.print(" graus para "); Serial.println(direcao == 1 ? "direita" : "esquerda");
  
  motorE.write(90);
  motorD.write(90);
  delay(100);
  
  int lambdasNecessarios;
  
  if (graus <= 90) {
    lambdasNecessarios = (graus * 18) / 90;
  } else {
    lambdasNecessarios = (graus * 35) / 180;
  }
  
  if (lambdasNecessarios < 1) lambdasNecessarios = 1; // Mínimo 1 lambda
  
  Serial.print("Lambdas necessarios para "); Serial.print(graus); 
  Serial.print(" graus: "); Serial.println(lambdasNecessarios);
  
  int leituraInicialCurva = analogRead(Hall_Direita);
  Serial.print("Leitura inicial do sensor Hall: "); Serial.println(leituraInicialCurva);

  delay(50);
  leituraInicialCurva = analogRead(Hall_Direita);
  bool leituraInicialBaixa = false;

  // Ajuste da leitura inicial se necessário (MESMA LÓGICA do andarRetoComHall)
  if (leituraInicialCurva < 350) {
    Serial.println("Valor ainda baixo após rodada extra, ajustando...");
    leituraInicialCurva = 360;
    leituraInicialBaixa = true;
  } else if(leituraInicialCurva > 660){
    Serial.println("Valor muito alto, ajustando...");
    leituraInicialCurva = 650;
    leituraInicialBaixa = true;
  }
  
  Serial.print("Leitura inicial ajustada foi necessária: "); Serial.println(leituraInicialBaixa ? "SIM" : "NÃO");
  
  int lambdasCurvaContados = 0;
  bool lambdaCurvaDetectado = false;
  unsigned long ultimaDeteccaoCurva = 0;
  const unsigned long debounceTimeCurva = 75;
  
  // Flag para ignorar primeira detecção se leitura inicial foi ajustada
  bool ignorarPrimeiraDeteccao = leituraInicialBaixa;
  
  int thresholdCurva = max(50, (int)(leituraInicialCurva * 0.20));
  
  if (direcao == 1) { // Direita
    motorE.write(veloBaseEsq + 30);
    motorD.write(veloBaseEsq + 30);
  } else { // Esquerda
    motorE.write(veloBaseDir - 30);
    motorD.write(veloBaseDir - 30);
  }
  
  int leituraAtualCurva;
  giro.update();
  bool mudancaDetectadaCurva;
  
  while(lambdasCurvaContados < lambdasNecessarios) {
    leituraAtualCurva = analogRead(Hall_Direita);
    Serial.println(leituraAtualCurva);
    mudancaDetectadaCurva = (leituraAtualCurva < (leituraInicialCurva - thresholdCurva) || 
                                  leituraAtualCurva > (leituraInicialCurva + thresholdCurva));
    
    if (mudancaDetectadaCurva && !lambdaCurvaDetectado && 
        (millis() - ultimaDeteccaoCurva > debounceTimeCurva)) {
      
      // NOVA LÓGICA: Se deve ignorar a primeira detecção (igual ao andarRetoComHall)
      if (ignorarPrimeiraDeteccao) {
        Serial.println("Ignorando primeira detecção de lambda curva (ajuste de leitura inicial)");
        ignorarPrimeiraDeteccao = false; // Só ignora uma vez
        lambdaCurvaDetectado = true;
        ultimaDeteccaoCurva = millis();
      } else {
        // Contabiliza normalmente
        lambdaCurvaDetectado = true;
        lambdasCurvaContados++;
        ultimaDeteccaoCurva = millis();
        Serial.print("Lambda curva detectado: "); Serial.println(lambdasCurvaContados);
      }
    }
    
    giro.update();
    if (!mudancaDetectadaCurva && lambdaCurvaDetectado) {
      lambdaCurvaDetectado = false;
    }
    
    delay(15);
  }
  
  motorE.write(90);
  motorD.write(90);
  delay(100);
  
  Serial.print("Curva de "); Serial.print(graus); 
  Serial.println(" graus concluída com precisão do sensor Hall!");
}

void curva90Direita() {
  curvaComHall(1, 90);
}

void curva90Esquerda() {
  curvaComHall(-1, 90);
}

void curva180Direita() {
  curvaComHall(1, 180);
}

void andarRetoComHall(int distanciaCm, int anguloReferencia ,bool praTras = false) {
  if (praTras) {
    Serial.print("Andando para TRÁS por "); Serial.print(distanciaCm); Serial.println(" cm com correção PID");
  } else {
    Serial.print("Andando para FRENTE por "); Serial.print(distanciaCm); Serial.println(" cm com correção PID");
  }
  
  erroI = 0;
  erroAnterior = 0;
  
  //int anguloReferencia = retornoAnguloZ();

  Serial.print("Ângulo de referência: "); Serial.println(anguloReferencia);
  
  // Cálculo dos lambdas necessários
  int lambdasNecessarios = (int)(distanciaCm * 1.05); // Ajustar conforme necessário
  if (lambdasNecessarios < 1) lambdasNecessarios = 1;
  
  Serial.print("Lambdas necessários para "); Serial.print(distanciaCm); 
  Serial.print(" cm: "); Serial.println(lambdasNecessarios);
  
  // Inicialização do sensor Hall
  int leituraInicial = analogRead(Hall_Direita);
  Serial.print("Leitura inicial do sensor Hall: "); Serial.println(leituraInicial);
  
  delay(50);
  leituraInicial = analogRead(Hall_Direita);
  bool leituraInicialBaixa = false;
  
  // Ajuste da leitura inicial se necessário
  if (leituraInicial < 350) {
    Serial.println("Valor ainda baixo, ajustando...");
    leituraInicial = 360;
    leituraInicialBaixa = true;
  } else if (leituraInicial > 660) {
    Serial.println("Valor muito alto, ajustando...");
    leituraInicial = 650;
    leituraInicialBaixa = true;
  }
  
  Serial.print("Leitura inicial ajustada foi necessária: "); Serial.println(leituraInicialBaixa ? "SIM" : "NÃO");
  
  // Variáveis para contagem de lambdas
  int lambdasContados = 0;
  bool lambdaDetectado = false;
  unsigned long ultimaDeteccao = 0;
  const unsigned long debounceTime = 75;
  
  // Flag para ignorar primeira detecção se leitura inicial foi ajustada
  bool ignorarPrimeiraDeteccao = leituraInicialBaixa;
  
  int threshold = max(50, (int)(leituraInicial * 0.20));
  
  // Inicia movimento
  int velE, velD;
  if (praTras) {
    // Velocidades invertidas para andar para trás
    velE = veloBaseDir; 
    velD = veloBaseEsq; 
  } else {
    // Velocidades normais para andar para frente
    velE = veloBaseEsq;
    velD = veloBaseDir;
  }
  
  motorE.write(velE);
  motorD.write(velD);
  
  int leituraAtual;
  bool mudancaDetectada;
  unsigned long ultimaCorrecao = 0;
  const unsigned long intervaloCorrecao = 25; // Correção PID a cada 25ms
  
  while(lambdasContados < lambdasNecessarios) {
    giro.update();
    leituraAtual = analogRead(Hall_Direita);
    
    // Detecção de lambda (mudança magnética)
    mudancaDetectada = (leituraAtual < (leituraInicial - threshold) || 
                       leituraAtual > (leituraInicial + threshold));
    
    if (mudancaDetectada && !lambdaDetectado && 
        (millis() - ultimaDeteccao > debounceTime)) {
      
      // Se deve ignorar a primeira detecção
      if (ignorarPrimeiraDeteccao) {
        Serial.println("Ignorando primeira detecção de lambda (ajuste de leitura inicial)");
        ignorarPrimeiraDeteccao = false; // Só ignora uma vez
        lambdaDetectado = true;
        ultimaDeteccao = millis();
      } else {
        // Contabiliza normalmente
        lambdaDetectado = true;
        lambdasContados++;
        ultimaDeteccao = millis();
        Serial.print("Lambda detectado: "); Serial.println(lambdasContados);
      }
    }
    
    if (!mudancaDetectada && lambdaDetectado) {
      lambdaDetectado = false;
    }
    
    // Correção PID a intervalos regulares
    if (millis() - ultimaCorrecao > intervaloCorrecao) {
      giro.update();
      anguloAtual = retornoAnguloZ();
      
      // Cálculo PID
      erroP = anguloAtual - anguloReferencia;
      erroI += erroP;
      erroI = constrain(erroI, -100, 100);
      erroD = erroP - erroAnterior;
      erroAnterior = erroP;
      
      // Saída PID
      ajuste = Kp * erroP + Ki * erroI + Kd * erroD;
      ajuste = abs(ajuste);
      ajuste = constrain(ajuste, 0, 40);
      
      if (praTras) {
        // Lógica invertida para movimento para trás
        if (erroP > 1) { // Desviou para a direita, corrigir para esquerda
          velE = veloBaseDir; 
          velD = veloBaseEsq - ajuste;
        } else if (erroP < -1) { // Desviou para a esquerda, corrigir para direita
          velE = veloBaseDir + ajuste;
          velD = veloBaseEsq; 
        } else { // Está alinhado
          velE = veloBaseDir;
          velD = veloBaseEsq;
        }
      } else {
        // Lógica normal para movimento para frente
        if (erroP > 1) { // Desviou para a direita, corrigir para esquerda
          velE = veloBaseEsq - ajuste;
          velD = veloBaseDir;
        } else if (erroP < -1) { // Desviou para a esquerda, corrigir para direita
          velE = veloBaseEsq;
          velD = veloBaseDir + ajuste;
        } else { // Está alinhado
          velE = veloBaseEsq;
          velD = veloBaseDir;
        }
      }
      
      // Constrain das velocidades
      velE = constrain(velE, 0, 180);
      velD = constrain(velD, 0, 180);
      
      // Aplicar velocidades
      motorE.write(velE);
      motorD.write(velD);
      
      ultimaCorrecao = millis();
      
      // Debug opcional
      // if (lambdasContados % 5 == 0 || abs(erroP) > 3) { 
      //   Serial.print("Hall: "); Serial.print(leituraAtual);
      //   Serial.print(" | Lambdas: "); Serial.print(lambdasContados);
      //   Serial.print("/"); Serial.print(lambdasNecessarios);
      //   Serial.print(" | Ignorar próxima: "); Serial.print(ignorarPrimeiraDeteccao ? "SIM" : "NÃO");
      //   Serial.print(" | Direção: "); Serial.println(praTras ? "TRÁS" : "FRENTE");
      // }
    }
    
    delay(10); // Pequeno delay para não sobrecarregar
  }
  
  // Para os motores
  motorE.write(90);
  motorD.write(90);
  delay(100);
  
  Serial.print("Movimento "); Serial.print(praTras ? "para TRÁS" : "para FRENTE");
  Serial.print(" de "); Serial.print(distanciaCm); 
  Serial.println(" cm concluído com correção PID!");
}

const int distanciaMaxima = 30; //25
const int distanciaParede = 50; //50
const int distanciaSaida = 60; //60

void resgate(){
  Serial.println(F("Resgate detectado!"));
  tocar_buzzer(500, 2, 25);
  
  motorE.write(90);
  motorD.write(90);

  if(retornoAnguloY() > anguloDoReto + erroRampaDescida || retornoAnguloY() < anguloDoReto - erroRampaDescida) {
    Serial.println("Não é resgate 1!");
    tocar_buzzer(500, 2, 25);
    return;
  }else if (mediaInfravermelho(1,2) >= distanciaMaxima && mediaInfravermelho(3,2) >= distanciaMaxima) {
    Serial.println("Não é resgate 2!");
    tocar_buzzer(500, 2, 25);
    return;
  }

  int valorEsquerda = mediaInfravermelho(1);
  int valorDireita = mediaInfravermelho(3);

  Serial.print("Valor Sensor Direita: "); Serial.println(valorDireita);
  Serial.print("Valor Sensor Esquerda: "); Serial.println(valorEsquerda);
  Serial.print("distanciaMaxima: "); Serial.println(distanciaMaxima);

  bool direita = valorDireita <= distanciaMaxima;
  bool esquerda = valorEsquerda <= distanciaMaxima;

  ligarGarra();
  garraMeio();
  desligarMotoresGarra();
  delay(250);

  //*************************************** */
  //
  //                PARTE 00
  //        Alinha na parede lateral
  //    
  //************************************** */

  andarRetoComHall(12, anguloReto);

  if(esquerda){
    curva90Direita();
  }else if(direita){
    curva90Esquerda();
  }

  andarPraTrasPorTempo(2000);
  delay(100);
  anguloReto = retornoAnguloZ();

  //*************************************** */
  //
  //                PARTE 01
  //  Vai até o meio para alinhar novamente  
  //
  //*************************************** */

  andarRetoComHall(30, anguloReto);
  
  //*************************************** */
  //
  //                PARTE 02
  //    Verifica se tem o triângulo na frente
  //           e alinha na parede
  //
  //*************************************** */

  if(esquerda){
    curva90Esquerda();
  }else if(direita){
    curva90Direita();
  }

  andarPraTrasPorTempo(2500);
  anguloReto = retornoAnguloZ();
  delay(250);

  bool paredeNaFrenteInicio, saidaFrenteInicio;

  if(esquerda){
    paredeNaFrenteInicio = mediaInfravermelho(3) <= distanciaParede;
    saidaFrenteInicio = mediaInfravermelho(3) >= distanciaSaida;
  }else if(direita){
    paredeNaFrenteInicio = mediaInfravermelho(1) <= distanciaParede;
    saidaFrenteInicio = mediaInfravermelho(1) >= distanciaSaida;
  }

  Serial.print("Parede na frente: "); Serial.println(paredeNaFrenteInicio);
  Serial.print("Saída na frente: "); Serial.println(saidaFrenteInicio);

  ligarGarra();
  abrirGarra();
  descerGarra();
  delay(250);

  //*************************************** */
  //
  //                PARTE 03
  //    Vai pro meio com a garra aberta
  // fecha, perto da borda, via mais pra frente
  //      e verifica as saidas pros cantos
  //
  //*************************************** */

  andarRetoComHall(25, anguloReto);
  bool saidaEsquerdaMeio = mediaInfravermelho(1) > 65; 
  bool saidaDireitaMeio = mediaInfravermelho(3) > 65;
  tocar_buzzer(1000, 1, 100);
  
  andarRetoComHall(20, anguloReto);
  fecharGarra();
  garraMeio();
  //desligarMotoresGarra();
  delay(250);
  andarRetoComHall(13, anguloReto);

  //Carrinho está la na frente

  bool saidaFrente = mediaInfravermelho(2) > distanciaMaxima;
  bool trianguloEsquerda = mediaInfravermelho(1) <= distanciaMaxima;
  bool trianguloDireita = mediaInfravermelho(3) <= distanciaMaxima;

  Serial.print("Triângulo Esquerda: "); Serial.println(trianguloEsquerda);
  Serial.print("Triângulo Direita: "); Serial.println(trianguloDireita);

  if(trianguloDireita && trianguloEsquerda){
    trianguloEsquerda == false;
  }

  if(trianguloDireita || trianguloEsquerda){
    andarRetoComHall(30, anguloReto, true);

    if(trianguloDireita){
      curvaComHall(1, 45);
    }else if(trianguloEsquerda){
      curvaComHall(-1, 45);
    }

    anguloReto = retornoAnguloZ();

    unsigned long startTime = millis();
    while(digitalRead(no) == HIGH){
      andarRetoPorTempo(50);
    }

    //ligarGarra();
    garra90();
    abrirGarra();
    delay(2500);
    fecharGarra();
    subirGarra();
    delay(500);
    desligarGarra();
    delay(500);

    andarRetoComHall(25, anguloReto, true);
    delay(500);

    if(trianguloDireita){
      curvaComHall(-1, 55);

    }else if(trianguloEsquerda){
      curvaComHall(1, 55);

    }

    anguloReto = retornoAnguloZ();

  }else{
    tocar_buzzer(1000, 5, 500);
  }
  
  //**************************************** */
  //
  //                PARTE 04
  //          Vai atrás da saida
  //
  //**************************************** */

  Serial.println("=== [DEBUG] Iniciando busca pela saída ===");
  Serial.print(F("Saida na frente em cima: ")); Serial.println(saidaFrente);
  Serial.print(F("Saida na direita meio: ")); Serial.println(saidaDireitaMeio);
  Serial.print(F("Saida na esquerda meio: ")); Serial.println(saidaEsquerdaMeio);
  Serial.print(F("Saida na frente no inicio: ")); Serial.println(saidaFrenteInicio);

  if(saidaFrente){
    Serial.println("Saindo pela frente");
    sairFrente();
    andarRetoComHall(3, anguloReto);
    return;
  }else if(saidaDireitaMeio){
    Serial.println("Saindo pela direita meio");
    sairMeioDireita(); 
    return;
  }else if(saidaEsquerdaMeio){
    Serial.println("Saindo pela esquerda meio");
    sairMeioEsquerda();
    return;
  }else if(saidaFrenteInicio){
    Serial.println("Saindo pela frente no inicio");

    return;
  }
}

int retornoAnguloY(){
  giro.update();
  return giro.getAngleY();
}

// Retorna: 0 = nenhum, 1 = direita, 2 = esquerda, 3 = ambos
int verificaVerdeNovamente(int delayy, int delay2=100) {
  motorE.write(70);
  motorD.write(130);
  delay(delayy);

  motorE.write(90);
  motorD.write(90);
  delay(delay2);
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  int cc1 = (int)c1;
  int cc2 = (int)c2;
  int luu1 = (int)lux1;
  int luu2 = (int)lux2;

  Serial.print(F("Direita - R: ")); Serial.print(r1);
  Serial.print(F(" G: ")); Serial.print(g1);
  Serial.print(F(" B: ")); Serial.print(b1);
  Serial.print(F(" C: ")); Serial.print(cc1);
  Serial.print(F(" Lux: ")); Serial.println(luu1);

  Serial.print(F("Esquerda - R: ")); Serial.print(r2);
  Serial.print(F(" G: ")); Serial.print(g2);
  Serial.print(F(" B: ")); Serial.print(b2);
  Serial.print(F(" C: ")); Serial.print(cc2);
  Serial.print(F(" Lux: ")); Serial.println(luu2);

  Serial.print(F("Limites Direita: minC=")); Serial.print(minCVerdeDir);
  Serial.print(F(" maxC=")); Serial.print(maxCVerdeDir);
  Serial.print(F(" minLux=")); Serial.print(minLuxVerdeDir);
  Serial.print(F(" maxLux=")); Serial.print(maxLuxVerdeDir);
  Serial.print(F(" difCor=")); Serial.println(diferencaDasCoresDir);

  Serial.print(F("Limites Esquerda: minC=")); Serial.print(minCVerdeEsq);
  Serial.print(F(" maxC=")); Serial.print(maxCVerdeEsq);
  Serial.print(F(" minLux=")); Serial.print(minLuxVerdeEsq);
  Serial.print(F(" maxLux=")); Serial.print(maxLuxVerdeEsq);
  Serial.print(F(" difCor=")); Serial.println(diferencaDasCoresEsq);

  bool verdeDireita = (
    g1 > r1 && g1 > b1 &&
    (g1 - r1) > diferencaDasCoresDir &&
    cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir &&
    luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir
  );
  bool verdeEsquerda = (
    g2 > r2 && g2 > b2 &&
    (g2 - r2) > diferencaDasCoresEsq &&
    cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq &&
    luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq
  );

  Serial.print("verdeDireita: "); Serial.println(verdeDireita);
  Serial.print("verdeEsquerda: "); Serial.println(verdeEsquerda);

  int resultado = 0;
  if (verdeDireita && verdeEsquerda) resultado = 3;
  else if (verdeDireita) resultado = 1;
  else if (verdeEsquerda) resultado = 2;

  Serial.print("Resultado verificaVerdeNovamente: ");
  if (resultado == 3) Serial.println("3 (Ambos)");
  else if (resultado == 1) Serial.println("1 (Direita)");
  else if (resultado == 2) Serial.println("2 (Esquerda)");
  else Serial.println("0 (Nenhum)");

  Serial.println("=== [DEBUG] Fim verificaVerdeNovamente ===");
  return resultado;
}

void giroVerde(bool encrusilhada = false) {
  Serial.println("Giro Verde");
  int resultado1 = verificaVerdeNovamente(0);
  Serial.print("[ANTES] Resultado verificaVerdeNovamente: ");
  Serial.println(resultado1);

  tocar_buzzer(1000, 1, 100);
  giro.update();
 
  if (resultado1 == 3) {
    Serial.println("VERDE!! Curva 180°");
    tocar_buzzer(1000, 3, 100);

    curva180Direita();

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
    delay(tempoDepoisDoVerde180);
    anguloReto = retornoAnguloZ();
    Serial.print("Angulo Reto: "); Serial.println(anguloReto);
    Serial.print("Angulo Atual: "); Serial.println(retornoAnguloZ());
    erroI = 0;
  } else if (resultado1 == 1) {
    Serial.println("Verde só na direita!");
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);

    if(encrusilhada){
      delay(delayCurvasverde + 250);
    }else{
      delay(delayCurvasverde);
    }

    curva90Direita();

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
    delay(tempoDepoisDoVerde90);
    anguloReto = retornoAnguloZ();
    erroI = 0;
  } else if (resultado1 == 2) {
    Serial.println("Verde só na esquerda!");
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
    
    if(encrusilhada){
      delay(delayCurvasverde + 250);
    }else{
      delay(delayCurvasverde);
    }
    
    curva90Esquerda();

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
    delay(tempoDepoisDoVerde90);
    anguloReto = retornoAnguloZ();
    erroI = 0;
  } else {
    Serial.println("Nenhum verde detectado.");
    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    delay(tempoDepoisDoVerdeFalso);
  }
}

//Esquerda + angulo
//Direita - angulo


void correcao() {
  sl = lerSensoresLinha();
  anguloAtual = retornoAnguloZ();
  
  if (sl[0] == 0 || sl[1] == 0 || sl[2] == 0 || sl[3] == 0 || sl[4] == 0) {
    return;
  } else {    
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
    ajuste = constrain(ajuste, 0, 30);

    if (erroP > 0) {
      motorE.write(veloBaseEsq - ajuste);
      motorD.write(veloBaseDir);
    } else if (erroP < 0) {
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir + ajuste);
    } else {
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
    }
    verificaCores();
  }
}

void correcaoObjeto() {
  anguloAtual = retornoAnguloZ();
  if (anguloReto - erro > anguloAtual) {
    motorE.write(50);
    motorD.write(160);
  }
  else if (anguloReto + erro < anguloAtual) {
    motorE.write(20);
    motorD.write(130);
  }
  else if (abs(anguloReto - anguloAtual) <= erro) {
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
  }
}

void correcaoRe(){
  anguloAtual = retornoAnguloZ();
  if (anguloReto - erro > anguloAtual) {
    motorE.write(160);
    motorD.write(50);
  }
  else if (anguloReto + erro < anguloAtual) {
    motorE.write(120);
    motorD.write(20);
  }
  else if (abs(anguloReto - anguloAtual) <= erro) {
    motorE.write(veloBaseDir);
    motorD.write(veloBaseEsq);
  }
}

void desvioObjeto() {
  if (SI_Frente.distance() <= distanciaDesvio) {
    int distaciaInicial; // Alterar para o ultrassonico
    motorD.write(veloBaseDir + veloCurva90);
    motorE.write(veloBaseEsq - veloCurva90);
    tocar_buzzer(500, 1, 100);    

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

static bool estavaDesalinhado = false;
static bool estavaDesalinhadoMais = false;

void andarReto() {
  giro.update();
  sl = lerSensoresLinha();

  int combinacaoSensores = sl[0] * 16 + sl[1] * 8 + sl[2] * 4 + sl[3] * 2 + sl[4];

  if(retornoAnguloY() > (anguloDoReto + erroRampaDescida)) {
    while (retornoAnguloY() > (anguloDoReto + erroRampaDescida))
    {
      Serial.println(F("Descida detectada!"));
      Serial.println(retornoAnguloY());
      motorD.write(veloBaseDir - veloCurva90);
      motorE.write(veloBaseEsq + veloCurva90);
    }
  }

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
      ajuste = constrain(ajuste, 0, 30);

      if (erroP > 0) {
        motorE.write(veloBaseEsq - ajuste);
        motorD.write(veloBaseDir);
      } else if (erroP < 0) {
        motorE.write(veloBaseEsq);
        motorD.write(veloBaseDir + ajuste);
      } else {
        motorE.write(veloBaseEsq);
        motorD.write(veloBaseDir);
      }

      Serial.print(F("Angulo Atual: ")); Serial.print(anguloAtual);
      Serial.print(F(" | Angulo Reto: ")); Serial.print(anguloReto);
      Serial.print(F(" | Ajuste: ")); Serial.print(ajuste);
      Serial.print(F(" | P: ")); Serial.print(Kp * erroP);
      Serial.print(F(" | I: ")); Serial.print(Ki * erroI);
      Serial.print(F(" | D: ")); Serial.println(Kd * erroD);

      if (estavaDesalinhado && estavaDesalinhadoMais) {
        anguloReto = (anguloAtual + anguloReto) / 2;
        erroI = 0;
        Serial.print(F("Novo angulo RETO (centralizado 1): ")); Serial.println(anguloReto);
        estavaDesalinhado = false;
        estavaDesalinhadoMais = false;
      }else if(estavaDesalinhado) {
        anguloReto = (anguloAtual + anguloReto);
        erroI = 0;
        Serial.print(F("Novo angulo RETO (centralizado 2): ")); Serial.println(anguloReto);
        estavaDesalinhado = false;
      }else if(estavaDesalinhadoMais) {
        anguloReto = (anguloAtual + anguloReto);
        erroI = 0;
        Serial.print(F("Novo angulo RETO (centralizado 3): ")); Serial.println(anguloReto);
        estavaDesalinhadoMais = false;
      }

      desvioObjeto();

      break;

    case 0b10011: // Pequena curva esquerda
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir + pequenaCurvaLadoC);
      Serial.println(F("Pequena curva esquerda"));
      estavaDesalinhado = true;
      giro.update();
      desvioObjeto();
      break;

    case 0b00011: // Curva falsa ou verde
      Serial.println(F("Curva falsa OU verde"));
      motorE.write(veloBaseDir);
      motorD.write(veloBaseEsq);
      delay(100);
      motorE.write(90);
      motorD.write(90);
      delay(250);
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      giro.update();
      break;

    case 0b00111: // Curva esquerda
      Serial.println(F("Curva esquerda"));
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      curva90Esquerda();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = retornoAnguloZ();

      erroI = 0;
      Serial.print(F("Novo angulo RETO : ")); Serial.println(anguloReto);
      break;

    case 0b11001: // Pequena curva direita
      Serial.println(F("Pequena curva direita"));
      motorE.write(veloBaseEsq - pequenaCurvaLadoC);
      motorD.write(veloBaseDir);
      estavaDesalinhado = true;
      desvioObjeto();
      giro.update();
      break;

    case 0b11000: // Curva falsa ou verde
      Serial.println(F("Curva falsa OU verde"));
      motorE.write(veloBaseDir);
      motorD.write(veloBaseEsq);
      delay(100);
      motorE.write(90);
      motorD.write(90);
      delay(250);
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
      giro.update();
      break;

    case 0b11100: // Curva direita
      Serial.println(F("Curva direita"));
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      curva90Direita();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = retornoAnguloZ();
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);

      break;

    case 0b00000:
      motorE.write(veloBaseDir);
      motorD.write(veloBaseEsq);
      delay(100);
      motorE.write(90);
      motorD.write(90);
      delay(250);
      Serial.println("Início da pista, ou encruzilhada");

      giroVerde(true);
      break;

    case 0b10111: // Saiu do principal, esquerda
      Serial.println("Saiu do principal, esquerda");
      motorE.write(80);
      motorD.write(veloBaseDir);
      desvioObjeto();
      giro.update();
      estavaDesalinhadoMais = true;
      break;

    case 0b11101: // Saiu do principal, direita
      Serial.println("Saiu do principal, direita");
      motorE.write(veloBaseEsq);
      motorD.write(100);
      desvioObjeto();
      giro.update();
      estavaDesalinhadoMais = true;
      break;

    case 0b11111: // Branco, final ou resgate
      correcao();
      break;

    case 0b00100: // T
      Serial.println("T");
      motorE.write(veloBaseDir);
      motorD.write(veloBaseEsq);
      delay(100);
      motorE.write(90);
      motorD.write(90);
      delay(250);

      giroVerde(true);
      break;

    case 0b01111: // Recuperando a linha, esquerda
      Serial.println(F("Recuperando a linha, esquerda"));
      while(sl[2] == 1 && sl[1] == 1 && sl[0] == 0){
        sl = lerSensoresLinha();
        motorE.write(90);
        motorD.write(veloBaseDir);
        giro.update();
      }
      anguloAtual = retornoAnguloZ();
      break;

    case 0b11110: // Recuperando a linha, direita
      Serial.println(F("Recuperando a linha, direita"));
      while(sl[4] == 0 && sl[3] == 1 && sl[2] == 1){
        sl = lerSensoresLinha();
        motorE.write(90);
        motorD.write(veloBaseDir);
        giro.update();
      }
      anguloAtual = retornoAnguloZ();
      break;

    case 0b10100:
      Serial.println("Curva direita 2");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      curva90Direita();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = retornoAnguloZ();
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);
      break;

    case 0b00101:
      Serial.println("Curva esquerda 2");
      sl = lerSensoresLinha();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoAntesCurva90);

      curva90Esquerda();

      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);

      delay(tempoDepoisCurva90);

      anguloReto = retornoAnguloZ();
      erroI = 0;
      Serial.print("Novo angulo RETO : "); Serial.println(anguloReto);
      break;

    case 0b10000: // Curva falsa ou verde
      motorD.write(veloBaseEsq);
      motorE.write(veloBaseDir);
      delay(50);
      giroVerde(true);
      motorD.write(veloBaseDir);
      motorE.write(veloBaseEsq);
      delay(tempoDepoisDoVerdeFalso);
      break;

    case 0b00001: // Curva falsa ou verde
      motorD.write(veloBaseEsq);
      motorE.write(veloBaseDir);
      delay(100);
      giroVerde(true);
      motorD.write(veloBaseDir);
      motorE.write(veloBaseEsq);
      delay(tempoDepoisDoVerdeFalso);
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
  delay(500);
}

void desligarGarra(){
  motorG.detach();
  motorEsqG.detach();
  motorDirG.detach();
  delay(500);
}

void desligarMotorPrincipal() {
  motorG.detach();
  delay(500);
}

void desligarMotoresGarra() {
  motorEsqG.detach();
  motorDirG.detach();
  delay(500);
}

void descerGarra() {
  Serial.println("Descendo Garra");
  motorG.write(170);
  delay(1000);
}

void garraMeio(){
  Serial.println("Garra Meio");
  motorG.write(55);
  delay(1000);
}

void garra90(){
  Serial.println("Garra 90°");
  motorG.write(90);
}

void subirGarra() {
  Serial.println("Subindo Garra");
  motorG.write(10);
  delay(1000);
}

void abrirGarra() {
  Serial.println("Abrindo Garra");
  motorEsqG.write(175);
  motorDirG.write(0);
  delay(250);
}

void fecharGarra() {
  Serial.println("Fechando Garra");
  motorEsqG.write(90);
  motorDirG.write(90);
  delay(250);
}

void fechar_bolinha(){
  motorG.write(170);
  delay(100);
  motorEsqG.write(160);
  motorDirG.write(40);
  delay(100);
  motorEsqG.write(140);
  motorDirG.write(60);
  delay(100);
  motorEsqG.write(120);
  motorDirG.write(80);
  delay(100);
  motorEsqG.write(120);
  motorDirG.write(110);
  delay(100);
  motorEsqG.write(100);
  motorDirG.write(100);
  delay(100);
  motorG.write(165);
  delay(100);
  motorG.write(160);
  delay(100);
  motorG.write(155);
  delay(100);
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

  Serial.print("Esquerda: "); Serial.print(d2); Serial.print("cm - ");
  Serial.print("Frente  : "); Serial.print(d1); Serial.print("cm - ");
  Serial.print("Direita : "); Serial.print(d3); Serial.print("cm - ");
  Serial.println(" ");
}

int menuAtual = 1;

void exibirMenuPrincipal() {
  Serial.println(F("=== Menu 01 ==="));
  Serial.println(F("1 - Ler sensores de cor"));
  Serial.println(F("2 - Calculos"));
  Serial.println(F("3 - Calibrar Verde"));
  Serial.println(F("4 - Calibrar Vermelho"));
  Serial.println(F("5 - Calibrar Cinza"));
  Serial.println(F("6 - Ajustar Branco Manual"));
  Serial.println(F("7 - Ajustar Verde Manual"));
  Serial.println(F("8 - Ajustar Vermelho Manual"));
  Serial.println(F("9 - Exibir valores da EEPROM"));
  Serial.println(F("0 - Sair"));
  Serial.println(F("================"));
}

void exibirMenuCalculos() {
  Serial.println(F("=== Menu 02 - Calculos ==="));
  Serial.println(F("1 - Calculos do Verde"));
  Serial.println(F("2 - Calculos do Vermelho"));
  Serial.println(F("3 - Calculos do Cinza"));
  Serial.println(F("4 - Voltar"));
  Serial.println(F("================"));
}

void exibirMenuAjusteVerde() {
  Serial.println(F("=== Menu 03 - Ajustar verde manual ==="));
  Serial.println(F("1 - Ler sensores de cor"));
  Serial.println(F("2 - Calculo do verde"));
  Serial.println(F("3 - maxLuxVerde"));
  Serial.println(F("4 - minLuxVerde"));
  Serial.println(F("5 - maxCVerde"));
  Serial.println(F("6 - minCVerde"));
  Serial.println(F("7 - Valores EEPROM Verde"));
  Serial.println(F("8 - Alterar diferenca cores verde"));
  Serial.println(F("9 - Voltar"));
  Serial.println(F("================"));
}

void exibirMenuAjusteVermelho() {
  Serial.println(F("=== Menu 04 - Ajustar vermelho manual ==="));
  Serial.println(F("1 - Ler sensores de cor"));
  Serial.println(F("2 - Calculo do Vermelho"));
  Serial.println(F("3 - maxLuxVermelho"));
  Serial.println(F("4 - minLuxVermelho"));
  Serial.println(F("5 - maxCVermelho"));
  Serial.println(F("6 - minCVermelho"));
  Serial.println(F("7 - Valores EEPROM Vermelho"));
  Serial.println(F("8 - DiferencaDasCoresVermelho"));
  Serial.println(F("9 - Voltar"));
  Serial.println(F("================"));
}

void exibirMenuAjusteCinza() {
  Serial.println(F("=== Menu 05 - Ajustar cinza manual ==="));
  Serial.println(F("1 - Ler sensores de cor"));
  Serial.println(F("2 - Calculo do Cinza"));
  Serial.println(F("3 - maxLuxCinza"));
  Serial.println(F("4 - minLuxCinza"));
  Serial.println(F("5 - maxCCinza"));
  Serial.println(F("6 - minCCinza"));
  Serial.println(F("7 - Valores EEPROM Cinza"));
  Serial.println(F("8 - Voltar"));
  Serial.println(F("================"));
}

void calibrarVerdeMedia() {
  Serial.println(F("=== Calibração do Verde (Média Individual) ==="));
  Serial.println(F("Coloque o sensor DIREITO sobre o VERDE e envie qualquer tecla para iniciar..."));
  while (!Serial.available()) { delay(10); }
  Serial.read();

  const int amostras = 10;
  long somaLuxDir = 0, somaCDir = 0, somaDifDir = 0;
  long somaLuxEsq = 0, somaCEsq = 0, somaDifEsq = 0;

  // Sensor DIREITO
  for (int i = 0; i < amostras; i++) {
    // Movimento para frente e para trás alternado
    if (i % 2 == 0) {
      motorE.write(110); // frente
      motorD.write(70);
    } else {
      motorE.write(70);  // trás
      motorD.write(110);
    }
    delay(100); // tempo de movimento

    motorE.write(90); // para
    motorD.write(90);
    delay(250);

    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    int cVerde = (int)c1;
    int luxVerde = (int)lux1;
    int difVerde = (int)(g1 - r1);

    somaLuxDir += luxVerde;
    somaCDir += cVerde;
    somaDifDir += difVerde;

    Serial.print("[Direita] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxVerde);
    Serial.print(" | C: "); Serial.print(cVerde);
    Serial.print(" | G-R: "); Serial.println(difVerde);

    delay(150);
  }

  Serial.println("Agora coloque o sensor ESQUERDO sobre o VERDE e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(25); }
  Serial.read();

  // Sensor ESQUERDO
  for ( int i = 0; i < amostras; i++) {
    // Movimento para frente e para trás alternado
    if (i % 2 == 0) {
      motorE.write(110); // frente
      motorD.write(70);
    } else {
      motorE.write(70);  // trás
      motorD.write(110);
    }
    delay(100);

    motorE.write(90); // para
    motorD.write(90);
    delay(250);

    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux2 = tcs2.calculateLux(r2, g2, b2);
    int cVerde = (int)c2;
    int luxVerde = (int)lux2;
    int difVerde = (int)(g2 - r2);

    somaLuxEsq += luxVerde;
    somaCEsq += cVerde;
    somaDifEsq += difVerde;

    Serial.print("[Esquerda] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxVerde);
    Serial.print(" | C: "); Serial.print(cVerde);
    Serial.print(" | G-R: "); Serial.println(difVerde);

    delay(150);
  }

  // Médias individuais
  int mediaLuxDir = somaLuxDir / amostras;
  int mediaCDir = somaCDir / amostras;
  int mediaDifDir = somaDifDir / amostras;

  int mediaLuxEsq = somaLuxEsq / amostras;
  int mediaCEsq = somaCEsq / amostras;
  int mediaDifEsq = somaDifEsq / amostras;

  // Margens (ajuste conforme necessário)
  int minLuxDir = mediaLuxDir * 0.6;
  int maxLuxDir = mediaLuxDir * 1.75; //1.85
  int minCDir = mediaCDir * 0.6;
  int maxCDir = mediaCDir * 1.75; //1.85

  int minLuxEsq = mediaLuxEsq * 0.6;
  int maxLuxEsq = mediaLuxEsq * 1.75; //1.85
  int minCEsq = mediaCEsq * 0.6;
  int maxCEsq = mediaCEsq * 1.75; //1.85

  int margem = 40;
  int difCorDir = mediaDifDir - margem;
  int difCorEsq = mediaDifEsq - margem;
  if (difCorDir < 1) difCorDir = 1;
  if (difCorEsq < 1) difCorEsq = 1;

  Serial.println(F("Calibração concluída!"));
  Serial.print(F("mediaLuxVerdeDir: ")); Serial.println(mediaLuxDir);
  Serial.print(F("mediaCVerdeDir: ")); Serial.println(mediaCDir);
  Serial.print(F("mediaDifVerdeDir (G-R): ")); Serial.println(mediaDifDir);
  Serial.print(F("mediaLuxVerdeEsq: ")); Serial.println(mediaLuxEsq);
  Serial.print(F("mediaCVerdeEsq: ")); Serial.println(mediaCEsq);
  Serial.print(F("mediaDifVerdeEsq (G-R): ")); Serial.println(mediaDifEsq);

  EEPROM.put(EEPROM_MIN_LUX_VERDE_DIR, minLuxDir);
  EEPROM.put(EEPROM_MAX_LUX_VERDE_DIR, maxLuxDir);
  EEPROM.put(EEPROM_MIN_C_VERDE_DIR, minCDir);
  EEPROM.put(EEPROM_MAX_C_VERDE_DIR, maxCDir);
  EEPROM.put(EEPROM_DIFERENCA_CORES_DIR, difCorDir);

  EEPROM.put(EEPROM_MIN_LUX_VERDE_ESQ, minLuxEsq);
  EEPROM.put(EEPROM_MAX_LUX_VERDE_ESQ, maxLuxEsq);
  EEPROM.put(EEPROM_MIN_C_VERDE_ESQ, minCEsq);
  EEPROM.put(EEPROM_MAX_C_VERDE_ESQ, maxCEsq);
  EEPROM.put(EEPROM_DIFERENCA_CORES_ESQ, difCorEsq);

  minLuxVerdeDir = minLuxDir; maxLuxVerdeDir = maxLuxDir;
  minCVerdeDir = minCDir; maxCVerdeDir = maxCDir;
  diferencaDasCoresDir = difCorDir;
  minLuxVerdeEsq = minLuxEsq; maxLuxVerdeEsq = maxLuxEsq;
  minCVerdeEsq = minCEsq; maxCVerdeEsq = maxCEsq;
  diferencaDasCoresEsq = difCorEsq;
}

void calibrarVermelhoMedia() {
  Serial.println(F("=== Calibração do Vermelho (Média) ==="));
  Serial.println(F("Coloque o sensor DIREITO sobre o VERMELHO e envie qualquer tecla para iniciar..."));
  while (!Serial.available()) { delay(10); }
  Serial.read();

  const int amostras = 10;
  long somaLux1 = 0, somaC1 = 0, somaDif1 = 0;
  long somaLux2 = 0, somaC2 = 0, somaDif2 = 0;

  // Sensor DIREITO (coleta dos dois sensores)
  for (int i = 0; i < amostras; i++) {
    // Movimento para frente e para trás alternado
    if (i % 2 == 0) {
      motorE.write(110); // frente
      motorD.write(70);
    } else {
      motorE.write(70);  // trás
      motorD.write(110);
    }
    delay(200);

    motorE.write(90); // para
    motorD.write(90);
    delay(100);

    tcs1.getRawData(&r1, &g1, &b1, &c1);
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    lux2 = tcs2.calculateLux(r2, g2, b2);

    int cVermelho1 = (int)c1;
    int luxVermelho1 = (int)lux1;
    int difVermelho1 = (int)(r1 - g1);

    int cVermelho2 = (int)c2;
    int luxVermelho2 = (int)lux2;
    int difVermelho2 = (int)(r2 - g2);

    somaLux1 += luxVermelho1;
    somaC1 += cVermelho1;
    somaDif1 += difVermelho1;

    somaLux2 += luxVermelho2;
    somaC2 += cVermelho2;
    somaDif2 += difVermelho2;

    Serial.print(F("[Direita] Amostra ")); Serial.print(i+1);
    Serial.print(F(" | Lux1: ")); Serial.print(luxVermelho1);
    Serial.print(F(" | C1: ")); Serial.print(cVermelho1);
    Serial.print(F(" | R1-G1: ")); Serial.print(difVermelho1);
    Serial.print(F(" || Lux2: ")); Serial.print(luxVermelho2);
    Serial.print(F(" | C2: ")); Serial.print(cVermelho2);
    Serial.print(F(" | R2-G2: ")); Serial.println(difVermelho2);

    delay(150);
  }

  Serial.println("Agora coloque o sensor ESQUERDO sobre o VERMELHO e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read();

  // Sensor ESQUERDO (coleta dos dois sensores)
  for (int i = 0; i < amostras; i++) {
    if (i % 2 == 0) {
      motorE.write(110); // frente
      motorD.write(70);
    } else {
      motorE.write(70);  // trás
      motorD.write(110);
    }
    delay(200);

    motorE.write(90); // para
    motorD.write(90);
    delay(100);

    tcs1.getRawData(&r1, &g1, &b1, &c1);
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    lux2 = tcs2.calculateLux(r2, g2, b2);

    int cVermelho1 = (int)c1;
    int luxVermelho1 = (int)lux1;
    int difVermelho1 = (int)(r1 - g1);

    int cVermelho2 = (int)c2;
    int luxVermelho2 = (int)lux2;
    int difVermelho2 = (int)(r2 - g2);

    somaLux1 += luxVermelho1;
    somaC1 += cVermelho1;
    somaDif1 += difVermelho1;

    somaLux2 += luxVermelho2;
    somaC2 += cVermelho2;
    somaDif2 += difVermelho2;

    Serial.print(F("[Esquerda] Amostra ")); Serial.print(i+1);
    Serial.print(F(" | Lux1: ")); Serial.print(luxVermelho1);
    Serial.print(F(" | C1: ")); Serial.print(cVermelho1);
    Serial.print(F(" | R1-G1: ")); Serial.print(difVermelho1);
    Serial.print(F(" || Lux2: ")); Serial.print(luxVermelho2);
    Serial.print(F(" | C2: ")); Serial.print(cVermelho2);
    Serial.print(F(" | R2-G2: ")); Serial.println(difVermelho2);

    delay(150);
  }

  int mediaLux1 = somaLux1 / (amostras * 2);
  int mediaC1   = somaC1   / (amostras * 2);
  int mediaDif1 = somaDif1 / (amostras * 2);

  int mediaLux2 = somaLux2 / (amostras * 2);
  int mediaC2   = somaC2   / (amostras * 2);
  int mediaDif2 = somaDif2 / (amostras * 2);

  // Agora tira a média entre os dois sensores
  int mediaLux = (mediaLux1 + mediaLux2) / 2;
  int mediaC   = (mediaC1   + mediaC2)   / 2;
  int mediaDif = (mediaDif1 + mediaDif2) / 2;

  int minLux = mediaLux * 0.7;
  int maxLux = mediaLux * 1.85;
  int minC = mediaC * 0.7;
  int maxC = mediaC * 1.85;

  int margem = 20;
  int diferencaAjustada = mediaDif - margem;
  if (diferencaAjustada < 1) diferencaAjustada = 1;

  Serial.println(F("Calibração concluída!"));
  Serial.print(F("mediaLuxVermelho: ")); Serial.println(mediaLux);
  Serial.print(F("mediaCVermelho: ")); Serial.println(mediaC);
  Serial.print(F("mediaDifVermelho (R-G): ")); Serial.println(mediaDif);
  Serial.print(F("diferencaDasCoresVermelho usada: ")); Serial.println(diferencaAjustada);
  Serial.print(F("minLuxVermelho (60%): ")); Serial.println(minLux);
  Serial.print(F("maxLuxVermelho (140%): ")); Serial.println(maxLux);
  Serial.print(F("minCVermelho (60%): ")); Serial.println(minC);
  Serial.print(F("maxCVermelho (140%): ")); Serial.println(maxC);

  EEPROM.put(EEPROM_minLuxVermelho, minLux);
  EEPROM.put(EEPROM_maxLuxVermelho, maxLux);
  EEPROM.put(EEPROM_minCVermelho, minC);
  EEPROM.put(EEPROM_maxCVermelho, maxC);
  EEPROM.put(EEPROM_diferencaDasCoresVermelho, diferencaAjustada);

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
  Serial.println("Coloque o sensor DIREITO sobre o CINZA e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read();

  const int amostras = 20;
  long somaLux1 = 0, somaC1 = 0;
  long somaLux2 = 0, somaC2 = 0;

  // Sensor DIREITO
  for (int i = 0; i < amostras; i++) {
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    lux1 = tcs1.calculateLux(r1, g1, b1);
    int cCinza = (int)c1;
    int luxCinza = (int)lux1;

    somaLux1 += luxCinza;
    somaC1 += cCinza;

    Serial.print("[Direita] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxCinza);
    Serial.print(" | C: "); Serial.println(cCinza);

    delay(150);
  }

  Serial.println("Agora coloque o sensor ESQUERDO sobre o CINZA e envie qualquer tecla para iniciar...");
  while (!Serial.available()) { delay(10); }
  Serial.read();

  // Sensor ESQUERDO
  for (int i = 0; i < amostras; i++) {
    tcs2.getRawData(&r2, &g2, &b2, &c2);
    lux2 = tcs2.calculateLux(r2, g2, b2);
    int cCinza = (int)c2;
    int luxCinza = (int)lux2;

    somaLux2 += luxCinza;
    somaC2 += cCinza;

    Serial.print("[Esquerda] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux: "); Serial.print(luxCinza);
    Serial.print(" | C: "); Serial.println(cCinza);

    delay(150);
  }

  int mediaLux = (somaLux1 + somaLux2) / (2 * amostras);
  int mediaC = (somaC1 + somaC2) / (2 * amostras);

  int minLux = mediaLux * 0.8;
  int maxLux = mediaLux * 1.2;
  int minC = mediaC * 0.8;
  int maxC = mediaC * 1.2;

  Serial.println(F("Calibração concluída!"));
  Serial.print(F("mediaLuxCinza: ")); Serial.println(mediaLux);
  Serial.print(F("mediaCCinza: ")); Serial.println(mediaC);
  Serial.print(F("minLuxCinza (80%): ")); Serial.println(minLux);
  Serial.print(F("maxLuxCinza (120%): ")); Serial.println(maxLux);
  Serial.print(F("minCCinza (80%): ")); Serial.println(minC);
  Serial.print(F("maxCCinza (120%): ")); Serial.println(maxC);

  EEPROM.put(EEPROM_minLuxCinza, minLux);
  EEPROM.put(EEPROM_maxLuxCinza, maxLux);
  EEPROM.put(EEPROM_minCNoCinza, minC);
  EEPROM.put(EEPROM_maxCNoCinza, maxC);

  minLuxCinza = minLux;
  maxLuxCinza = maxLux;
  minCNoCinza = minC;
  maxCNoCinza = maxC;

  Serial.println("Valores salvos na EEPROM!");
}

void imprimirValoresEEPROM() {
  int valor;

  Serial.println(F("=== Valores armazenados na EEPROM ==="));

  int valor1, valor2;

  EEPROM.get(EEPROM_MIN_LUX_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MIN_LUX_VERDE_ESQ, valor2);
  Serial.print(F("minLuxVerde: ")); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MAX_LUX_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MAX_LUX_VERDE_ESQ, valor2);
  Serial.print(F("maxLuxVerde: ")); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MIN_C_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MIN_C_VERDE_ESQ, valor2);
  Serial.print(F("minCVerde: ")); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MAX_C_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MAX_C_VERDE_ESQ, valor2);
  Serial.print(F("maxCVerde: ")); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_DIFERENCA_CORES_DIR, valor1);
  EEPROM.get(EEPROM_DIFERENCA_CORES_ESQ, valor2);
  Serial.print(F("diferencaDasCores (Verde): ")); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_C_BRANCO, valor);
  Serial.print(F("valorCnoBranco: ")); Serial.println(valor);

  EEPROM.get(EEPROM_maxLuxPreto, valor);
  Serial.print(F("maxLuxPreto: ")); Serial.println(valor);
  EEPROM.get(EEPROM_maxCPreto, valor);
  Serial.print(F("maxCPreto: ")); Serial.println(valor);

  EEPROM.get(EEPROM_minLuxCinza, valor);
  Serial.print(F("minLuxCinza: ")); Serial.println(valor);
  EEPROM.get(EEPROM_maxLuxCinza, valor);
  Serial.print(F("maxLuxCinza: ")); Serial.println(valor);
  EEPROM.get(EEPROM_minCNoCinza, valor);
  Serial.print(F("minCNoCinza: ")); Serial.println(valor);
  EEPROM.get(EEPROM_maxCNoCinza, valor);
  Serial.print(F("maxCNoCinza: ")); Serial.println(valor);

  EEPROM.get(EEPROM_minLuxVermelho, valor);
  Serial.print(F("minLuxVermelho: ")); Serial.println(valor);
  EEPROM.get(EEPROM_maxLuxVermelho, valor);
  Serial.print(F("maxLuxVermelho: ")); Serial.println(valor);
  EEPROM.get(EEPROM_minCVermelho, valor);
  Serial.print(F("minCVermelho: ")); Serial.println(valor);
  EEPROM.get(EEPROM_maxCVermelho, valor);
  Serial.print(F("maxCVermelho: ")); Serial.println(valor);
  EEPROM.get(EEPROM_diferencaDasCoresVermelho, valor);
  Serial.print(F("diferencaDasCoresVermelho: ")); Serial.println(valor);

  Serial.println(F("======================================"));
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
    bool verdeDireita = (
      g1 > r1 && g1 > b1 &&
      (g1 - r1) > diferencaDasCoresDir &&
      cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir &&
      luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir
    );
    bool verdeEsquerda = (
      g2 > r2 && g2 > b2 &&
      (g2 - r2) > diferencaDasCoresEsq &&
      cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq &&
      luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq
    );
    bool verdeAmbos = (abs(cc1 - cc2) < subtracaoSensoresCor) && verdeDireita && verdeEsquerda;
    Serial.print(F("Verde Direita: ")); Serial.println(verdeDireita);
    Serial.print(F("Verde Esquerda: ")); Serial.println(verdeEsquerda);
    Serial.print(F("Verde Ambos: ")); Serial.println(verdeAmbos);
    Serial.print(F("cc1: ")); Serial.print(cc1); Serial.print(F(" | cc2: ")); Serial.println(cc2);
    Serial.print(F("Lux1: ")); Serial.print(lux1); Serial.print(F(" | Lux2: ")); Serial.println(lux2);
    Serial.print(F("R1: ")); Serial.print(r1); Serial.print(F(" | R2: ")); Serial.println(r2);
    Serial.print(F("G1: ")); Serial.print(g1); Serial.print(F(" | G2: ")); Serial.println(g2);
    Serial.print(F("B1: ")); Serial.print(b1); Serial.print(F(" | B2: ")); Serial.println(b2);
    Serial.print(F("C1: ")); Serial.print(c1); Serial.print(F(" | C2: ")); Serial.println(c2);
    Serial.print(F("Lux1: ")); Serial.print(lux1); Serial.print(F(" | Lux2: ")); Serial.println(lux2);
    Serial.print(F("cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir: ")); Serial.println(cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir);
    Serial.print(F("cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq: ")); Serial.println(cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq);
    Serial.print(F("luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir: ")); Serial.println(luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir);
    Serial.print(F("luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq: ")); Serial.println(luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq);
    Serial.print(F("g1 > r1 && g1 > b1: ")); Serial.println(g1 > r1 && g1 > b1);
    Serial.print(F("g1 - r1 > diferencaDasCoresDir: ")); Serial.println(g1 - r1 > diferencaDasCoresDir);
    Serial.print(F("g2 > r2 && g2 > b2: ")); Serial.println(g2 > r2 && g2 > b2);
    Serial.print(F("g2 - r2 > diferencaDasCoresEsq: ")); Serial.println(g2 - r2 > diferencaDasCoresEsq);
    Serial.print(F("Abs(cc1 - cc2) < subtracaoSensoresCor: ")); Serial.println(abs(cc1 - cc2) < subtracaoSensoresCor);
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

    Serial.print(F("Vermelho Direita: ")); Serial.println(vermelhoDireita);
    Serial.print(F("Vermelho Esquerda: ")); Serial.println(vermelhoEsquerda);
    Serial.print(F("Vermelho Ambos: ")); Serial.println(vermelhoAmbos);
    Serial.print(F("cc1: ")); Serial.print(cc1); Serial.print(F(" | cc2: ")); Serial.println(cc2);
    Serial.print(F("Lux1: ")); Serial.print(lux1); Serial.print(F(" | Lux2: ")); Serial.println(lux2);
    Serial.print(F("R1: ")); Serial.print(r1); Serial.print(F(" | R2: ")); Serial.println(r2);
    Serial.print(F("G1: ")); Serial.print(g1); Serial.print(F(" | G2: ")); Serial.println(g2);
    Serial.print(F("B1: ")); Serial.print(b1); Serial.print(F(" | B2: ")); Serial.println(b2);
    Serial.print(F("C1: ")); Serial.print(c1); Serial.print(F(" | C2: ")); Serial.println(c2);

    Serial.print(F("cc1 >= minCVermelho && cc1 <= maxCVermelho: ")); Serial.println(cc1 >= minCVermelho && cc1 <= maxCVermelho);
    Serial.print(F("cc2 >= minCVermelho && cc2 <= maxCVermelho: ")); Serial.println(cc2 >= minCVermelho && cc2 <= maxCVermelho);
    Serial.print(F("luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho: ")); Serial.println(luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho);
    Serial.print(F("luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho: ")); Serial.println(luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho);
    Serial.print(F("r1 > g1 && r1 > b1: ")); Serial.println(r1 > g1 && r1 > b1);
    Serial.print(F("r1 - g1 > diferencaDasCoresVermelho: ")); Serial.println(r1 - g1 > diferencaDasCoresVermelho);
    Serial.print(F("r2 > g2 && r2 > b2: ")); Serial.println(r2 > g2 && r2 > b2);
    Serial.print(F("r2 - g2 > diferencaDasCoresVermelho: ")); Serial.println(r2 - g2 > diferencaDasCoresVermelho);
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

    Serial.print(F("Cinza Direita: ")); Serial.println(cinzaDireita);
    Serial.print(F("Cinza Esquerda: ")); Serial.println(cinzaEsquerda);
    Serial.print(F("Cinza Ambos: ")); Serial.println(cinzaAmbos);
    Serial.print(F("cc1: ")); Serial.print(cc1); Serial.print(F(" | cc2: ")); Serial.println(cc2);
    Serial.print(F("Lux1: ")); Serial.print(lux1); Serial.print(F(" | Lux2: ")); Serial.println(lux2);
    Serial.print(F("R1: ")); Serial.print(r1); Serial.print(F(" | R2: ")); Serial.println(r2);
    Serial.print(F("G1: ")); Serial.print(g1); Serial.print(F(" | G2: ")); Serial.println(g2);
    Serial.print(F("B1: ")); Serial.print(b1); Serial.print(F(" | B2: ")); Serial.println(b2);
    Serial.print(F("C1: ")); Serial.print(c1); Serial.print(F(" | C2: ")); Serial.println(c2);
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
          case 3: // maxLuxVerde
            Serial.println("1 - maxLuxVerdeDir | 2 - maxLuxVerdeEsq");
            while (!Serial.available()) { delay(10); }
            if (Serial.parseInt() == 1)
              alterarValorEEPROM("maxLuxVerdeDir", EEPROM_MAX_LUX_VERDE_DIR, maxLuxVerdeDir);
            else
              alterarValorEEPROM("maxLuxVerdeEsq", EEPROM_MAX_LUX_VERDE_ESQ, maxLuxVerdeEsq);
            break;
          case 4: // minLuxVerde
            Serial.println("1 - minLuxVerdeDir | 2 - minLuxVerdeEsq");
            while (!Serial.available()) { delay(10); }
            if (Serial.parseInt() == 1)
              alterarValorEEPROM("minLuxVerdeDir", EEPROM_MIN_LUX_VERDE_DIR, minLuxVerdeDir);
            else
              alterarValorEEPROM("minLuxVerdeEsq", EEPROM_MIN_LUX_VERDE_ESQ, minLuxVerdeEsq);
            break;
          case 5: // maxCVerde
            Serial.println("1 - maxCVerdeDir | 2 - maxCVerdeEsq");
            while (!Serial.available()) { delay(10); }
            if (Serial.parseInt() == 1)
              alterarValorEEPROM("maxCVerdeDir", EEPROM_MAX_C_VERDE_DIR, maxCVerdeDir);
            else
              alterarValorEEPROM("maxCVerdeEsq", EEPROM_MAX_C_VERDE_ESQ, maxCVerdeEsq);
            break;
          case 6: // minCVerde
            Serial.println("1 - minCVerdeDir | 2 - minCVerdeEsq");
            while (!Serial.available()) { delay(10); }
            if (Serial.parseInt() == 1)
              alterarValorEEPROM("minCVerdeDir", EEPROM_MIN_C_VERDE_DIR, minCVerdeDir);
            else
              alterarValorEEPROM("minCVerdeEsq", EEPROM_MIN_C_VERDE_ESQ, minCVerdeEsq);
            break;
          case 7: imprimirValoresEEPROM(); break;
          case 8: // diferencaDasCores
            Serial.println("1 - diferencaDasCoresDir | 2 - diferencaDasCoresEsq");
            while (!Serial.available()) { delay(10); }
            if (Serial.parseInt() == 1)
              alterarValorEEPROM("diferencaDasCoresDir", EEPROM_DIFERENCA_CORES_DIR, diferencaDasCoresDir);
            else
              alterarValorEEPROM("diferencaDasCoresEsq", EEPROM_DIFERENCA_CORES_ESQ, diferencaDasCoresEsq);
            break;
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

  Wire.begin();
  // Wire.setClock(400000);

  byte status = giro.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){
  }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  giro.calcOffsets();
  Serial.println("Done!\n");

  for (int i = 0; i < 5; i++) {
    pinMode(sensores[i], INPUT);
  }

  pinMode(pinoR1, OUTPUT); 
  pinMode(pinoG1, OUTPUT); 
  pinMode(pinoB1, OUTPUT);
  pinMode(pinoR2, OUTPUT); 
  pinMode(pinoG2, OUTPUT); 
  pinMode(pinoB2, OUTPUT);
  
  pinMode(Hall_Esquerda, INPUT);
  pinMode(Hall_Direita, INPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(no, INPUT_PULLUP);

  motorE.attach(motorEpin);
  motorD.attach(motorDpin);
  
  motorE.write(90);
  motorD.write(90);

  anguloDoReto = retornoAnguloY();
  anguloReto = retornoAnguloZ();

  anguloRampaSubida = anguloDoReto - 10;
  anguloRampaDescida = anguloDoReto + 10;

  retornoSensoresCor();

  EEPROM.get(EEPROM_MIN_LUX_VERDE_DIR, minLuxVerdeDir);
  EEPROM.get(EEPROM_MAX_LUX_VERDE_DIR, maxLuxVerdeDir);
  EEPROM.get(EEPROM_MIN_C_VERDE_DIR, minCVerdeDir);
  EEPROM.get(EEPROM_MAX_C_VERDE_DIR, maxCVerdeDir);
  EEPROM.get(EEPROM_DIFERENCA_CORES_DIR, diferencaDasCoresDir);

  EEPROM.get(EEPROM_MIN_LUX_VERDE_ESQ, minLuxVerdeEsq);
  EEPROM.get(EEPROM_MAX_LUX_VERDE_ESQ, maxLuxVerdeEsq);
  EEPROM.get(EEPROM_MIN_C_VERDE_ESQ, minCVerdeEsq);
  EEPROM.get(EEPROM_MAX_C_VERDE_ESQ, maxCVerdeEsq);
  EEPROM.get(EEPROM_DIFERENCA_CORES_ESQ, diferencaDasCoresEsq);

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

  setCorDireita(1,1,1);
  setCorEsquerda(1,1,1);

  ligarGarra();
  subirGarra();
  fecharGarra();
  desligarMotoresGarra();

  //tocar_buzzer(1000, 3, 125);
}

//******************************************************************************
//*                                                                            *
//*                                Void Loop                                   *
//*                                                                            *
//*******************************************************************************

void loop() {
  processarComandoSerial();
  if (!modoConfig) {
    andarReto();
  }
}


