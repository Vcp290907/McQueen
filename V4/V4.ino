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

int veloBaseEsq = 140; //140
int veloBaseDir = 40; //40
int pequenaCurvaLadoC = 15;//15
int pequenaCurvaLadoR = 5; //5
int veloCurva90 = 40; //40

int grausCurva90 = 90;
int graqusCurva180 = 180;
int grausCurva45 = 45;

int anguloAtual = 0;

int verificacaoCurvaVerde = 350; //Pulinho para ver se é curva verde
int erroGiro = 0;
int erroRampa = 3; // 3
int erroRampaDescida = 8; // 5
int tempoDepoisDoVerde90 = 2000;
int delayCurvasverde = 500;
int tempoAntesCurva90 = 500;
int tempoDepoisCurva90 = 1000; //1000
int tempoDepoisDoVerde180 = 1000; //1000
int tempoDepoisDoVerdeFalso = 750; //1000
int paredeResgate = 2000; //2000
int paredeResgateSaida = 1000; //1000

//Branco
int valorCnoBranco = 1000; 

// Verde mais claro
int minLuxVerdeDir, minLuxVerdeEsq;
int maxLuxVerdeDir, maxLuxVerdeEsq;
int minCVerdeDir, minCVerdeEsq;
int maxCVerdeDir, maxCVerdeEsq;
int diferencaDasCoresDir, diferencaDasCoresEsq;

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
int subtracaoSensoresCor = 100;

//PID

float Kp = 2.0;
float Ki = 0.2;
float Kd = 1.5;
int erroP = 0;
int erroAnterior = 0;
float erroI = 0;
float erroD = 0;
int ajuste = 0;

//Desvio OBJETO
int distanciaDesvio = 0; //6
int delayCurva1 = 3;
int delayCurva2 = 7;
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
    Serial.print("Distancia medida: ");Serial.println(medida);
    delay(10);
  }
  Serial.print("Media Sensor "); Serial.print(sensor); Serial.print(": "); Serial.println(soma / numLeituras);
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

  Serial.println("=== [DEBUG] verificaVermelho ===");
  Serial.print("Sensor Direita - R: "); Serial.print(r1);
  Serial.print(" G: "); Serial.print(g1);
  Serial.print(" B: "); Serial.print(b1);
  Serial.print(" C: "); Serial.print(cc1);
  Serial.print(" Lux: "); Serial.println(luu1);

  Serial.print("Sensor Esquerda - R: "); Serial.print(r2);
  Serial.print(" G: "); Serial.print(g2);
  Serial.print(" B: "); Serial.print(b2);
  Serial.print(" C: "); Serial.print(cc2);
  Serial.print(" Lux: "); Serial.println(luu2);

  Serial.print("Limites: minCVermelho="); Serial.print(minCVermelho);
  Serial.print(" maxCVermelho="); Serial.print(maxCVermelho);
  Serial.print(" minLuxVermelho="); Serial.print(minLuxVermelho);
  Serial.print(" maxLuxVermelho="); Serial.print(maxLuxVermelho);
  Serial.print(" diferencaDasCoresVermelho="); Serial.println(diferencaDasCoresVermelho);

  bool vermelhoDireita = (r1 > g1 && r1 > b1 && r1 - g1 > diferencaDasCoresVermelho && 
                          cc1 >= minCVermelho && cc1 <= maxCVermelho && luu1 >= minLuxVermelho && luu1 <= maxLuxVermelho);
  bool vermelhoEsquerda = (r2 > g2 && r2 > b2 && r2 - g2 > diferencaDasCoresVermelho && 
                          cc2 >= minCVermelho && cc2 <= maxCVermelho && luu2 >= minLuxVermelho && luu2 <= maxLuxVermelho);

  Serial.print("vermelhoDireita: "); Serial.println(vermelhoDireita);
  Serial.print("vermelhoEsquerda: "); Serial.println(vermelhoEsquerda);
  Serial.print("r1 - g1: "); Serial.println(r1 - g1);
  Serial.print("r2 - g2: "); Serial.println(r2 - g2);

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

bool testarAmplitudeSensor(SharpIR &sensor, int amplitudeMaxima = 9, int numLeituras = 20) {
  int amp = amplitudeSensor(sensor, numLeituras);
  Serial.print("Amplitude do sensor "); Serial.println(amp);
  bool vazio = (amp > amplitudeMaxima);
  if (vazio) Serial.println("Sensor no vazio!");
  Serial.println("---------------------");
  return !vazio;
}

void sairDireita(){
  tocar_buzzer(1000, 1, 500);
  Serial.println("Saida!");
  Serial.println("Saindo a direita");

  motorE.write(veloBaseDir);
  motorD.write(veloBaseDir);
  while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
    giro.update();
  }
  anguloReto = anguloReto - grausCurva90;

  sl = lerSensoresLinha();
  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1) {
    sl = lerSensoresLinha();
    correcaoObjeto();
  }
}

void sairEsquerda(){
  tocar_buzzer(1000, 1, 500);
  Serial.println("Saida!");

  motorE.write(veloBaseEsq);
  motorD.write(veloBaseEsq);
  Serial.println("Fazendo curva para a esquerda da saida");
  while(((anguloReto + grausCurva90) > retornoAnguloZ())) {
    giro.update();
    sl = lerSensoresLinha();
  }
  anguloReto = anguloReto + grausCurva90;

  sl = lerSensoresLinha();
  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1) {
    sl = lerSensoresLinha();
    correcaoObjeto();
  }
}

void sairFrente(){
  tocar_buzzer(1000, 2, 250);
  Serial.println("Saindo pra frente");
  sl = lerSensoresLinha();

  while(sl[0] == 1 && sl[1] == 1 && sl[2] == 1 && sl[3] == 1 && sl[4] == 1){
    sl = lerSensoresLinha();
    correcaoObjeto();
  }
  return;
}

void andarRetoPorTempo(int tempo){
  Serial.print("Andando reto por "); Serial.print(tempo); Serial.println(" ms");
  unsigned long startTime = millis();
  while (millis() - startTime < tempo) {
    correcaoObjeto();
  }
  motorE.write(90);
  motorD.write(90);
}

void andarPraTrasPorTempo(int tempo){
  Serial.print("Andando reto por "); Serial.print(tempo); Serial.println(" ms");
  unsigned long startTime = millis();
  while (millis() - startTime < tempo) {
    correcaoRe();
  }
  motorE.write(90);
  motorD.write(90);
}

int distanciaMaxima = 20; 

void resgate(){
  Serial.println("Resgate iniciado!");
  tocar_buzzer(500, 3, 200);
  motorE.write(90);
  motorD.write(90);

  if(retornoAnguloY() > anguloDoReto + erroRampaDescida || retornoAnguloY() < anguloDoReto - erroRampaDescida) {
    Serial.println("Não é resgate 1!");
    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    tocar_buzzer(500, 2, 100);
    return;
  }else if (mediaInfravermelho(1) >= distanciaMaxima && mediaInfravermelho(3) >= distanciaMaxima) {
    Serial.println("Não é resgate 2!");
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseDir);
    tocar_buzzer(500, 2, 100);
    return;
  }
  
  tocar_buzzer(500, 2, 200);

  Serial.print("Angulo Reto: "); Serial.print(anguloReto); Serial.print(" | Angulo Atual: "); Serial.println(retornoAnguloZ());

  unsigned long startTime = millis();
  while(startTime + 2 * 1000 > millis()) {
    correcaoObjeto();
    giro.update();
  }

  motorE.write(90);
  motorD.write(90);

  //==================================================
  //
  // Verificando se tem saida antes da primiera curva
  //
  //==================================================

  int valorDireita;
  int valorEsquerda;

  bool vEsquerda;
  bool vDireita;

  valorEsquerda = mediaInfravermelho(1);
  valorDireita = mediaInfravermelho(3);

  Serial.print("Valor Sensor Direita: "); Serial.println(valorDireita);
  Serial.print("Valor Sensor Esquerda: "); Serial.println(valorEsquerda);
  Serial.print("distanciaMaxima: "); Serial.println(distanciaMaxima);

  bool direita = valorDireita <= distanciaMaxima;
  bool esquerda = valorEsquerda <= distanciaMaxima;

  //==================================================
  //
  //                  Primeira curva
  //
  //==================================================

  ligarGarra();
  garraMeio();
  desligarMotoresGarra();
  delay(500);
  bool saidaNoMeioFrente, saidaNoMeioDireita, saidaNoMeioEsquerda;
  //*************************************** */
  //
  //                PARTE 01
  //          alinhando na parede   
  //
  //*************************************** */

  if (esquerda) { // ESQUERDA
    Serial.println("Alinhar a esquerda!");

    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto - grausCurva90;

    motorD.write(veloBaseEsq);
    motorE.write(veloBaseDir);
    delay(paredeResgate);
    anguloReto = retornoAnguloZ();

    Serial.print("Angulo Reto: "); Serial.print(anguloReto);

    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    delay(paredeResgateSaida);
    
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto + grausCurva90;

  } else if (direita) { // DIREITA
    Serial.println("Alinhar a direita!!");

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto + grausCurva90;

    motorD.write(veloBaseEsq);
    motorE.write(veloBaseDir);
    delay(paredeResgate);
    anguloReto = retornoAnguloZ();

    Serial.print("Angulo Reto: "); Serial.print(anguloReto);

    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    delay(paredeResgateSaida);

    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto - grausCurva90;

  }

  //*********************************************** */
  //
  //                PARTE 02
  //          alinhando na parede do meio  
  //
  //*********************************************** */

  Serial.println("Alinhado!");

  startTime = millis();
  while(startTime + 5 * 1000 > millis()) {
    correcaoObjeto();
  }

  motorD.write(90);
  motorE.write(90);

  //Verificando a saida, no meio
  bool saidaMeioInicial = false;

  if(esquerda){
    int d1 = mediaInfravermelho(1);
    if(d1 > 30){
      tocar_buzzer(1000, 3, 500); 
      saidaMeioInicial = true;
      Serial.println("Tem saida no meio inicio!");
    }
    Serial.println("Sem saida!");

  }else if(direita){
    int d2 = mediaInfravermelho(3);
    if(d2 > 30){
      tocar_buzzer(1000, 3, 500); 
      saidaMeioInicial = true;
      Serial.println("Tem saida no meio inicio!");
    }
    Serial.println("Sem saida!");
  }

  motorD.write(90);
  motorE.write(90);

  //*********************************************** */
  //
  //                PARTE 03
  // Estando no meio, proximo a lateral da entrada
  //       vai ver o que tem na frente   
  //
  //*********************************************** */

  bool irMaisPraFrente = false;
  if(mediaInfravermelho(2) > 30) {
    irMaisPraFrente = true;
    Serial.println("Indo pra frente ver saida!");
    startTime = millis();
    while(startTime + 4 * 1000 > millis()) {
      correcaoObjeto();
    }

    if(esquerda){
      valorEsquerda = mediaInfravermelho(1);
      if(valorEsquerda > 35){
        Serial.println("Tem saida na esquerda!");
        sairEsquerda();
        return;
      
    }else if(direita){

      valorDireita = mediaInfravermelho(3);
      if(valorDireita > 35){
        Serial.println("Tem saida na direita!");
        sairDireita();
        return;
      }

    }
  }else if(mediaInfravermelho(2) > 30) {
    Serial.println("SAIDAAAAAA");
    tocar_buzzer(1000, 1, 500);
    sairFrente();
    return;
  }


  //*********************************************** */
  //
  //                PARTE 03.1
  //        Se o carrinho foi mais pra frente
  //       ele vai dar ré e voltar para o meio
  //         e ir pra frente normalmente
  //
  //*********************************************** */


  if(irMaisPraFrente){
    Serial.println("bolinho de arroz");

    while(digitalRead(no) == HIGH){
      correcaoObjeto();
    }

    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    tocar_buzzer(1000, 2, 150);
    anguloReto = retornoAnguloZ();

    startTime = millis();
    while(startTime + 7 * 1000 > millis()) {
      correcaoRe();
    }
  }

  //*********************************************** */
  //
  //                PARTE 04
  //     Estando no meio, vai virar para
  //          ir com a garra aberta
  //             pegar bolinhas
  //
  //*********************************************** */

  if(esquerda){
    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - grausCurva90) < retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto - grausCurva90;

    motorE.write(veloBaseDir);
    motorD.write(veloBaseEsq);
    delay(paredeResgate);
    anguloReto = retornoAnguloZ();
    Serial.print("Angulo Reto: "); Serial.print(anguloReto);
    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);

  }else if(direita){
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + grausCurva90) > retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto + grausCurva90;

    motorE.write(veloBaseDir);
    motorD.write(veloBaseEsq);
    delay(paredeResgate);
    anguloReto = retornoAnguloZ();
    Serial.print("Angulo Reto: "); Serial.print(anguloReto);
    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
  }

  //*********************************************** */
  //
  //                PARTE 04.1
  //    Agora virado pra frente, vai abrir a garra
  //        descer ela, e alinhar na parede
  //
  //*********************************************** */

  ligarGarra();
  abrirGarra();
  descerGarra();
  delay(250);

  motorD.write(veloBaseEsq);
  motorE.write(veloBaseDir);
  delay(500);
  anguloReto = retornoAnguloZ();

  //*********************************************** */
  //
  //                PARTE 05
  //      Agora vai ir pro meio com a garra aberta
  //
  //*********************************************** */

  startTime = millis();
  while(startTime + 4 * 1000 > millis()) {
    correcaoObjeto();
    giro.update();
  }

  //*********************************************** */
  //
  //                PARTE 05.1
  //      Verificar se tem saida na lateral, 
  //              estando no meio
  //
  //*********************************************** */

  saidaNoMeioDireita = false;
  saidaNoMeioEsquerda = false;

  if(esquerda){ //Não pode ter saida no lado direito nesse ponto, considerando entrada no canto
    int dEsquerda = mediaInfravermelho(1);
    if(dEsquerda > 45){
      saidaNoMeioEsquerda = true;
      Serial.println("Tem saida no meio a esquerda!");
      tocar_buzzer(1000, 1, 500);
    }
  }else if(direita){ //Não pode ter saida no lado esquerdo nesse ponto, considerando entrada no canto
    int dDireita = mediaInfravermelho(3);
    if(dDireita > 45){
      saidaNoMeioDireita = true;
      Serial.println("Tem saida no meio a direita!");
      tocar_buzzer(1000, 2, 250);
    }
  }
  //*********************************************** */
  //
  //                PARTE 05.2
  //      Verificar se tem saida na frente
  // estando no meio, se não tiver, vai pra frente
  //    por 5 segundos, pra ficar na parede
  //
  //*********************************************** */

  saidaNoMeioFrente = false;
  if(mediaInfravermelho(2) > 50) {
    Serial.println("Tem saida no meio!");
    saidaNoMeioFrente = true;
    tocar_buzzer(1000, 1, 1500);
  } else {
    Serial.println("Não tem saida no meio!");
    
    andarRetoPorTempo(5000);

    anguloReto = retornoAnguloZ();
    fecharGarra();
    startTime = millis();
    while(startTime + 1 * 100 > millis()) {
      correcaoRe();
    }
    garraMeio();
    }

    while(digitalRead(no) == HIGH){
      correcaoObjeto();
    }
    motorD.write(90);
    motorE.write(90);
  }

  //*********************************************** */
  //
  //                PARTE 06
  //     Alinhado na parede do meio da frente,
  // vai verificar em qual lugar deixar as bolinhas
  //        e voltar ao meio para sair
  //
  //*********************************************** */

  bool deixarBolinha = true; //False para esquerda, True para direita

  andarPraTrasPorTempo(3500);

  if(deixarBolinha){
    Serial.println("Deixar bolinha a direita!");
    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - 45) < retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto - 45;
    andarRetoPorTempo(3000);
    garra90();
    abrirGarra();
    delay(500);
    subirGarra();
    fecharGarra();
    desligarGarra();
    delay(500);
    andarPraTrasPorTempo(3500);

    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + 45) > retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto + 45;

  }else if(!deixarBolinha){
    Serial.println("Deixar bolinha a esquerda!");
    motorE.write(veloBaseEsq);
    motorD.write(veloBaseEsq);
    while (((anguloReto + 45) > retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto + 45;
    andarRetoPorTempo(3000);
    garra90();
    abrirGarra();
    delay(500);
    subirGarra();
    fecharGarra();
    desligarGarra();
    delay(500);
    andarPraTrasPorTempo(3500);

    motorE.write(veloBaseDir);
    motorD.write(veloBaseDir);
    while (((anguloReto - 45) < retornoAnguloZ())) {
      giro.update();
    }
    anguloReto = anguloReto - 45;
  }

  andarPraTrasPorTempo(3000);

  if(saidaNoMeioDireita){
    Serial.println("Saindo a direita do resgate!");
    sairDireita();

  }else if(saidaNoMeioEsquerda){
    Serial.println("Saindo a esquerda do resgate!");
    sairEsquerda();
  }

  Serial.println("=== [DEBUG] Fim do resgate ===");
  Serial.println("Saiadas:"); 
  Serial.print("saidaNoMeioFrente: "); Serial.println(saidaNoMeioFrente);
  Serial.print("saidaNoMeioEsquerda: "); Serial.println(saidaNoMeioEsquerda);
  Serial.print("saidaNoMeioDireita: "); Serial.println(saidaNoMeioDireita);

  while (true){
    motorD.write(90);
    motorE.write(90);
  }
}

void lerCinza() { //
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  lux1 = tcs1.calculateLux(r1, g1, b1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  lux2 = tcs2.calculateLux(r2, g2, b2);
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

  Serial.print("Direita - R: "); Serial.print(r1);
  Serial.print(" G: "); Serial.print(g1);
  Serial.print(" B: "); Serial.print(b1);
  Serial.print(" C: "); Serial.print(cc1);
  Serial.print(" Lux: "); Serial.println(luu1);

  Serial.print("Esquerda - R: "); Serial.print(r2);
  Serial.print(" G: "); Serial.print(g2);
  Serial.print(" B: "); Serial.print(b2);
  Serial.print(" C: "); Serial.print(cc2);
  Serial.print(" Lux: "); Serial.println(luu2);

  Serial.print("Limites Direita: minC="); Serial.print(minCVerdeDir);
  Serial.print(" maxC="); Serial.print(maxCVerdeDir);
  Serial.print(" minLux="); Serial.print(minLuxVerdeDir);
  Serial.print(" maxLux="); Serial.print(maxLuxVerdeDir);
  Serial.print(" difCor="); Serial.println(diferencaDasCoresDir);

  Serial.print("Limites Esquerda: minC="); Serial.print(minCVerdeEsq);
  Serial.print(" maxC="); Serial.print(maxCVerdeEsq);
  Serial.print(" minLux="); Serial.print(minLuxVerdeEsq);
  Serial.print(" maxLux="); Serial.print(maxLuxVerdeEsq);
  Serial.print(" difCor="); Serial.println(diferencaDasCoresEsq);

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

void giroVerde() {
  Serial.println("Giro Verde");
  int resultado1 = verificaVerdeNovamente(0);
  Serial.print("[ANTES] Resultado verificaVerdeNovamente: ");
  Serial.println(resultado1);

  tocar_buzzer(1000, 1, 100);
  giro.update();
 
  if (resultado1 == 3) {
    Serial.println("Verde nos dois sensores!");
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
  } else if (resultado1 == 1) {
    Serial.println("Verde só na direita!");
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
  } else if (resultado1 == 2) {
    Serial.println("Verde só na esquerda!");
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
  } else {
    Serial.println("Nenhum verde detectado.");
    motorD.write(veloBaseDir);
    motorE.write(veloBaseEsq);
    delay(tempoDepoisDoVerdeFalso);
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
      motorE.write(130);
      motorD.write(80);
      Serial.println("Correção1");
    }
    else if (anguloReto + erro < anguloAtual) {
      verificaVermelho();
      motorE.write(100);
      motorD.write(50);
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
    motorE.write(130);
    motorD.write(80);
    // Serial.println("Objeto Correção1");
  }
  else if (anguloReto + erro < anguloAtual) {
    motorE.write(100);
    motorD.write(50);
    // Serial.println("Objeto Correção2");
  }
  else if (abs(anguloReto - anguloAtual) <= erro) {
    motorE.write(veloBaseEsq + veloCurva90);
    motorD.write(veloBaseDir - veloCurva90);
    // Serial.println("Objeto Correção3");
  }
}

void correcaoRe(){
  anguloAtual = retornoAnguloZ();
  if (anguloReto - erro > anguloAtual) {
    motorE.write(80);
    motorD.write(130);
    //Serial.println("Objeto Correção1");
  }
  else if (anguloReto + erro < anguloAtual) {
    motorE.write(50);
    motorD.write(100);
    //Serial.println("Objeto Correção2");
  }
  else if (abs(anguloReto - anguloAtual) <= erro) {
    motorE.write(veloBaseDir - veloCurva90);
    motorD.write(veloBaseEsq + veloCurva90);
    //Serial.println("Objeto Correção3");
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

static bool estavaDesalinhado = true;

void andarReto() {
  giro.update();
  sl = lerSensoresLinha();

  int combinacaoSensores = sl[0] * 16 + sl[1] * 8 + sl[2] * 4 + sl[3] * 2 + sl[4];

  if(retornoAnguloY() > (anguloDoReto + erroRampaDescida)) {
    while (retornoAnguloY() > (anguloDoReto + erroRampaDescida))
    {
      Serial.println("Descida detectada!");
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
      ajuste = constrain(ajuste, 0, 40);

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

      Serial.print("Andando reto | Angulo Atual: "); Serial.print(anguloAtual);
      Serial.print(" | Angulo Reto: "); Serial.print(anguloReto);
      Serial.print(" | Ajuste: "); Serial.print(ajuste);
      Serial.print(" | P: "); Serial.print(Kp * erroP);
      Serial.print(" | I: "); Serial.print(Ki * erroI);
      Serial.print(" | D: "); Serial.println(Kd * erroD);

      if (estavaDesalinhado) {
        anguloReto = (anguloAtual + anguloReto*2) / 3;
        erroI = 0;
        Serial.print("Novo angulo RETO (centralizado): "); Serial.println(anguloReto);
        estavaDesalinhado = false;
      }
      
      desvioObjeto();

      break;

    case 0b10011: // Pequena curva esquerda
      motorE.write(veloBaseEsq + pequenaCurvaLadoC);
      motorD.write(veloBaseDir);
      Serial.println("Pequena curva esquerda");
      estavaDesalinhado = true;
      //desvioObjeto();
      break;

    case 0b00011: // Curva falsa ou verde
      Serial.println("Curva falsa OU verde");
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
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
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir - pequenaCurvaLadoC);
      estavaDesalinhado = true;
      //desvioObjeto();
      break;

    case 0b11000: // Curva falsa ou verde
      Serial.println("Curva falsa OU verde");
      giroVerde();
      motorE.write(veloBaseEsq);
      motorD.write(veloBaseDir);
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

    case 0b00000:
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
      estavaDesalinhado = true;
      break;

    case 0b11101: // Saiu do principal, direita
      Serial.println("Saiu do principal, direita");
      motorE.write(100);
      motorD.write(veloBaseDir);
      sl = lerSensoresLinha();
      desvioObjeto();
      estavaDesalinhado = true;
      break;

    case 0b11111: // Branco, final ou resgate
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
  motorG.write(55); // Posição de meio
  delay(2000); // Tempo para meio
}

void garra90(){
  Serial.println("Garra 90°");
  motorG.write(85);
}

void subirGarra() {
  Serial.println("Subindo Garra");
  motorG.write(0);
  delay(1000);
}

void subirGarraRapido() {
  Serial.println("Subindo Garra Rápido");
  motorG.write(0);
  delay(1000);
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
  Serial.println("=== Calibração do Verde (Média Individual) ===");
  Serial.println("Coloque o sensor DIREITO sobre o VERDE e envie qualquer tecla para iniciar...");
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
  for (int i = 0; i < amostras; i++) {
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

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxVerdeDir: "); Serial.println(mediaLuxDir);
  Serial.print("mediaCVerdeDir: "); Serial.println(mediaCDir);
  Serial.print("mediaDifVerdeDir (G-R): "); Serial.println(mediaDifDir);
  Serial.print("mediaLuxVerdeEsq: "); Serial.println(mediaLuxEsq);
  Serial.print("mediaCVerdeEsq: "); Serial.println(mediaCEsq);
  Serial.print("mediaDifVerdeEsq (G-R): "); Serial.println(mediaDifEsq);

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
  Serial.println("=== Calibração do Vermelho (Média) ===");
  Serial.println("Coloque o sensor DIREITO sobre o VERMELHO e envie qualquer tecla para iniciar...");
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

    Serial.print("[Direita] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux1: "); Serial.print(luxVermelho1);
    Serial.print(" | C1: "); Serial.print(cVermelho1);
    Serial.print(" | R1-G1: "); Serial.print(difVermelho1);
    Serial.print(" || Lux2: "); Serial.print(luxVermelho2);
    Serial.print(" | C2: "); Serial.print(cVermelho2);
    Serial.print(" | R2-G2: "); Serial.println(difVermelho2);

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

    Serial.print("[Esquerda] Amostra "); Serial.print(i+1);
    Serial.print(" | Lux1: "); Serial.print(luxVermelho1);
    Serial.print(" | C1: "); Serial.print(cVermelho1);
    Serial.print(" | R1-G1: "); Serial.print(difVermelho1);
    Serial.print(" || Lux2: "); Serial.print(luxVermelho2);
    Serial.print(" | C2: "); Serial.print(cVermelho2);
    Serial.print(" | R2-G2: "); Serial.println(difVermelho2);

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

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxVermelho: "); Serial.println(mediaLux);
  Serial.print("mediaCVermelho: "); Serial.println(mediaC);
  Serial.print("mediaDifVermelho (R-G): "); Serial.println(mediaDif);
  Serial.print("diferencaDasCoresVermelho usada: "); Serial.println(diferencaAjustada);
  Serial.print("minLuxVermelho (60%): "); Serial.println(minLux);
  Serial.print("maxLuxVermelho (140%): "); Serial.println(maxLux);
  Serial.print("minCVermelho (60%): "); Serial.println(minC);
  Serial.print("maxCVermelho (140%): "); Serial.println(maxC);

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

  Serial.println("Calibração concluída!");
  Serial.print("mediaLuxCinza: "); Serial.println(mediaLux);
  Serial.print("mediaCCinza: "); Serial.println(mediaC);
  Serial.print("minLuxCinza (80%): "); Serial.println(minLux);
  Serial.print("maxLuxCinza (120%): "); Serial.println(maxLux);
  Serial.print("minCCinza (80%): "); Serial.println(minC);
  Serial.print("maxCCinza (120%): "); Serial.println(maxC);

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

  Serial.println("=== Valores armazenados na EEPROM ===");

  int valor1, valor2;

  EEPROM.get(EEPROM_MIN_LUX_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MIN_LUX_VERDE_ESQ, valor2);
  Serial.print("minLuxVerde: "); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MAX_LUX_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MAX_LUX_VERDE_ESQ, valor2);
  Serial.print("maxLuxVerde: "); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MIN_C_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MIN_C_VERDE_ESQ, valor2);
  Serial.print("minCVerde: "); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_MAX_C_VERDE_DIR, valor1);
  EEPROM.get(EEPROM_MAX_C_VERDE_ESQ, valor2);
  Serial.print("maxCVerde: "); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

  EEPROM.get(EEPROM_DIFERENCA_CORES_DIR, valor1);
  EEPROM.get(EEPROM_DIFERENCA_CORES_ESQ, valor2);
  Serial.print("diferencaDasCores (Verde): "); Serial.print(valor1); Serial.print(" / "); Serial.println(valor2);

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
    Serial.print("cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir: "); Serial.println(cc1 >= minCVerdeDir && cc1 <= maxCVerdeDir);
    Serial.print("cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq: "); Serial.println(cc2 >= minCVerdeEsq && cc2 <= maxCVerdeEsq);
    Serial.print("luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir: "); Serial.println(luu1 >= minLuxVerdeDir && luu1 <= maxLuxVerdeDir);
    Serial.print("luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq: "); Serial.println(luu2 >= minLuxVerdeEsq && luu2 <= maxLuxVerdeEsq);
    Serial.print("g1 > r1 && g1 > b1: "); Serial.println(g1 > r1 && g1 > b1);
    Serial.print("g1 - r1 > diferencaDasCoresDir: "); Serial.println(g1 - r1 > diferencaDasCoresDir);
    Serial.print("g2 > r2 && g2 > b2: "); Serial.println(g2 > r2 && g2 > b2);
    Serial.print("g2 - r2 > diferencaDasCoresEsq: "); Serial.println(g2 - r2 > diferencaDasCoresEsq);
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
  Wire.setClock(400000);

  byte status = giro.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  giro.calcOffsets();
  Serial.println("Done!\n");

  for (int i = 0; i < 5; i++) {
    pinMode(sensores[i], INPUT);
  }

  pinMode(buzzer, OUTPUT);
  pinMode(no, INPUT_PULLUP);

  motorE.attach(motorEpin);
  motorD.attach(motorDpin);
  
  motorE.write(90);
  motorD.write(90);

  anguloDoReto = retornoAnguloY();
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

  // tocar_buzzer(750, 1, 125);

  ligarGarra();
  subirGarra();
  desligarGarra(); 

  tocar_buzzer(1000, 2, 125);
}

//******************************************************************************
//*                                                                            *
//*                                Void Loop                                   *
//*                                                                            *
//*******************************************************************************

void loop() {
  //verificaVermelho();
  //retornoSensoresCor();
  //lerInfravermelho();
  // delay(1000);
  //andarReto(

  processarComandoSerial();
  if (!modoConfig) {
    andarReto();
  }
}