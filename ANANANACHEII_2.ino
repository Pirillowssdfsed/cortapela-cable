// NodemCu ESP-32s  v1.1
// Incluir las librerías necesarias
#include <Config.h>
#include "BluetoothSerial.h"
#include <TickTwo.h>
#include "Wire.h"
#include <AS5600.h>

#define step3 13
#define dir3 12
#define step2 27
#define dir2 26
#define step1 25
#define dir1 33
#define led_rojo 15
#define led_amarillo 17
#define led_verde 5
#define buzzer 18
#define pulsador 14
#define infrarrojo1 22
#define infrarrojo2 21
#define ENCODER_DIR 0
#define ENCODER_SCL 4
#define ENCODER_SDA 23
#define en_mot1 16
#define en_mot2 19
#define en_mot3 32
#define notConected_1 1
#define notConected_2 2
#define notConected_34 34
#define notConected_35 35
#define notConected_36 36
#define notConected_39 39

AS5600 encoder;
volatile int counting_to_cut = 0;
bool estado_de_activacion_para_cortar = 0;
const float distance_to_cortaCable_mm = 19.31;
float distance_to_cortaCable_pasos = 0;
int valor_final = -1;
bool deteccion_de_digitos = 0;
int valor_temporal = -1;
const int delay_for_enabled_motors = 2;
const int delay_for_motors = 5;
volatile int cables_cortados = 0;
volatile int cables_a_cortar = 0;
volatile bool pelado = 0;
volatile bool posicion_del_cable = 0;
bool EMERGENCIA = 0;
bool direccion_movimiento_pelado = 0;
const int pasos_cortar_cable = 800;
volatile int pasos_reales = 0;
volatile int contador = 0;
int16_t ultima_posicion = 0;
volatile int pasos_dados_pelar_cable = 0;
const float distancia_maxima_seccion_pelado = 38.9;
const int pasos_maximos_x_vuelta = 200;
const int delay_bt_beep = 5000;
const int delay_beep = 150;
volatile int xy = 0;
volatile bool variable_estado_espera_terminal = 0;
volatile int ms_terminal = 0;
volatile bool delay_easybuzzer_estado = 0;
volatile int delay_easybuzzer = 0;
volatile bool condicion_pelado_derecho = 0;
volatile bool condicion_pelado_izquierdo = 0;
volatile bool condicion_de_movmiento = 0;
volatile int pasos_dados_mover_cable_gral = 0;
volatile int couting_to_cut = 0;
int seccion_cable = 0;
float pelado_izquierdo_mm = 0;
float pelado_derecho_mm = 0;
float largo_cable_mm = 0;
float diametro_cable_mm = 0;
int diametro_cable_pasos = 0;
int pelado_izquierdo_pasos = 0;
int pelado_derecho_pasos = 0;
int largo_cable_pasos = 0;
float diametro_holder = 16.3;
const float ang_paso = 0.01;
volatile bool stepaso = 0;
volatile bool leds = 0;
volatile bool infra1;
volatile bool infra2 ;
volatile bool pulso;
volatile int ms = 0;
volatile bool del_motor_pap = 0;
volatile bool del_leds = 0;
volatile bool condicion_de_avance = 0;
const int pasos_post_infra1_negado = 1;
volatile bool dir_mot1 = 1;
volatile bool dir_mot2 = 1;
volatile bool dir_mot3 = 1;
volatile bool del_pulso = 0;
volatile int del_pulso_ms = 0;
volatile bool del_led_amarillo = 0;
typedef enum {parada, menu, funcionamiento, fin} estados;
estados estado_actual = parada;
typedef enum {esperando_largo_cable, esperando_pelado_derecho, esperando_pelado_izquierdo, esperando_diametro, esperando_cantidad_de_cables, todos_los_datos_obtenidos} etapas_obtencion_datos;
etapas_obtencion_datos etapa_de_datos_actual = esperando_largo_cable;
typedef enum {etapa1, etapa2, etapa3} etapa;
etapa etapa_actual = etapa1;
volatile bool Verificacion_de_datos = 0;
BluetoothSerial terminal_esp32;
String comando_terminal;
volatile int ms_delay_obtener_valores = 0;
volatile bool delay_obtener_valores = 0;
bool obtener_val = 0;
bool conversor = 0;


void tmr(){
  ms ++;
  if(!(ms % 100)){
    del_led_amarillo = !del_led_amarillo;
  }
  if(!(ms % 10000) && estado_actual == parada && !EMERGENCIA) {
    terminal_esp32.println ("Introduzca el cable hasta que la punta salga por la pinza corta-cable.");
    terminal_esp32.println("Asegurese que ambos infrarojos enciendan las 3 luces.");
  }
  if(!(ms % 10000) && estado_actual == parada && EMERGENCIA) {
    terminal_esp32.println ("El sistema está listo apra iniciar nuevamente.");
    terminal_esp32.println("Presione el pulsador cuando desee que continue.");
  }
  if(del_pulso){
    del_pulso_ms ++;
    if(del_pulso_ms >= 200){
      del_pulso_ms = 0;
      del_pulso = 0;
    }
  }
  if(terminal_esp32.available()){
    ms_terminal = 0;
    variable_estado_espera_terminal = 0;
  }
  if(!terminal_esp32.available() && estado_actual == menu){
    ms_terminal++;
    if(ms_terminal >= 10000){
      variable_estado_espera_terminal = 1;
      ms_terminal = 0;
    }
  }
  if(delay_obtener_valores){
    ms_delay_obtener_valores++;
    if(ms_delay_obtener_valores >= 100){
      ms_delay_obtener_valores = 0;
      delay_obtener_valores = 0;
    }
  }
  if(estado_actual == parada){
    delay_easybuzzer = delay_easybuzzer + 1;
    if((delay_easybuzzer > delay_bt_beep) && !delay_easybuzzer_estado) {
      digitalWrite(buzzer, HIGH);
      delay_easybuzzer_estado = 1;
      delay_easybuzzer = 0;
    }
    if((delay_easybuzzer >= delay_beep) && delay_easybuzzer_estado){
      digitalWrite(buzzer, LOW);
      delay_easybuzzer_estado = 0;
      delay_easybuzzer = 0;
    }
  }
}
TickTwo del(tmr, 1, 0, MILLIS);

void mover_motor1(){
  digitalWrite(dir1, dir_mot1);
  digitalWrite(step1, HIGH);
  delay(delay_for_motors);
  digitalWrite(step1, LOW);
  delay(delay_for_motors);
}

void setup() {
  Serial.begin(9600);
  pinMode(notConected_1, OUTPUT); digitalWrite(notConected_1, LOW);
  pinMode(notConected_2, OUTPUT); digitalWrite(notConected_2, LOW);
  pinMode(notConected_35, OUTPUT); digitalWrite(notConected_35, LOW);
  pinMode(notConected_34, OUTPUT); digitalWrite(notConected_34, LOW);
  pinMode(notConected_36, OUTPUT); digitalWrite(notConected_36, LOW);
  pinMode(notConected_39, OUTPUT); digitalWrite(notConected_39, LOW);
  pinMode(ENCODER_DIR, OUTPUT);
  pinMode(step1, OUTPUT);
  pinMode(dir1, OUTPUT); digitalWrite(dir1, dir_mot1);
  pinMode(step2, OUTPUT);
  pinMode(dir2, OUTPUT); digitalWrite(dir2, dir_mot2);
  pinMode(step3, OUTPUT);
  pinMode(dir3, OUTPUT); digitalWrite(dir3, dir_mot3);
  pinMode(led_rojo, OUTPUT);
  pinMode(led_amarillo, OUTPUT);
  pinMode(led_verde, OUTPUT);
  pinMode(pulsador, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(infrarrojo1, INPUT);
  pinMode(infrarrojo2, INPUT);
  pinMode(en_mot1, OUTPUT);
  digitalWrite(en_mot1, HIGH);
  pinMode(en_mot2, OUTPUT);
  digitalWrite(en_mot2, HIGH);
  pinMode(en_mot3, OUTPUT); 
  digitalWrite(en_mot3, HIGH);
  del.start();
  terminal_esp32.begin ("Cortapela-cable");
  digitalWrite(ENCODER_DIR, LOW);
  Wire.begin(ENCODER_SDA, ENCODER_SCL);
  ultima_posicion = encoder.getZPosition() * 200 / 4096;
}

void loop() {
  del.update();
  infra1 = digitalRead(infrarrojo1);
  infra2 = digitalRead(infrarrojo2);
  pulso = digitalRead(pulsador);
  switch (estado_actual) {
    case parada :
      digitalWrite(en_mot1, HIGH);
      digitalWrite(en_mot2, HIGH);
      digitalWrite(en_mot3, HIGH);
      digitalWrite(led_rojo, HIGH);
      digitalWrite(led_amarillo, LOW);
      digitalWrite(led_verde, LOW);
      if(infra1 == LOW && infra2 == LOW && EMERGENCIA == 0){
        devolver_valores();
        estado_actual = menu; 
        break;
      }
      else if(infra1 == LOW && infra2 == LOW && EMERGENCIA == 1){
        devolver_valores();
        if(!pulso && !del_pulso){
          estado_actual = funcionamiento;
          del_pulso = 1;
        }
      }
    break;
    
    case menu :
      while((!obtener_val && !conversor && !Verificacion_de_datos) || (!conversor && obtener_val && !Verificacion_de_datos) || (conversor && !obtener_val && !Verificacion_de_datos)){
        del.update();
        digitalWrite(en_mot1, HIGH);
        digitalWrite(en_mot2, HIGH);
        digitalWrite(en_mot3, HIGH);
        infra1 = digitalRead(infrarrojo1);
        infra2 = digitalRead(infrarrojo2);
        pulso = digitalRead(pulsador);
        digitalWrite(led_verde, LOW);
        digitalWrite(led_amarillo, HIGH);
        digitalWrite(led_rojo, LOW);
        if(obtener_val){
          if(conversor){
            Verificacion_de_datos =  1;
            break;
          }
          else {
            conversor = conversor_mm_pasos();
          }
        }
        else {
          obtener_val = obtener_valores();
        }
        if (infra1 == HIGH || infra2 == HIGH){
          estado_actual = parada;
          terminal_esp32.println("OCURRIO UN ERROR: SE DEJÓ DE DETECTAR EL CABLE!");
          break;
        }
      }

      if(Verificacion_de_datos && infra1 == LOW && infra2 == LOW){
        digitalWrite(led_verde, LOW);
        digitalWrite(led_amarillo, del_led_amarillo);
        digitalWrite(led_rojo, LOW);
        if(!pulso && !del_pulso){
          estado_actual = funcionamiento;
          del_pulso = 1;
        }
        else if (infra1 == HIGH || infra2 == HIGH){
          estado_actual = parada;
          break;
        }
      }
    break;
   
    case funcionamiento :
      digitalWrite(led_verde, HIGH);
      digitalWrite(led_amarillo, LOW);
      digitalWrite(led_rojo, LOW);
      mover_cable();
    break;

    case fin :
      digitalWrite(en_mot1, HIGH);
      digitalWrite(en_mot2, HIGH);
      digitalWrite(en_mot3, HIGH);
      if(terminal_esp32.available()){
        mensaje_toInt_cleanned();

        if(!valor_final){
          terminal_esp32.println("ADIOS!!");
          terminal_esp32.println("Muchas gracias x elegirnos!!");
          while(1);
        }
        if(valor_final){
          terminal_esp32.println("Se reinició el sistema.");
          devolver_todos_los_valores();
        }
      }
    break;

    default :
      digitalWrite(en_mot1, HIGH);
      digitalWrite(en_mot2, HIGH);
      digitalWrite(en_mot3, HIGH);
      estado_actual = parada;
    break;
  }
}

bool mover_cable(){
  for(int i = 0; i < ((largo_cable_pasos * cables_a_cortar) + distance_to_cortaCable_pasos); i++){
    dir_mot1 = 1;
    digitalWrite(dir1, dir_mot1);

    if(infra1 == LOW || infra2 == LOW || (!pulso && !del_pulso)){
      del_pulso = 1;
      EMERGENCIA = 1;
      estado_actual = parada;
      return 0;
    }

    if(estado_de_activacion_para_cortar == 1 && counting_to_cut == largo_cable_pasos){
      digitalWrite(en_mot1, HIGH);
      delay(delay_for_enabled_motors);
      cortar_cable();
      digitalWrite(en_mot1, LOW);
      delay(delay_for_enabled_motors);
    }

    else if (estado_de_activacion_para_cortar == 0 && counting_to_cut == distance_to_cortaCable_pasos){
      estado_de_activacion_para_cortar = 1;
      counting_to_cut = 0;
      digitalWrite(en_mot1, HIGH);
      delay(delay_for_enabled_motors);
      cortar_cable();
      digitalWrite(en_mot1, LOW);
      delay(delay_for_enabled_motors);
    }

    if(pasos_dados_mover_cable_gral == pelado_derecho_pasos){
      digitalWrite(en_mot1, HIGH);
      delay(delay_for_enabled_motors);
      pelar_cable();
      digitalWrite(en_mot1, LOW);
      delay(delay_for_enabled_motors);
    }

    else if(pasos_dados_mover_cable_gral == (largo_cable_pasos - pelado_izquierdo_pasos)){
      digitalWrite(en_mot1, HIGH);
      delay(delay_for_enabled_motors);
      pelar_cable();
      digitalWrite(en_mot1, LOW);
      delay(delay_for_enabled_motors);
    }

    else if(pasos_dados_mover_cable_gral >= largo_cable_pasos){
      pasos_dados_mover_cable_gral = 0;
    }

    mover_motor1();
    counting_to_cut = counting_to_cut + 1;
    pasos_dados_mover_cable_gral = pasos_dados_mover_cable_gral + 1;
  }
}

bool obtener_valores(){
  const int limite_inf_largo_cable = 20;
  const int limite_sup_largo_cable = 1000;
  const float limite_sup_multiplo_pelado_cable = 3.5;
  const float limite_inf_diametro_cable = 2.5;
  const int limite_sup_diametro_cable = 5;
  const int limite_inf_cant_cables = 0;
  const int limite_sup_cant_cables = 20;
  const int val_en_xy_cero = 0;
  const int val_en_xy_uno = 1;
  const int val_en_xy_dos = 2;
  const int val_en_xy_tres = 3;
  const int val_en_xy_cuatro = 4;
  const int val_en_xy_cinco = 5;
  const int val_en_xy_seis = 6;
  if(terminal_esp32.available()){
    switch (etapa_de_datos_actual) {
      case esperando_largo_cable :
        if(deteccion_de_digitos){
          if(largo_cable_mm < limite_inf_largo_cable || largo_cable_mm > limite_sup_largo_cable){
            gramatical_error();
          }
          else {
            terminal_esp32.print("Comando correcto, valor establecido en: ");
            terminal_esp32.print(largo_cable_mm);
            terminal_esp32.println("[mm].");
            if(xy == 2){
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 1;
              imprimir_parametros();
            }
            else {
              etapa_de_datos_actual = esperando_pelado_derecho;
              imprimir_parametros();
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == esperando_largo_cable){
          if (terminal_esp32.available()){
            mensaje_toFloat_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != esperando_largo_cable)){
            break;
          }
        }
      break;

      case esperando_pelado_derecho :
        if(deteccion_de_digitos){
          if((limite_sup_multiplo_pelado_cable * pelado_derecho_mm) > largo_cable_mm || pelado_derecho_mm < 0){
            gramatical_error();
          }
          else {
            terminal_esp32.print("Comando correcto, valor establecido en: ");
            terminal_esp32.print(pelado_derecho_mm);
            terminal_esp32.println("[mm].");
            if(xy == 2){
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 1;
              imprimir_parametros();
            }
            else {
              etapa_de_datos_actual = esperando_pelado_izquierdo;
              imprimir_parametros();
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == esperando_pelado_derecho){
          if (terminal_esp32.available()){
            mensaje_toFloat_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != esperando_pelado_derecho)){
            break;
          }
        }
      break;

      case esperando_pelado_izquierdo :
        if(deteccion_de_digitos){
          if((limite_sup_multiplo_pelado_cable * pelado_izquierdo_mm) > largo_cable_mm || pelado_izquierdo_mm < 0){
            gramatical_error();
          }
          else {
            terminal_esp32.print("Comando correcto, valor establecido en: ");
            terminal_esp32.print(pelado_izquierdo_mm);
            terminal_esp32.println("[mm].");
            if(xy == 2){
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 1;
              imprimir_parametros();
            }
            else {
              etapa_de_datos_actual = esperando_diametro;
              imprimir_parametros();
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == esperando_pelado_izquierdo){
          if (terminal_esp32.available()){
            mensaje_toFloat_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != esperando_pelado_izquierdo)){
            break;
          }
        }
      break;

      case esperando_diametro :
        if(deteccion_de_digitos){
          if(limite_inf_diametro_cable > diametro_cable_mm || diametro_cable_mm > limite_sup_diametro_cable){
            gramatical_error();
          }
          else {
            terminal_esp32.print("Comando correcto, valor establecido en: ");
            terminal_esp32.print(diametro_cable_mm);
            terminal_esp32.println("[mm].");
            if(xy == 2){
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 1;
              imprimir_parametros();
            }
            else {
              etapa_de_datos_actual = esperando_cantidad_de_cables;
              imprimir_parametros();
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == esperando_diametro){
          if (terminal_esp32.available()){
            mensaje_toFloat_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != esperando_diametro)){
            break;
          }
        }
      break;

      case esperando_cantidad_de_cables :
        if(deteccion_de_digitos){
          if(limite_inf_cant_cables > cables_a_cortar || cables_a_cortar > limite_sup_diametro_cable){
            gramatical_error();
          }
          else {
            terminal_esp32.print("Comando correcto, valor establecido en: ");
            terminal_esp32.print(cables_a_cortar);
            terminal_esp32.println(".");
            if(xy == 2){
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 1;
              imprimir_parametros();
            }
            else {
              etapa_de_datos_actual = todos_los_datos_obtenidos;
              xy = 0;
              imprimir_parametros();
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == esperando_cantidad_de_cables){
          if (terminal_esp32.available()){
            mensaje_toInt_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != esperando_cantidad_de_cables)){
            break;
          }
        }
      break;

      case todos_los_datos_obtenidos :
        if(deteccion_de_digitos){
          if(xy == 0){
            if(valor_temporal < 0 || valor_temporal > 1){
              gramatical_error();
            }

            else if (valor_temporal == 0) {
              imprimir_parametros();
            }

            else if (valor_temporal == 1) {
              terminal_esp32.println("Todos los datos fueron guardados y verificados.");
              ms_terminal = 0;
              valor_temporal = -1;
              xy = 0;
              return 1;
            }
          }
          
          else if (xy == 1) {
            if(valor_temporal == val_en_xy_cero || valor_temporal < -1 || valor_temporal > val_en_xy_seis){
              gramatical_error();
            }

            else if (valor_temporal >= val_en_xy_uno && valor_temporal < val_en_xy_seis){
              retorno();
            }

            else if (valor_temporal == val_en_xy_seis){
              terminal_esp32.println("Todos los datos fueron guardados y verificados.");
              ms_terminal = 0;
              valor_temporal = -1;
              xy = 0;
              return 1;
            }
          }
        }

        else while (!deteccion_de_digitos && etapa_de_datos_actual == todos_los_datos_obtenidos){
          if (terminal_esp32.available()){
            mensaje_toInt_cleanned();
          }

          if(deteccion_de_digitos || (etapa_de_datos_actual != todos_los_datos_obtenidos)){
            break;
          }
        }
      break;
    }
  }

  else if (variable_estado_espera_terminal){
    imprimir_parametros();
  }
}

bool conversor_mm_pasos(){
  diametro_cable_pasos = (distancia_maxima_seccion_pelado - diametro_cable_mm) * pasos_maximos_x_vuelta;

  largo_cable_pasos = largo_cable_mm / (((diametro_holder + diametro_cable_mm) /2) * ang_paso);

  pelado_izquierdo_pasos = pelado_izquierdo_mm / (((diametro_holder + diametro_cable_mm) /2) * ang_paso);

  pelado_derecho_pasos = pelado_derecho_mm / (((diametro_holder + diametro_cable_mm) /2) * ang_paso);

  distance_to_cortaCable_pasos = distance_to_cortaCable_mm / (((diametro_holder + diametro_cable_mm) /2) * ang_paso);


  if((largo_cable_pasos <= '0') || 
  (largo_cable_pasos <= pelado_izquierdo_pasos) || 
  (largo_cable_pasos <= pelado_derecho_pasos) || 
  (largo_cable_pasos <= largo_cable_mm) || 
  (largo_cable_pasos <= (pelado_izquierdo_pasos + pelado_derecho_pasos)) || 
  (pelado_izquierdo_pasos <= pelado_izquierdo_mm) || 
  (pelado_derecho_pasos <= pelado_derecho_mm)){
    return 0;
  }
  else{
    return 1;
  }
}

bool contador_de_pasos(){
  int16_t posicion_actual = encoder.getZPosition() * 200 / 4096;

  if (posicion_actual != ultima_posicion && dir_mot2) {
    pasos_reales ++;
    ultima_posicion = posicion_actual;
  }
  if (posicion_actual != ultima_posicion && !dir_mot2) {
    pasos_reales --;
    ultima_posicion = posicion_actual;
  }

  if(pasos_reales != pasos_dados_pelar_cable){
    bool solucionar_disparidad();
  }
  delay (10);
}

bool solucionar_disparidad(){
  if (pasos_reales < pasos_dados_pelar_cable){
    while(pasos_reales < pasos_dados_pelar_cable){
      int posicion_actual = encoder.getZPosition() * 200 / 4096;

      if (posicion_actual != ultima_posicion && dir_mot2) {
        pasos_reales ++;
        ultima_posicion = posicion_actual;
      }
      if (posicion_actual != ultima_posicion && !dir_mot2) {
        pasos_reales --;
        ultima_posicion = posicion_actual;
      }

      if (pasos_reales == pasos_dados_pelar_cable){
        return 1;
      }

      else if (pasos_reales < pasos_dados_pelar_cable){
        digitalWrite(step2, HIGH);
        delay(5);
        digitalWrite(step2, LOW);
        delay(5);
      }
      delay (10);
    }
  }
}

bool devolver_valores(){
  pasos_dados_mover_cable_gral = 0;
  pasos_dados_pelar_cable = 0;
  pasos_reales = 0;
}

bool cortar_cable(){
  digitalWrite(en_mot3, LOW);
  delay(delay_for_enabled_motors);
  for(int i = 0; i <= pasos_cortar_cable; i++){
    digitalWrite(step3, HIGH);
    delay (5);
    digitalWrite(step3, LOW);
    delay(5);
    if (i == pasos_cortar_cable){
      digitalWrite(en_mot3, HIGH);
      return 1;
    }
  }
}

bool pelar_cable(){
  digitalWrite(en_mot2, LOW);
  delay(delay_for_enabled_motors);
  for(int i = 0; (i <= diametro_cable_pasos) && (!direccion_movimiento_pelado); i++){
    dir_mot2 = 1;
    digitalWrite(dir2, dir_mot2);
    digitalWrite(step2, HIGH);
    delay (delay_for_motors);
    digitalWrite(step2, LOW);
    delay(delay_for_motors);
    contador_de_pasos();
    if (i == diametro_cable_pasos) {
      direccion_movimiento_pelado = 1;
    }
  }
  delay (10);
  for(int i = 0; (i <= diametro_cable_pasos) && (direccion_movimiento_pelado); i++){
    dir_mot2 = 0;
    digitalWrite(dir2, dir_mot2);
    digitalWrite(step2, HIGH);
    delay (delay_for_motors);
    digitalWrite(step2, LOW);
    delay(delay_for_motors);
    contador_de_pasos();
    if (i == diametro_cable_pasos) {
      direccion_movimiento_pelado = 0;
      digitalWrite(en_mot2, HIGH);
      return 1;
    }
  }
}

void devolver_todos_los_valores(){
  counting_to_cut = 0;
  distance_to_cortaCable_pasos = 0;
  valor_final = -1;
  estado_de_activacion_para_cortar = 0;
  etapa_de_datos_actual = esperando_largo_cable;
  valor_temporal = -1;
  estado_actual = parada;
  deteccion_de_digitos = 0;
  etapa_de_datos_actual = esperando_largo_cable;
  etapa_actual = etapa1;
  Verificacion_de_datos = 0;
  comando_terminal = "";
  ms_delay_obtener_valores = 0;
  delay_obtener_valores = 0;
  obtener_val = 0;
  conversor = 0;
  cables_cortados = 0;
  cables_cortados = 0;
  cables_a_cortar = 0;
  pelado = 0;
  posicion_del_cable = 0;
  EMERGENCIA = 0;
  direccion_movimiento_pelado = 0;
  pasos_reales = 0;
  contador = 0;
  ultima_posicion = 0;
  pasos_dados_pelar_cable = 0;
  xy = 0;
  variable_estado_espera_terminal = 0;
  ms_terminal = 0;
  delay_easybuzzer_estado = 0;
  delay_easybuzzer = 0;
  condicion_pelado_derecho = 0;
  condicion_pelado_izquierdo = 0;
  condicion_de_movmiento = 0;
  pasos_dados_mover_cable_gral = 0;
  seccion_cable = 0;
  pelado_izquierdo_mm = 0;
  pelado_derecho_mm = 0;
  largo_cable_mm = 0;
  diametro_cable_mm = 0;
  diametro_cable_pasos = 0;
  pelado_izquierdo_pasos = 0;
  pelado_derecho_pasos = 0;
  largo_cable_pasos = 0;
  stepaso = 0;
  leds = 0;
  infra1;
  infra2;
  pulso;
  ms = 0;
  del_motor_pap = 0;
  del_leds = 0;
  condicion_de_avance = 0;
  dir_mot1 = 1;
  dir_mot2 = 1;
  dir_mot3 = 1;
  del_pulso = 0;
  del_pulso_ms = 0;
  del_led_amarillo = 0;
}

float mensaje_toFloat_cleanned(){
  String valorExtraido = "";
  comando_terminal = "";
  deteccion_de_digitos = 0;
  comando_terminal = terminal_esp32.readStringUntil('\n');
  for (int i = 0; i < comando_terminal.length(); i++) {
    if (isDigit(comando_terminal[i])) {
      valorExtraido += comando_terminal[i];
      deteccion_de_digitos = 1;
    }
  }
  float flotante_obtenido = valorExtraido.toFloat();
  if(deteccion_de_digitos){
    if(comando_terminal.startsWith(".") && 
    (comando_terminal.endsWith("MM") || comando_terminal.endsWith("mm")) && 
    (comando_terminal.indexOf("ERROR") == -1 && comando_terminal.indexOf("error") == -1)){
      guardar_datos(flotante_obtenido);
    }

    else {
      deteccion_de_digitos = 0;
    }
  }

  else if (deteccion_de_digitos == 0){
    limpiar_texto_error();
  }
}

int mensaje_toInt_cleanned(){
  String valorExtraido = "";
  deteccion_de_digitos = 0;
  comando_terminal = "";
  comando_terminal = terminal_esp32.readStringUntil('\n');
  for (int i = 0; i < comando_terminal.length(); i++) {
    if (isDigit(comando_terminal[i])) {
      valorExtraido += comando_terminal[i];
      deteccion_de_digitos = 1;
    }
  }
  int entero_obtenido = valorExtraido.toInt();
  if(deteccion_de_digitos){
    if(estado_actual == menu){
      if (etapa_de_datos_actual == todos_los_datos_obtenidos && entero_obtenido != 0 && entero_obtenido != 1 && ((xy == 0) || 
      (entero_obtenido != 3 && entero_obtenido != 4 && entero_obtenido != 5 &&
      entero_obtenido != 6 && xy == 1))){
        gramatical_error();
      }

      else if(comando_terminal.startsWith(".") && 
      (comando_terminal.endsWith("MM") || comando_terminal.endsWith("mm")) && 
      (comando_terminal.indexOf("ERROR") == -1 && comando_terminal.indexOf("error") == -1)){
        guardar_datos(entero_obtenido);
      }
    }

    else if (estado_actual == fin){
      if(entero_obtenido != 0 && entero_obtenido != 1){
        imprimir_parametros();
      }

      else {
        valor_final = entero_obtenido;
      }
    }
      else {
        deteccion_de_digitos = 0;
      }
  }

  else if (deteccion_de_digitos == 0){
    limpiar_texto_error();
  }
}

void gramatical_error(){
  terminal_esp32.println("ERROR <COMANDO INCORRECTO>");
  if(estado_actual == menu){
    if(etapa_de_datos_actual != todos_los_datos_obtenidos){
      imprimir_parametros();
    }

    else if(xy == 0){
      xy = 0;
      etapa_de_datos_actual = todos_los_datos_obtenidos;
      imprimir_parametros();
      ms_terminal = 0;
      valor_temporal = -1;
    }

    else if(xy == 1){
      xy = 1;
      etapa_de_datos_actual = todos_los_datos_obtenidos;
      imprimir_parametros();
    }
  }

  else if(estado_actual == fin){
    imprimir_parametros();
  }
}

void retorno(){
  switch (etapa_de_datos_actual){
    case esperando_largo_cable :
      terminal_esp32.println("ERROR <COMANDO INCORRECTO>");
      etapa_de_datos_actual = esperando_largo_cable;
      imprimir_parametros();
    break;

    case esperando_pelado_derecho :
      terminal_esp32.println("<COMANDO ERROR APLICADO>");
      etapa_de_datos_actual = esperando_largo_cable;
      imprimir_parametros();
    break;

    case esperando_pelado_izquierdo :
      terminal_esp32.println("<COMANDO ERROR APLICADO>");
      etapa_de_datos_actual = esperando_pelado_derecho;
      imprimir_parametros();
    break;

    case esperando_diametro :
      terminal_esp32.println("<COMANDO ERROR APLICADO>");
      etapa_de_datos_actual = esperando_pelado_izquierdo;
      imprimir_parametros();
    break;

    case esperando_cantidad_de_cables :
      terminal_esp32.println("<COMANDO ERROR APLICADO>");
      etapa_de_datos_actual = esperando_diametro;
      imprimir_parametros();
    break;

    case todos_los_datos_obtenidos :
      if(!xy){
        terminal_esp32.println("<COMANDO ERROR APLICADO>");
        etapa_de_datos_actual = esperando_cantidad_de_cables;
        imprimir_parametros();
        xy = 0;
      }

      else if(xy == 1){
        if(valor_temporal == -1){
          terminal_esp32.println("<COMANDO ERROR APLICADO>");
          etapa_de_datos_actual = todos_los_datos_obtenidos;
          xy = 0;
          imprimir_parametros();
          valor_temporal = -1;
        }

        else if(valor_temporal == 1){
          etapa_de_datos_actual = esperando_largo_cable;
          xy = 2;
          imprimir_parametros();
        }

        else if(valor_temporal == 2){
          etapa_de_datos_actual = esperando_pelado_derecho;
          xy = 2;
          imprimir_parametros();
        }

        else if(valor_temporal == 3){
          etapa_de_datos_actual = esperando_pelado_izquierdo;
          xy = 2;
          imprimir_parametros();
        }

        else if(valor_temporal == 4){
          etapa_de_datos_actual = esperando_diametro;
          xy = 2;
          imprimir_parametros();
        }

        else if(valor_temporal == 5){
          etapa_de_datos_actual = esperando_cantidad_de_cables;
          xy = 2;
          imprimir_parametros();
        }
      }
    break;
  }
}

void limpiar_texto_error(){
  comando_terminal = terminal_esp32.readStringUntil('\n');
  String textoFiltrado = "";
  for (int i = 0; i < comando_terminal.length(); i++) {
    char letra_actual = comando_terminal[i];
    letra_actual = tolower(letra_actual);

    if (letra_actual == 'e' || letra_actual == 'r' || letra_actual == 'o') {
      textoFiltrado += letra_actual;
    }
  }

  if(textoFiltrado.indexOf("error") != '-1'){
    retorno();
  }

  else {
    gramatical_error();
  }
}

void imprimir_parametros(){
  const int limite_inf_largo_cable = 20;
  const int limite_sup_largo_cable = 1000;
  const float limite_sup_multiplo_pelado_cable = 3.5;
  const float limite_inf_diametro_cable = 2.5;
  const int limite_sup_diametro_cable = 5;
  const int limite_inf_cant_cables = 0;
  const int limite_sup_cant_cables = 20;
  const int val_en_xy_cero = 0;
  const int val_en_xy_uno = 1;
  const int val_en_xy_dos = 2;
  const int val_en_xy_tres = 3;
  const int val_en_xy_cuatro = 4;
  const int val_en_xy_cinco = 5;
  const int val_en_xy_seis = 6;
  const int máx_letras_x_errorChar = 6;
  const char mensaje_de_error[máx_letras_x_errorChar] = "Error";

  if(estado_actual == menu){
    switch (etapa_de_datos_actual){
      case esperando_largo_cable :
        terminal_esp32.println("Introduzca el valor en [mm] del largo del cable siguiendo los siguientes parametros:");
        terminal_esp32.print(limite_inf_largo_cable);
        terminal_esp32.print("[mm] < [largo del cable] < ");
        terminal_esp32.print(limite_sup_largo_cable);
        terminal_esp32.println("[mm].");
        terminal_esp32.println("Formato de escritura: .[valor en mm sin corchetes]mm");
        largo_cable_mm = 0;
        ms_terminal = 0;
      break;

      case esperando_pelado_derecho :
        terminal_esp32.println("Introduzca el valor en [mm] del desenvainado derecho del cable siguiendo los siguientes parametros:");
        terminal_esp32.print("([desenvainado derecho] x ");
        terminal_esp32.print(limite_sup_multiplo_pelado_cable);
        terminal_esp32.println(") < [largo del cable]");
        terminal_esp32.println("Formato de escritura: .[valor en mm sin corchetes]mm");
        terminal_esp32.println("Introduzca el comando 'ERROR' para poder modificar el valor anterior (valor anterior: [desenvainado derecho del cable])");
        pelado_derecho_mm = 0;
        ms_terminal = 0;
      break;

      case esperando_pelado_izquierdo :
        terminal_esp32.println("Introduzca el valor en [mm] del desenvainado izquierdo del cable siguiendo los siguientes parametros:");
        terminal_esp32.print("([desenvainado derecho] x ");
        terminal_esp32.print(limite_sup_multiplo_pelado_cable);
        terminal_esp32.println(") < [largo del cable]");
        terminal_esp32.println("Formato de escritura: .[valor en mm sin corchetes]mm");
        terminal_esp32.println("Introduzca el comando 'ERROR' para poder modificar el valor anterior (valor anterior: [desenvainado derecho del cable])");
        pelado_izquierdo_mm = 0;
        ms_terminal = 0;
      break;

      case esperando_diametro :
        terminal_esp32.println("Introduzca el valor en [mm] del diametro del cable siguiendo los siguientes parametros:");
        terminal_esp32.print(limite_inf_diametro_cable);
        terminal_esp32.print("[mm] <= [diametro del cable] <= ");
        terminal_esp32.print(limite_sup_diametro_cable);
        terminal_esp32.println("[mm].");
        terminal_esp32.println("Formato de escritura: .[valor en mm sin corchetes]mm");
        terminal_esp32.println("Introduzca el comando 'ERROR' para poder modificar el valor anterior: [desenvainado izquierdo del cable]");
        diametro_cable_mm = 0;
        ms_terminal = 0;
      break;

      case esperando_cantidad_de_cables :
        terminal_esp32.println("Introduzca el valor de la cantidad de cables que desea procesar siguiendo los siguientes parametros:");
        terminal_esp32.print(limite_inf_cant_cables);
        terminal_esp32.print(" < [cantidad de cables] >= ");
        terminal_esp32.print(limite_sup_cant_cables);
        terminal_esp32.println("[mm].");
        terminal_esp32.println("Formato de escritura: .[valor corchetes]");
        terminal_esp32.println("Introduzca el comando 'ERROR' para poder modificar el valor anterior: [desenvainado izquierdo del cable]");
        cables_a_cortar = 0;
        ms_terminal = 0;
      break;

      case todos_los_datos_obtenidos :
        if(xy == 0){
          if(valor_temporal == -1){
            terminal_esp32.println("Todos los datos fueron recopilados debidamente");
            terminal_esp32.print("Digite '");
            terminal_esp32.print(val_en_xy_cero);
            terminal_esp32.println("' si desea revisar los valores asignados.");
            terminal_esp32.print("Digite '");
            terminal_esp32.print(val_en_xy_uno);
            terminal_esp32.println("' si está seguro de haber ingresado correctamente todos los datos.");
            etapa_de_datos_actual = todos_los_datos_obtenidos;
          }

          else if (valor_temporal == 0){
            terminal_esp32.print("Largo del cable: ");
            terminal_esp32.print(largo_cable_mm);
            terminal_esp32.println("[mm].");
            terminal_esp32.print("Desenvainado derecho: ");
            terminal_esp32.print(pelado_derecho_mm);
            terminal_esp32.println("[mm].");
            terminal_esp32.print("Desenvainado izquierdo: ");
            terminal_esp32.print(pelado_izquierdo_mm);
            terminal_esp32.println("[mm].");
            terminal_esp32.print("Diametro del cable: ");
            terminal_esp32.print(diametro_cable_mm);
            terminal_esp32.println("[mm].");
            terminal_esp32.print("Cantidad de cables a cortar: ");
            terminal_esp32.print(cables_a_cortar);
            terminal_esp32.println(".");
            terminal_esp32.println("Estás seguro de haber ingresado todos los datos debidamente?");
            terminal_esp32.print("Envía un '");
            terminal_esp32.print(val_en_xy_cero);
            terminal_esp32.println("' si deseas cambiar alguno de los datos ya ingresados.");
            terminal_esp32.print("Envía un '");
            terminal_esp32.print(val_en_xy_uno);
            terminal_esp32.println("' si ingresaste todos los datos correctos y deseas comenzar");
            xy = 1;
            valor_temporal = -1;
          }
          ms_terminal = 0;
        }
        else if(xy == 1){
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_uno);
          terminal_esp32.println("' si desea alterar el valor del largo del cable");
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_dos);
          terminal_esp32.println("' si desea alterar el valor del desenvainado derecho del cable");
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_tres);
          terminal_esp32.println("' si desea alterar el valor del desenvainado izquierdo del cable");
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_cuatro);
          terminal_esp32.println("' si desea alterar el valor del diametro del cable");
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_cinco);
          terminal_esp32.println("' si desea alterar la cantidad de cables a cortar");
          terminal_esp32.print("Digite '");
          terminal_esp32.print(val_en_xy_seis);
          terminal_esp32.println("' si desea NO cambiar ningún valor y arrancar con el proceso");
          terminal_esp32.print("En caso de necesitar volver al caso anterior digite: '");
          terminal_esp32.print(mensaje_de_error);
          terminal_esp32.print("'.");
          etapa_de_datos_actual = todos_los_datos_obtenidos;
          ms_terminal = 0;
        }
      break;
    }
  }

  else if (estado_actual == fin){
    terminal_esp32.println("desea volver a realizar el proceso?");
    terminal_esp32.println("Escriba un '0' si desea finalizar el programa.");
    terminal_esp32.println("Digite un '1' si desea volver a iniciar el programa");
  }
}

void guardar_datos (int dato_a_guardar){
  if(estado_actual == menu){
    switch (etapa_de_datos_actual){
      case esperando_cantidad_de_cables :
        cables_a_cortar = dato_a_guardar;
      break;

      case todos_los_datos_obtenidos :
        if(!xy){
          valor_temporal = dato_a_guardar;
        }

        else if(xy == 1){
          valor_temporal = dato_a_guardar;
        }
      break;
    }
  }

  else if (estado_actual == fin){
    valor_final = dato_a_guardar;
  }
}

void guardar_datos (float dato_a_guardar){
  switch (etapa_de_datos_actual){
    case esperando_largo_cable :
      largo_cable_mm = dato_a_guardar;
    break;

    case esperando_pelado_derecho :
      pelado_derecho_mm = dato_a_guardar;
    break;

    case esperando_pelado_izquierdo :
      pelado_izquierdo_mm = dato_a_guardar;
    break;

    case esperando_diametro :
      diametro_cable_mm = dato_a_guardar;
    break;
  }
}