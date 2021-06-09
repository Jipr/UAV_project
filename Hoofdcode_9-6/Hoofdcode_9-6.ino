#include <math.h>
#include "MPU9250.h"

/* pin defines */
// motoren
#define LINKS_PWM_PIN     9
#define LINKS_VOORUIT     10
#define LINKS_ACHTERUIT   11
#define RECHTS_PWM_PIN    12
#define RECHTS_VOORUIT    13
#define RECHTS_ACHTERUIT  14
#define ZIJ_PWM_PIN       22
#define ZIJ_VOORUIT       9
#define ZIJ_ACHTERUIT     8
// Time-of-Flight xshut pins
#define TOF_VOOR        5 // klopt
#define TOF_ZIJ_VOOR    7 // klopt
#define TOF_ZIJ_ACHTER  6 // klopt
// relaits
#define HOOFD_RELAIT  7
#define VENT_RELAIT   8
// knoppen
#define NOODSTOP_KNOP 18

/* systeem constanten */
// hovercraft traagheden en afmetingen constante
#define M 1.216   // massa                          in kg
#define I 0.0246   // traagheidsmoment               in kg.m^2
#define LE 0.45 // lengte                         in m
#define BR 0.29 // breedte                        in m
#define D 0.1355  // afstand motor tot middenlijn   in m
#define B 0.271   // afstand tussen motoren (2*d)   in m

// motor constante
#define MOTOR_LINKS_MAX 0.13734
#define MOTOR_LINKS_MIN -0.14715
#define MOTOR_LINKS_MIN_PWM_VOOR 89.0
#define MOTOR_LINKS_MIN_PWM_ACHTER 77.069
#define MOTOR_LINKS_VOOR_FUNC(F) (2459.2*F - 89)
#define MOTOR_LINKS_ACHTER_FUNC(F) (2211.2*F -77.069)
#define MOTOR_RECHTS_MAX 0.16677
#define MOTOR_RECHTS_MIN -0.14715
#define MOTOR_RECHTS_MIN_PWM_VOOR 59.799
#define MOTOR_RECHTS_MIN_PWM_ACHTER 24.027
#define MOTOR_RECHTS_VOOR_FUNC(F) (1855.9*F - 59.799)
#define MOTOR_RECHTS_ACHTER_FUNC(F) (1907.7*F - 24.027)
#define MOTOR_ZIJ_MAX 0.15
#define MOTOR_ZIJ_MIN -0.15
#define MOTOR_ZIJ_MIN_PWM_VOOR 50
#define MOTOR_ZIJ_MIN_PWM_ACHTER 50
#define MOTOR_ZIJ_VOOR_FUNC(F) 2000*F - 50
#define MOTOR_ZIJ_ACHTER_FUNC(F) 2000*F - 50

/* Waardes Gyroscoop */
MPU9250 IMU(Wire,0x68);
int status;
int resetteller1 = 49;
int resetteller2 = 0;
int resetteller3 = 0;
float xs, ys, as, xa, ya, angle, xc, yc, xai, xaiold, yai, yaiold, asi, asiold, somxa, somya, somas;

/* state difines */
#define IDLE_STATE 0
#define NOODSTOP_STATE 1
int state = IDLE_STATE;

//const bool simulator = true;
struct Vect {
  float x;
  float y;
};

/* bewegins waardes */
// lineare domein
//Vect a = {0.0, 0.0};    // versnelling  in m/s^2
//Vect v = {0.0, 0.0};  // snelheid     in m/s
//Vect s = {0.0, 0.0};  // afstand      in m
float a_x = 0.0;
float a_y = 0.0;
float v_x = 0.0;
float v_y = 0.0;
float s_x = 0.0;
float s_y = 0.0;

//rotatie domein
float alpha = 0.0;  // hoekversnelling  in rad/s^2
float omega = 0.0;  // hoeksnelheid     in rad/s
float theta = 0.0;  // hoek             in rad

/* Krachten */
//Krachten per regelaar
float Fmax = 0.34; // Max kracht voorwaarts
float Fmin = -0.34; // Max kracht achteren
// Snelheids regelaar
float F_v = 0.0; // Kracht voor voorwaartse beweging
// Stand regelaar 
float F_hoek = 0.1;   // N, standregelaar


/*Waardes regelaar */
// Snelheids regelaar Jip
const float sne_Kp = 6;                    // Snelheid Proportionele versterkingsfactor
const float sne_Ki = 0.0;                  // Snelheid Integral versterkingsfactor
const float sne_Kd = 0.0;                  // Snelheid Derivitive versterkingsfactor
float sne_sp = 0.6;                        // Snelheid Setpoint
float sne_error, sne_error_oud, sne_d_error;// Snelheid errors

//Stand regelaar Evelien
const float Kpw = 1;       // Proportionele versterkingsfactor standregelaar
const float Kdw = 1.3;       // differentiële versterkingsfactor standregelaar
const float Kiw = 0.1;       // integrale verstekingsfactor standregelaar
float spw = 0;       // Setpoint standregelaar
float errorw, error_oudw, derrw, xw, errorsomw; //variabelen standregelaar


/* Cycle variables */
const float cyclustijd = 100;  // cyclustijd van de superloop  in ms
long t_oud, t_nw;
float dt;
int dt_ms;

void setup() {
  Serial.begin(57600);

  /* set pin in-/ output */
  // outputs
  pinMode(LINKS_PWM_PIN, OUTPUT);
  pinMode(LINKS_VOORUIT, OUTPUT);
  pinMode(LINKS_ACHTERUIT, OUTPUT);
  pinMode(RECHTS_PWM_PIN, OUTPUT);
  pinMode(RECHTS_VOORUIT, OUTPUT);
  pinMode(RECHTS_ACHTERUIT, OUTPUT);
  pinMode(ZIJ_PWM_PIN, OUTPUT);
  pinMode(ZIJ_VOORUIT, OUTPUT);
  pinMode(ZIJ_ACHTERUIT, OUTPUT);
  
  pinMode(TOF_VOOR, OUTPUT);
  pinMode(TOF_ZIJ_VOOR, OUTPUT);
  pinMode(TOF_ZIJ_ACHTER, OUTPUT);
  
  pinMode(HOOFD_RELAIT, OUTPUT);
  pinMode(VENT_RELAIT, OUTPUT);
  // inputs
  pinMode(NOODSTOP_KNOP, INPUT);

  /* Zet pin voor start up */
  // motoren
  digitalWrite(LINKS_PWM_PIN, LOW);
  digitalWrite(LINKS_VOORUIT, LOW);
  digitalWrite(LINKS_ACHTERUIT, LOW);
  digitalWrite(RECHTS_PWM_PIN, LOW);
  digitalWrite(RECHTS_VOORUIT, LOW);
  digitalWrite(RECHTS_ACHTERUIT, LOW);
  digitalWrite(ZIJ_PWM_PIN, LOW);
  digitalWrite(ZIJ_VOORUIT, LOW);
  digitalWrite(ZIJ_ACHTERUIT, LOW);
  
  digitalWrite(TOF_VOOR, HIGH);
  digitalWrite(TOF_ZIJ_VOOR, HIGH);
  digitalWrite(TOF_ZIJ_ACHTER, HIGH);
  
  digitalWrite(HOOFD_RELAIT, LOW);
  digitalWrite(VENT_RELAIT, LOW);
  
  t_oud = millis();
  
  /* Gyroscoop */
  // start communication with IMU  
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // Berekningen ruis gyroscoop
  for(int i=0; i <= 1000; i++){
    somxa += IMU.getAccelX_mss();
    somya += IMU.getAccelY_mss();
    somas += IMU.getGyroZ_rads()/3.14159265359*180;
  }
  xaiold = IMU.getAccelX_mss();
  yaiold = IMU.getAccelY_mss();
  somxa /= 1000;
  somya /= 1000;
  somas /= 1000;
  /* Eind gyroscoop */

}

void loop() {
  t_nw = millis();
  dt_ms = t_nw - t_oud;
  if (dt_ms > cyclustijd){

    /* Tijd definiëren */
    dt = dt_ms * .001;
    t_oud = t_nw;

    /* Gyroscoop */
     //reset van de ruis
    if((asi > -6) && (asi < 6)){
      resetteller1++;
      asiold += asi;
      if (resetteller1 == 50){
        somas = asiold/50;
        resetteller1 = 0;
        if(angle < 10 && angle > -10){
           angle = 0;
        }  
      }
    }
    if((xai > -0.2) && (xai < 0.2)){
      resetteller2++;
      xaiold += xai;
      if (resetteller2 == 10){
        somxa = xaiold/10;
        xaiold = 0;
        resetteller2 = 0;
        if(xs < 0.5 && xs > -0.5){
           xs = 0;
        } 
        
      }
    }
    if((yai > -0.3) && (yai < 0.3)){
      resetteller3++;
      yaiold += yai;
      if (resetteller3 == 10){
        somya = yaiold/10;
        yaiold = 0;
        resetteller3 = 0;
        if(ys < 2 && ys > -2){
           ys = 0;
        }
      }
    }
    // read the sensor
    IMU.readSensor();
  
    // data
    asi = IMU.getGyroZ_rads()/3.14159265359*180;  // hoeksnelheid inlezing
    as = asi - somas;           // hoeksnelheid met correctie
    angle = angle + (as * dt);  // hoek

    // Opletten x omgekeerd, x is vooruit
    xai = IMU.getAccelX_mss();  // x-as versnelling inlezing
    xa = xai - somxa;           // x-as versnelling met correctie
    xs = xs + (xa * dt);        // x-as snelheid
    xc = xc + (xs * dt);        // x-as positie
    
    yai = IMU.getAccelY_mss();  // y-as versnelling inlezing
    ya = yai - somya;           // y-as versnelling met correctie
    ys = ys + (ya * dt);        // y-as snelheid
    yc = yc + (ys * dt);        // y-as positie
    /* Eind Gyroscoop */
    
    /* Regelaars */
    
    // Snelheidsregelaar Jip
    sne_error = sne_sp - xs;                // Snelheid setpoint - positie : voor Proportionele versterking
    sne_d_error = sne_error - sne_error_oud;   // Snelheid errorverschil : voor Derivitive versterking
    F_v = sne_Kp * sne_error + sne_Kd * sne_d_error / dt; // C, PD regelaar
    F_v = constrain(F_v, Fmin, Fmax);
    sne_error_oud = sne_error;

    // Standregelaar Evelien
    errorw = spw - theta; //theta = hoek gyro
    derrw = errorw - error_oudw;
    F_hoek = Kpw * errorw + Kdw * derrw / dt + Kiw * errorsomw * dt;
    F_hoek = constrain(F_hoek, Fmin, Fmax);
    error_oudw = errorw;
    errorsomw += errorw*dt;
    
    switch (state) {
      case NOODSTOP_STATE:
        // noodstop code
        break;
      default:
        // Idle state
        break;
    }

  }
}

void hoofd_motorsturing(float Fv, float bF, float F_zij)  {
  
  float Flinks = Fv / 2 - bF / B;
  float Frechts = Fv / 2 + bF / B;
  
  set_pwm_links(Flinks);
  set_pwm_rechts(Frechts);

  set_pwm_zij(F_zij);
}

void set_pwm_links(float F) {
  // constrain de krachten tot de minimale en maximale van de motor
  F = constrain(F, MOTOR_LINKS_MIN, MOTOR_LINKS_MAX);
  // DT van het pwm signaal
  float pwm = 0;
  // als de motor vooruit moet
  if (F > 0) {
    pwm = MOTOR_LINKS_VOOR_FUNC(F);
    digitalWrite(LINKS_VOORUIT, HIGH);
    digitalWrite(LINKS_ACHTERUIT, LOW);
  // als de motor acter uit moet
  } else {
    F = fabs(F);
    pwm = MOTOR_LINKS_ACHTER_FUNC(F);
    digitalWrite(LINKS_VOORUIT, LOW);
    digitalWrite(LINKS_ACHTERUIT, HIGH);
  }
  // limiteer het pwm sigaal dus de uiterst mogelijke waarden
  pwm = constrain(pwm, 0, 254);
  analogWrite(LINKS_PWM_PIN, int(pwm));
}
void set_pwm_rechts(float F) {
  // constrain de krachten tot de minimale en maximale van de motor
  F = constrain(F, MOTOR_RECHTS_MIN, MOTOR_RECHTS_MAX);
  // DT van het pwm signaal
  float pwm = 0;
  // als de motor vooruit moet
  if (F > 0) {
    pwm = MOTOR_RECHTS_VOOR_FUNC(F);
    digitalWrite(RECHTS_VOORUIT, HIGH);
    digitalWrite(RECHTS_ACHTERUIT, LOW);
  // als de motor acter uit moet
  } else {
    F = fabs(F);
    pwm = MOTOR_RECHTS_ACHTER_FUNC(F);
    digitalWrite(RECHTS_VOORUIT, LOW);
    digitalWrite(RECHTS_ACHTERUIT, HIGH);
  }
  // limiteer het pwm sigaal dus de uiterst mogelijke waarden
  pwm = constrain(pwm, 0, 254);
  analogWrite(RECHTS_PWM_PIN, int(pwm));
}
void set_pwm_zij(float F) {
  // constrain de krachten tot de minimale en maximale van de motor
  F = constrain(F, MOTOR_ZIJ_MIN, MOTOR_ZIJ_MAX);
  // DT van het pwm signaal
  float pwm = 0;
  // als de motor vooruit moet
  if (F > 0) {
    pwm = MOTOR_ZIJ_VOOR_FUNC(F);
    digitalWrite(ZIJ_VOORUIT, HIGH);
    digitalWrite(ZIJ_ACHTERUIT, LOW);
  // als de motor acter uit moet
  } else {
    F = fabs(F);
    pwm = MOTOR_ZIJ_ACHTER_FUNC(F);
    digitalWrite(ZIJ_VOORUIT, LOW);
    digitalWrite(ZIJ_ACHTERUIT, HIGH);
  }
  // limiteer het pwm sigaal dus de uiterst mogelijke waarden
  pwm = constrain(pwm, 0, 254);
  analogWrite(ZIJ_PWM_PIN, int(pwm));
}
