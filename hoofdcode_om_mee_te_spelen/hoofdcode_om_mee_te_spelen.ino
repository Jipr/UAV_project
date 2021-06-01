/*
    Simulatie van plaats s en snelheid v van een massa m en een stuwkracht F
    Bewegingsvergelijking:
       m.a = F

    Integratie obv
       a = dv/dt
       v = dx/dt

    => dv = a.dt => v_nw - v_oud = a.dt => v_nw = v_oud + a.dt =>
       v = v + a*dt
       x = x + v*dt
*/
const bool simulator = true;

const float m = 0.500;      // kg
const float F_max = 1.0;
const float F_min = -1.0;
const float bF_max = 1.0;
const float bF_min = -1.0;
const float dr_error_som_min = -1.0;
const float dr_error_som_max = 1.0;
float alpha;                    // m/s2
float omega = 0.0, theta = 0.0;     // Beginwaarden
float dr_error, dr_error_oud, dr_derror, dr_error_som;
float dr_sp = 90.0;
float dr_Kp = 1.0;
float dr_Kd = 2.0;
float dr_Ki = 30.0;
float bF = 0.1;
float I = 0.1;
float dr_Mz = 0.01;

float F = 1;
float a;                    // m/s2
float v = -1.0, x = 310.0;     // Beginwaarden
float st_error, st_error_oud, st_derror;
float st_sp = 300;
float st_Kp = 5.0;
float st_Kd = 1.0;

long t_oud, t_nw;
float dt;

// Zorg dat de plot op één pagina past
const long cyclustijd = 10;           // ms
const long simulatietijd = 10 * 1000; // ms
long tEind;
const int pixels = 500;               // aantal weer te geven punten
const float tijdPerPixel = simulatietijd * 1.0 / pixels;
double tijdVoorNwePixelPlot;

void setup() {
  Serial.begin(57600);
  Serial.print("bF alpha omega theta");  // De legenda
  if (tijdPerPixel < cyclustijd) Serial.println("__XXXXXX__tijdPerPixel.<.cyclustijd___XXXXXX");
  else Serial.println();

  t_oud = millis();
  tijdVoorNwePixelPlot = t_oud;
  tEind = t_oud + simulatietijd;

  dr_error_oud = dr_sp - theta;
  st_error_oud = st_sp - x;

}

void loop() {
  t_nw = millis();
  if (t_nw - t_oud > cyclustijd)
  {
    dt = (t_nw - t_oud) * .001;
    t_oud = t_nw;

    if (not simulator)
    {
    }

    //regelaar

    dr_error = dr_sp - theta;
    dr_derror = dr_error - dr_error_oud;
    dr_error_som += dr_error;
    bF = dr_Kp * dr_error + dr_Kd * dr_derror / dt + dr_Ki * dr_error_som * dt;
    dr_error_som = constrain(dr_error_som, dr_error_som_min, dr_error_som_max);
    bF = constrain(bF, bF_min, bF_max);
    dr_error_oud = dr_error;

    st_error = st_sp - x;
    st_derror = st_error - st_error_oud;
    F = st_Kp * st_error + st_Kd * st_derror / dt;
    F = constrain(F, F_min, F_max);
    st_error_oud = st_error;


    if (simulator)
    {

      alpha = bF / I;
      omega = omega + alpha * dt;
      theta = theta + omega * dt;

      a = F / m;      //
      v = v + a * dt; // deze twee vgln eigenlijk verwisselen van plaats (evt: v += a * dt)
      x = x + v * dt;

      // Alleen plotten als tijdPerPixel is verlopen
      if (t_nw > tijdVoorNwePixelPlot) {
        tijdVoorNwePixelPlot = tijdVoorNwePixelPlot + tijdPerPixel;
        Serial.print(bF);
        Serial.print(" ");
        Serial.print(alpha);
        Serial.print(" ");
        Serial.print(omega);
        Serial.print(" ");
        Serial.println(dr_error);

//        Serial.print(F);
//        Serial.print(" ");
//        Serial.print(a);
//        Serial.print(" ");
//        Serial.print(v);
//        Serial.print(" ");
//        Serial.println(x);
      }
      // Stop met plotten, anders loopt de grafiek van het scherm af
      while (t_nw > tEind); // Vanglus, einde van het programma
    }
  }
}

//void motorsturing(F, bF)  {
//  Flinks = F / 2 - bF / b;
//  Frechts = F / 2 + bF / b;
//  pwm_links = motorfunctie_links();
//  pwm_rechts = motorfunctie_rechts();
//}
//
//void motorfunctie_links(Flinks) {
//  if (Flinks > 0) {
//    pwm_links = avooruit * x + bvooruitlinks;
//  }
//  else
//    (
//      pwm_links = aaachteruit * x + bachteruitlinks;
//  }
//}
