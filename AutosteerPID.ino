void calcSteeringPID(void) {

 if (gpsSpeed < 0.2) {
    pwmDrive = 0; // Lenkmotor komplett stoppen
    return;       // Funktion sofort verlassen, keine weiteren Berechnungen
  }
  
/*
// Ersetzt die Originalzeile:
const float AMPLITUDE = 2.0;      // Die maximale Abweichung in Grad (also ±2.0 Grad)
const float PERIOD_MS = 3000.0;   // Die Dauer einer kompletten Schwingung in Millisekunden (z.B. 6 Sekunden)

float sineWaveError = AMPLITUDE * sin(millis() * (TWO_PI / PERIOD_MS));
steerAngleError = sineWaveError;
*/


if(isallnavy)
{
allyfaktor=2;
}
else allyfaktor=1;


  //Proportional only
  pValue = (steerSettings.Kp/allyfaktor) * steerAngleError;
  pwmDrive = (int16_t)pValue;

  errorAbs = fabs(steerAngleError);
  
  int16_t newMax = 0;

  // ✅ SCHUTZ 1: Prüfe LOW_HIGH_DEGREES vor Division
    if (LOW_HIGH_DEGREES <= 0) {
        // Fallback auf sicheren Standardwert
        Serial.println("ERROR: LOW_HIGH_DEGREES invalid, using default 30°");
        highLowPerDeg = (float)(steerSettings.highPWM - steerSettings.lowPWM) / 30.0;
    } else {
        highLowPerDeg = (float)(steerSettings.highPWM - steerSettings.lowPWM) / LOW_HIGH_DEGREES;
    }

    // ✅ SCHUTZ 2: Prüfe ob highLowPerDeg valide ist
    if (isnan(highLowPerDeg) || isinf(highLowPerDeg)) {
        Serial.println("ERROR: highLowPerDeg calculation failed");
        highLowPerDeg = 0;
    }

  if (errorAbs < LOW_HIGH_DEGREES) {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;

  } else newMax = steerSettings.highPWM;

  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;




  //limit the pwm drive
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;
  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

  if (steerConfig.IsDanfoss) {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2;  // Devide by 4
    pwmDrive += 128;           // add Center Pos.

    // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
    // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
    // Voh=11.08 volts, Vol=0.185v
    // (11.08/12)*255=235.45
    // (0.185/12)*255=3.93
    // output now lies in the range [67 ... 205], the center position is now 136
    //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
  }
}

//#########################################################################################

void motorDrive(void) {
  if (isKeya) {
    // HAUPTSCHALTER: Führe die Motorlogik NUR aus, wenn die Lenkung aktiv sein soll.
    if (isKeyaSteeringActive) {
      // Intelligente Aktivierung für das "Zero-Speed-Stop"-Problem.
      // Wird ausgelöst, wenn von Stillstand zu Bewegung gewechselt wird.
      if (previousPwmDrive == 0 && pwmDrive != 0) {
        enableKeyaSteer();
      }
      // Sende immer den aktuellen Geschwindigkeitsbefehl.
      // pwmDrive = 0 bedeutet jetzt "aktiv halten, aber nicht bewegen".
      SteerKeya(pwmDrive);
    }
    // Wenn isKeyaSteeringActive == false ist, passiert hier absolut nichts.
    // Der disable-Befehl wurde bereits gesendet.

    // Anzeige-Logik bleibt bestehen
    pwmDisplay = abs(pwmDrive);

  } else if (steerConfig.CytronDriver) {
    // Cytron-Logik (unverändert)
    if (pwmDrive >= 0) {
      digitalWrite(DIR1_RL_ENABLE, HIGH);
    } else {
      digitalWrite(DIR1_RL_ENABLE, LOW);
      pwmDrive = -1 * pwmDrive;
    }
    analogWrite(PWM1_LPWM, pwmDrive);
    pwmDisplay = pwmDrive;

  } else {
    // IBT 2-Logik (unverändert)
    if (pwmDrive > 0) {
      analogWrite(PWM2_RPWM, 0);
      analogWrite(PWM1_LPWM, pwmDrive);
    } else {
      pwmDrive = -1 * pwmDrive;
      analogWrite(PWM1_LPWM, 0);
      analogWrite(PWM2_RPWM, pwmDrive);
    }
    pwmDisplay = pwmDrive;
  }

  // WICHTIG: Aktualisiere den letzten Zustand am Ende der Funktion.
  // Wenn die Lenkung inaktiv ist, wird previousPwmDrive auf 0 zurückgesetzt.
  // Das stellt sicher, dass die nächste Aktivierung den enable-Befehl korrekt auslöst.
  if (isKeyaSteeringActive) {
    previousPwmDrive = pwmDrive;
  } else {
    previousPwmDrive = 0;
  }
}
