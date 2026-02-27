package org.firstinspires.ftc.teamcode.AGE.libs.libs;

import com.qualcomm.robotcore.util.Range;

public class SpeedController {

    // -----------------------
    // Variabile interne
    // -----------------------
    double lastSpeed = 2e9, lastActualSpeed = 2e9, speed = 0;              // viteza curentă calculată de controller
    double lastTime, startTime;    // timpi pentru calcularea diferențelor de timp
    public double accumulatedError;       // termen integrator (suma erorilor în timp)
    public double lastError, errRate = 0;     // ultima eroare și rata de schimbare a erorii

    // Coeficienți și parametri de configurare
    double kA;           // accelerația maximă admisă (control asupra creșterii vitezei)
    double kTan;         // factor de scalare pentru funcția atan(error * kTan)
    double kV;           // viteza maximă
    double kI;           // câștigul integratorului (termenul I)
    double tol;          // toleranța pentru considerarea că am ajuns la țintă
    double brDist;       // distanța de frânare (folosită în scalarea erorii)
    double cutOffLimit; // limită pentru rata de schimbare a erorii (folosită la onTarget)

    // -----------------------
    // Funcții utilitare
    // -----------------------

    // Returnează timpul curent în secunde
    public double getTime(){
        return (double)System.currentTimeMillis() * 0.001;
    }

    // -----------------------
    // Constructori
    // -----------------------

    // Constructor simplu (fără integral și toleranță)
    public SpeedController(double accel, double maxV, double brakeDist){
        this(accel,maxV,brakeDist,0,0);
    }

    // Constructor cu accelerație, viteză max, distanță frânare și limită pe rata erorii
    public SpeedController(double accel, double maxV, double brakeDist, double cutOff){
        this(accel,maxV,brakeDist,0,0,cutOff);
    }

    // Constructor cu integral și toleranță (folosit pentru mai mult control fin)
    public SpeedController(double accel, double maxV, double brakeDist, double integral, double tolerance){
        this(accel,maxV,brakeDist,integral,tolerance,tolerance);
    }

    // Constructor complet
    public SpeedController(double accel, double maxV, double brakeDist, double integral, double tolerance, double cutOff){
        kA = accel;
        kTan = Math.PI * 0.5 / brakeDist; // factor pentru atan, face ca funcția să ajungă la π/2 la brakeDist
        brDist = brakeDist;
        kV = maxV;
        kI = integral;
        tol = tolerance;
        cutOffLimit = cutOff;

        // inițializări
        accumulatedError = 0;
        startTime = lastTime = getTime(); // valori mari inițiale (pentru a evita false positives la început)
        errRate = lastError = 0;
    }

    // -----------------------
    // Filtrare zgomot eroare
    // -----------------------

    double filteredError = 0;      // eroarea filtrată (stocată între apeluri)
    double alpha = 0.8;            // coeficient pentru filtrarea exponențială (EMA)

    // Actualizează coeficienții controller-ului dinamic
    public void updateCoef(double accel, double maxV, double brakeDist, double integral, double tolerance, double cutOff){
        kA = accel;
        kTan = Math.PI * 0.5 / brakeDist;
        kV = maxV;
        kI = integral;
        tol = tolerance;
        cutOffLimit = cutOff;
    }

    // Varianta veche de calcul (fără integral și filtrare) – păstrată doar pentru test/comparație
    public double getSpeed(double error){
        double time = getTime();

        // accelerație non-lineară în timp
        speed = kA * (time - startTime);
        speed = Range.clip(speed, 0, kV);

        // scaling cu atan pentru eroare
        return Range.clip(Math.atan(error * kTan) * speed, -kV, kV);
    }

    // Aplica filtru exponențial pe eroare
    private double filterError(double error) {

        // exponential moving average (EMA) – netezește zgomotul
        filteredError = alpha * error + (1 - alpha) * filteredError;
        //low-pass filter

        return filteredError;
    }

    // -----------------------
    // Funcția principală: getSpeed
    // -----------------------

    public double getSpeed2(double rawError) {
        // filtrăm eroarea pentru a reduce zgomotul senzorilor
        double error = filterError(rawError);
        double time = getTime();

        // calcul integral doar în zona de frânare
        if (Math.abs(lastSpeed) <= 0.1 && time - lastTime >= Config.readingTime) {
            accumulatedError += error * (time - lastTime);

            errRate = (lastError - error) / (time - lastTime);

            lastTime = time;
            lastError = error;
        }

        // reset integrator dacă eroarea e în toleranță
        if (Math.abs(error) <= tol) accumulatedError = 0;

        // creștere a vitezei cu accel + termen integral
        speed = kA * (time - startTime);
        speed = Range.clip(speed, 0, kV) * Math.atan(error * kTan) + kI * accumulatedError;

        lastSpeed = Range.clip(speed, -kV, kV);

        return lastSpeed * (onTarget() ? 0:1);
    }

    // -----------------------
    // Alte funcții utile
    // -----------------------

    // Verifică dacă am ajuns la țintă (eroare mică și rata erorii mică)
    public boolean onTarget() {
        return Math.abs(lastError) <= tol;
    }

    // Resetează controller-ul la starea inițială
    public void resetSpeed() {
        speed = 0;
        lastTime = startTime = getTime();
        accumulatedError = 0;
        errRate = lastError = 2e9;
    }


}