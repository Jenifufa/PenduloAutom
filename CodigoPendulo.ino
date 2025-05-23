#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h> // Para sqrt(), pow(), fabs(), PI, sin()

// --- CONFIGURACIÓN DEL USUARIO ---
// Pines
#define BUTTON_PIN 9
#define SERVO_PIN 10
#define SENSOR_PIN 12

// LCD
#define LCD_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Péndulo
#define PENDULUM_LENGTH 0.26 // Metros (L)
#define NUM_OSCILLATIONS_FOR_AVG 10
#define KNOWN_GRAVITY 9.80665 // m/s^2
#define INITIAL_RELEASE_ANGLE_DEGREES 30.0 // Ángulo inicial de liberación

// Servomotor
// DEBES AJUSTAR ESTE ÁNGULO MECÁNICAMENTE PARA QUE CORRESPONDA A 50 GRADOS
#define SERVO_HOLD_ANGLE 0   // Ángulo para sostener la pesa (ajusta esto para 50 grados)
#define SERVO_RELEASE_ANGLE 90 // Ángulo para liberar la pesa

// Constantes para filtrado e incertidumbre
#define N_FILTERED_VALUES 5
#define ERROR_LONGITUD 0.001     // Metros
#define ERROR_TIPO_B_MS 1        // Milisegundos
// --- FIN DE CONFIGURACIÓN DEL USUARIO ---

// Objetos
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
Servo pendulumServo;

// Estados del sistema
// ESTA ES LA DEFINICIÓN QUE FALTABA O ESTABA MAL COLOCADA
enum SystemState {
  STATE_IDLE,
  STATE_RELEASING,
  STATE_ARMING_FIRST_PASS,
  STATE_TIMING_HALF_PERIOD,
  STATE_CALCULATING
};
SystemState currentState = STATE_IDLE; // Ahora STATE_IDLE es conocido

// Variables para la medición
unsigned long firstPassTime_ms = 0;
unsigned long all_measured_periods_ms[NUM_OSCILLATIONS_FOR_AVG];
unsigned long filtered_periods_ms[N_FILTERED_VALUES];
int oscillationCount = 0;
bool sensorState = false;
bool lastSensorState = false;
bool sensorDebounceLock = false;


void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SENSOR_PIN, INPUT);
  pendulumServo.attach(SERVO_PIN);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Pendulo Simple");
  lcd.setCursor(0, 1);
  lcd.print("Angulo: "); lcd.print(INITIAL_RELEASE_ANGLE_DEGREES, 0); lcd.print("deg");
  delay(2000);
  lcd.clear();
  lcd.print("Pendulo Simple");
  lcd.setCursor(0, 1);
  lcd.print("Presiona boton");


  pendulumServo.write(SERVO_HOLD_ANGLE);
  currentState = STATE_IDLE; // Correcto
}

void loop() {
  sensorState = (digitalRead(SENSOR_PIN) == LOW); 

  switch (currentState) {
    case STATE_IDLE: // Correcto
      if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
          currentState = STATE_RELEASING; // Correcto
        }
      }
      break;

    case STATE_RELEASING: // Correcto
      lcd.clear();
      lcd.print("Soltando pesa...");
      lcd.setCursor(0,1);
      lcd.print("@"); lcd.print(INITIAL_RELEASE_ANGLE_DEGREES,0); lcd.print(" deg");
      pendulumServo.write(SERVO_RELEASE_ANGLE);
      delay(1000);
      
      lcd.clear();
      lcd.print("Midiendo...");
      lcd.setCursor(0,1);
      lcd.print("Osc: 0/"); 
      lcd.print(NUM_OSCILLATIONS_FOR_AVG);
      
      oscillationCount = 0;
      sensorDebounceLock = false;
      lastSensorState = false;
      currentState = STATE_ARMING_FIRST_PASS; // Correcto
      break;

    case STATE_ARMING_FIRST_PASS: // Correcto
      if (sensorState && !lastSensorState && !sensorDebounceLock) {
        firstPassTime_ms = millis(); 
        sensorDebounceLock = true;
        lastSensorState = true;
        currentState = STATE_TIMING_HALF_PERIOD; // Correcto
        Serial.println("ARMADO: Detectada 1ra senal (inicio de T/2).");
      } else if (!sensorState && lastSensorState) {
         sensorDebounceLock = false;
         lastSensorState = false;
      }
      break;

    case STATE_TIMING_HALF_PERIOD: // Correcto
      if (sensorState && !lastSensorState && !sensorDebounceLock) {
        unsigned long currentTime_ms = millis(); 
        unsigned long halfPeriod_ms = currentTime_ms - firstPassTime_ms;
        unsigned long fullPeriod_ms = halfPeriod_ms * 2; 

        if (oscillationCount < NUM_OSCILLATIONS_FOR_AVG) {
          all_measured_periods_ms[oscillationCount] = fullPeriod_ms;
          
          Serial.print("Oscilacion Completa "); Serial.print(oscillationCount + 1);
          Serial.print(": Intervalo(T/2) = "); Serial.print(halfPeriod_ms);
          Serial.print(" ms, Oscilacion Completa (T) = "); Serial.print(fullPeriod_ms); Serial.println(" ms");

          lcd.setCursor(0,1); 
          lcd.print("Osc:"); lcd.print(oscillationCount + 1); 
          lcd.print(" T:");lcd.print(fullPeriod_ms);      
          lcd.print("ms    ");                            
        }
        oscillationCount++; 
        firstPassTime_ms = currentTime_ms; 
        sensorDebounceLock = true;
        lastSensorState = true;

        if (oscillationCount >= NUM_OSCILLATIONS_FOR_AVG) {
          currentState = STATE_CALCULATING; // Correcto
        }
      } else if (!sensorState && lastSensorState) { 
         sensorDebounceLock = false; 
         lastSensorState = false;
      }
      break;

    case STATE_CALCULATING: // Correcto
      lcd.clear();
      lcd.print("Calculando...");
      Serial.println("\nCalculando resultados...");
      calculateAndDisplayResults(); 
      pendulumServo.write(SERVO_HOLD_ANGLE);
      
      lcd.clear();
      lcd.print("Pendulo Simple");
      lcd.setCursor(0,1);
      lcd.print("Presiona boton");
      currentState = STATE_IDLE; // Correcto
      break;
  }
}


// --- FUNCIONES DE CÁLCULO Y FILTRADO ---

float calculateAverageMs(unsigned long data[], int size) {
  if (size == 0) return 0.0;
  unsigned long sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return (float)sum / size;
}

void findNClosestToAverage(unsigned long original_ms[], int nOriginal, 
                           unsigned long output_ms[], int nOutput) {
  if (nOutput == 0 || nOriginal == 0 ) {
    for(int i=0; i<nOutput; ++i) output_ms[i] = 0;
    return;
  }
  int actual_nOutput = (nOutput > nOriginal) ? nOriginal : nOutput;
  
  float overallAverage_ms = calculateAverageMs(original_ms, nOriginal);
  bool used_flags[nOriginal];
  for(int i=0; i<nOriginal; ++i) used_flags[i] = false;

  for (int i = 0; i < actual_nOutput; i++) {
    long best_index = -1;
    float min_diff = 1.0e9; 

    for (int j = 0; j < nOriginal; j++) {
      if (!used_flags[j]) {
        float diff = fabs((float)original_ms[j] - overallAverage_ms);
        if (diff < min_diff) {
          min_diff = diff;
          best_index = j;
        }
      }
    }
    if (best_index != -1) {
      output_ms[i] = original_ms[best_index];
      used_flags[best_index] = true;
    } else {
      output_ms[i] = 0; 
    }
  }
   for(int i = actual_nOutput; i < nOutput; ++i) {
        output_ms[i] = 0; 
   }
}

float calculateStdDevMs(unsigned long data[], int size, float average_ms) {
  if (size <= 1) return 0.0;
  double sum_sq_diff = 0.0;
  for (int i = 0; i < size; i++) {
    double diff = (double)data[i] - average_ms;
    sum_sq_diff += diff * diff;
  }
  return sqrt(sum_sq_diff / (size - 1));
}

void calculateAndDisplayResults() {
  if (oscillationCount < 1) {
    lcd.clear(); lcd.print("No hay datos!"); Serial.println("No hay datos para calcular.");
    delay(2000); return;
  }

  Serial.println("--- Todos los periodos COMPLETOS (T) medidos (ms) ---");
  for(int i=0; i<oscillationCount; ++i) {
    Serial.print("T"); Serial.print(i+1); Serial.print(": "); Serial.println(all_measured_periods_ms[i]);
  }

  int actual_num_to_filter_for_calc = (oscillationCount < N_FILTERED_VALUES) ? oscillationCount : N_FILTERED_VALUES;
  if (actual_num_to_filter_for_calc == 0 && oscillationCount > 0) actual_num_to_filter_for_calc = oscillationCount;
  
  if (oscillationCount < N_FILTERED_VALUES && oscillationCount > 0) {
    Serial.print("Advertencia: Menos mediciones ("); Serial.print(oscillationCount);
    Serial.print(") que N_FILTERED_VALUES ("); Serial.print(N_FILTERED_VALUES);
    Serial.println("). Usando todas las mediciones disponibles para 'filtrado'.");
  }
  findNClosestToAverage(all_measured_periods_ms, oscillationCount, filtered_periods_ms, actual_num_to_filter_for_calc);
  
  Serial.println("\n--- Periodos Filtrados (ms) ---");
  for(int i=0; i<actual_num_to_filter_for_calc; ++i) {
    Serial.print("Tf"); Serial.print(i+1); Serial.print(": "); Serial.println(filtered_periods_ms[i]);
  }

  float avg_measured_period_ms = calculateAverageMs(filtered_periods_ms, actual_num_to_filter_for_calc);
  float avg_measured_period_s = avg_measured_period_ms / 1000.0; 

  double theta_rad = INITIAL_RELEASE_ANGLE_DEGREES * PI / 180.0;
  double sin_half_theta = sin(theta_rad / 2.0);
  double K_correction_factor = 1.0 + (1.0/4.0) * pow(sin_half_theta, 2); 
  
  float avg_corrected_period_s = avg_measured_period_s; 
  if (INITIAL_RELEASE_ANGLE_DEGREES > 0.1 && K_correction_factor > 0.001) { 
      avg_corrected_period_s = avg_measured_period_s / K_correction_factor; 
      Serial.print("Factor K de correccion por angulo ("); Serial.print(INITIAL_RELEASE_ANGLE_DEGREES,1); Serial.print(" deg): "); Serial.println(K_correction_factor, 5);
      Serial.print("Periodo Medido Promedio (T_medido): "); Serial.print(avg_measured_period_s, 4); Serial.println(" s");
      Serial.print("Periodo Corregido a angulo pequeno (T_corregido): "); Serial.print(avg_corrected_period_s, 4); Serial.println(" s");
  }

  float std_dev_filtered_period_ms = calculateStdDevMs(filtered_periods_ms, actual_num_to_filter_for_calc, avg_measured_period_ms);
  float uncertainty_A_period_ms = (actual_num_to_filter_for_calc > 0) ? std_dev_filtered_period_ms / sqrt(actual_num_to_filter_for_calc) : 0.0;
  float uncertainty_C_period_ms = sqrt(pow(uncertainty_A_period_ms, 2) + pow(ERROR_TIPO_B_MS, 2));
  float uncertainty_C_measured_period_s = uncertainty_C_period_ms / 1000.0;

  double calculated_g = 0.0;
  if (avg_corrected_period_s > 0.0001) {
    calculated_g = (4.0 * PI * PI * PENDULUM_LENGTH) / pow(avg_corrected_period_s, 2);
  }

  double uncertainty_g = 0.0;
  if (avg_measured_period_s > 0.0001 && PENDULUM_LENGTH > 0.0001 && actual_num_to_filter_for_calc > 0) {
    double rel_err_L_sq = pow(ERROR_LONGITUD / PENDULUM_LENGTH, 2);
    double rel_err_T_sq = pow(2.0 * uncertainty_C_measured_period_s / avg_measured_period_s, 2); 
    uncertainty_g = calculated_g * sqrt(rel_err_L_sq + rel_err_T_sq);
  }
   if (actual_num_to_filter_for_calc <= 1) uncertainty_g = 0;

  double percentError = (KNOWN_GRAVITY > 0.0001) ? (fabs(calculated_g - KNOWN_GRAVITY) / KNOWN_GRAVITY) * 100.0 : 0.0;

  lcd.clear();
  lcd.print("Tmed(f):"); lcd.print(avg_measured_period_s, 3); lcd.print("s"); 
  lcd.setCursor(0, 1);
  lcd.print("u(Tm):+/-"); lcd.print(uncertainty_C_measured_period_s, 3); 

  delay(5000); 

  lcd.clear();
  lcd.print("g_calc:"); lcd.print(calculated_g, 2); 
  lcd.setCursor(0, 1);
  lcd.print("u(g):+/-"); lcd.print(uncertainty_g, 2); 
  
  delay(5000);

  lcd.clear();
  lcd.print("%Error: "); lcd.print(percentError, 2); lcd.print("%");
  lcd.setCursor(0,1);
  lcd.print("g_conoc: "); lcd.print(KNOWN_GRAVITY,2);
  delay(5000);

  Serial.println("\n--- Resultados Finales ---");
  Serial.print("Angulo Liberacion: "); Serial.print(INITIAL_RELEASE_ANGLE_DEGREES, 1); Serial.println(" grados");
  Serial.print("Periodo Medido Promedio Filtrado (T_medido): "); Serial.print(avg_measured_period_ms, 2); Serial.print(" ms ("); Serial.print(avg_measured_period_s, 4); Serial.println(" s)");
  if (INITIAL_RELEASE_ANGLE_DEGREES > 0.1) {
    Serial.print("Periodo Corregido a angulo pequeno (T_corregido): "); Serial.print(avg_corrected_period_s * 1000.0, 2); Serial.print(" ms ("); Serial.print(avg_corrected_period_s, 4); Serial.println(" s)");
  }
  Serial.print("Incertidumbre Combinada Periodo Medido (uC_T_medido): +/-"); Serial.print(uncertainty_C_period_ms, 2); Serial.print(" ms ("); Serial.print(uncertainty_C_measured_period_s, 4); Serial.println(" s)");
  Serial.print("Gravedad Calculada (g_calc, usando T_corregido): "); Serial.print(calculated_g, 4); Serial.println(" m/s^2");
  Serial.print("Incertidumbre de g (u_g): +/- "); Serial.print(uncertainty_g, 4); Serial.println(" m/s^2");
  Serial.print("Gravedad Conocida (g_known): "); Serial.print(KNOWN_GRAVITY, 4); Serial.println(" m/s^2");
  Serial.print("Porcentaje de Error: "); Serial.print(percentError, 2); Serial.println(" %");
  Serial.println("------------------------------------");
}
