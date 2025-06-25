/**
 * @file Neurexo.ino
 * @brief Versión del exoesqueleto con triple confirmación para la activación de motores.
 *
 * La lógica de actuación requiere que se cumplan tres condiciones simultáneamente:
 * 1.  El clasificador DTW/k-NN predice un movimiento conocido.
 * 2.  La señal del sensor de flexión es creciente y supera un umbral.
 * 3.  La señal del sensor EMG es creciente y supera un umbral.
 * 
 * Esto aumenta significativamente la seguridad y robustez del sistema, evitando activaciones
 * accidentales y asegurando que la asistencia solo se proporciona cuando la intención
 * del usuario es clara y confirmada por múltiples fuentes.
 */

// --- LIBRERÍAS ---
#include <math.h>

// --- PARÁMETROS DE AJUSTE (TUNING) ---
// --- Parámetros del Clasificador ---
#define K_NEIGHBORS 3                      // Número de vecinos a considerar en k-NN. Debe ser impar.
#define NUM_EXPERT_TRAJECTORIES 4          // Número de demostraciones expertas en nuestra base de datos.
#define MAX_TRAJECTORY_POINTS 50           // Longitud máxima de una trayectoria (en número de puntos).

// --- Parámetros de Detección de Movimiento ---
#define START_TRACKING_DISTANCE_SQ  (5 * 5)  // Distancia (al cuadrado) para empezar a grabar un movimiento.
#define STOP_TRACKING_DISTANCE_SQ   (3 * 3)  // Distancia para considerar que el movimiento ha terminado.

// --- Parámetros de Sensores ---
#define FLEX_ACTIVATION_THRESHOLD  1300 // Valor ADC que debe superar el sensor Flex para considerar activación.
#define EMG_ACTIVATION_THRESHOLD   980 // Valor ADC que debe superar el sensor EMG para considerar activación.
#define SENSOR_TREND_WINDOW        10    // Número de muestras para calcular la tendencia (creciente/decreciente).
#define SENSOR_TREND_HYSTERESIS    50   // Diferencia mínima entre medias para confirmar una tendencia creciente.

// --- DEFINICIONES DE PINES ---
#define LED_PIN_KNEE_RED      13
#define LED_PIN_KNEE_GREEN    12
#define LED_PIN_KNEE_YELLOW   14
#define LED_PIN_HIP_RED       22
#define LED_PIN_HIP_GREEN     23
#define LED_PIN_HIP_YELLOW    15
#define ENCODER_KNEE_CLK      18
#define ENCODER_KNEE_DT       19
#define ENCODER_HIP_CLK       5
#define ENCODER_HIP_DT        21
#define MOTOR_HIP_IN1         2
#define MOTOR_HIP_IN2         0
#define MOTOR_KNEE_IN1        4
#define MOTOR_KNEE_IN2       16
#define FLEX_PIN              34
#define EMG_PIN               35

// --- ESTRUCTURAS Y ENUMS GLOBALES ---
typedef struct { float hip; float knee; } Vector2D;
typedef enum { MOVEMENT_UNKNOWN, KNEE_TO_CHEST, HEEL_TO_BUTT, LEG_FWD_STRAIGHT, LEG_BACK_EXTEND } movementType_t;
typedef enum { DETECTOR_STATE_IDLE, DETECTOR_STATE_TRACKING } detectorState_t;
typedef enum { STOP = 0, FWD = 1, BACK = -1 } DirectionSimple;
// <<< NUEVO: Enum para la tendencia de los sensores >>>
typedef enum { TREND_STABLE, TREND_RISING, TREND_FALLING } SensorTrend;

// Estructura para almacenar una trayectoria de movimiento etiquetada
typedef struct {
    movementType_t label;
    Vector2D points[MAX_TRAJECTORY_POINTS];
    int num_points;
} LabeledTrajectory;

// --- VARIABLES GLOBALES ---
volatile int positionKnee = 0, positionHip = 0, lastStateKnee = 0, lastStateHip = 0;
const int8_t dirTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

LabeledTrajectory expert_trajectories[NUM_EXPERT_TRAJECTORIES];
LabeledTrajectory current_user_trajectory;

detectorState_t detector_state = DETECTOR_STATE_IDLE;
Vector2D startPosition;
movementType_t last_executed_move = MOVEMENT_UNKNOWN;
detectorState_t last_detector_state = DETECTOR_STATE_IDLE;

int flex_samples[SENSOR_TREND_WINDOW] = {0};
int emg_samples[SENSOR_TREND_WINDOW] = {0};
int sensor_sample_index = 0;
SensorTrend flex_trend = TREND_STABLE;
SensorTrend emg_trend = TREND_STABLE;
int current_flex_value = 0;
int current_emg_value = 0;


/****************************************************************/
/*      MÓDULO 1: ALGORITMOS DE PREDICCIÓN (DTW y k-NN)         */
/****************************************************************/
const char* movement_type_to_string(movementType_t type) {
    switch (type) {
        case MOVEMENT_UNKNOWN: return "UNKNOWN";
        case KNEE_TO_CHEST:    return "KNEE_TO_CHEST";
        case HEEL_TO_BUTT:     return "HEEL_TO_BUTT";
        case LEG_FWD_STRAIGHT: return "LEG_FWD_STRAIGHT";
        case LEG_BACK_EXTEND:  return "LEG_BACK_EXTEND";
        default:               return "INVALID";
    }
}

float euclidean_distance_sq(Vector2D p1, Vector2D p2) {
    float dx = p1.hip - p2.hip;
    float dy = p1.knee - p2.knee;
    return dx * dx + dy * dy;
}

float dtw_distance(const LabeledTrajectory* t1, const LabeledTrajectory* t2) {
    int n = t1->num_points;
    int m = t2->num_points;
    if (n == 0 || m == 0) return INFINITY;

    if (n < m) {
        const LabeledTrajectory* temp_t = t1; t1 = t2; t2 = temp_t;
        int temp_len = n; n = m; m = temp_len;
    }
    
    float prev_row[m];
    float current_row[m];

    prev_row[0] = euclidean_distance_sq(t1->points[0], t2->points[0]);
    for (int j = 1; j < m; j++) {
        prev_row[j] = prev_row[j - 1] + euclidean_distance_sq(t1->points[0], t2->points[j]);
    }

    for (int i = 1; i < n; i++) {
        current_row[0] = prev_row[0] + euclidean_distance_sq(t1->points[i], t2->points[0]);
        for (int j = 1; j < m; j++) {
            float cost = euclidean_distance_sq(t1->points[i], t2->points[j]);
            current_row[j] = cost + fmin(fmin(prev_row[j], current_row[j - 1]), prev_row[j - 1]);
        }
        memcpy(prev_row, current_row, sizeof(float) * m);
    }
    
    return sqrtf(prev_row[m - 1]);
}

movementType_t knn_classify(const LabeledTrajectory* current_traj, int k) {
    if (current_traj->num_points < 3) return MOVEMENT_UNKNOWN;

    struct DistanceLabel { float distance; movementType_t label; };
    DistanceLabel distances[NUM_EXPERT_TRAJECTORIES];

    for (int i = 0; i < NUM_EXPERT_TRAJECTORIES; i++) {
        distances[i].distance = dtw_distance(current_traj, &expert_trajectories[i]);
        distances[i].label = expert_trajectories[i].label;
    }

    for (int i = 0; i < NUM_EXPERT_TRAJECTORIES - 1; i++) {
        for (int j = 0; j < NUM_EXPERT_TRAJECTORIES - i - 1; j++) {
            if (distances[j].distance > distances[j + 1].distance) {
                DistanceLabel temp = distances[j];
                distances[j] = distances[j + 1];
                distances[j + 1] = temp;
            }
        }
    }

    int votes[LEG_BACK_EXTEND + 1] = {0};
    for (int i = 0; i < k; i++) {
        votes[distances[i].label]++;
    }

    int max_votes = 0;
    movementType_t best_match = MOVEMENT_UNKNOWN;
    for (int i = 1; i <= LEG_BACK_EXTEND; i++) {
        if (votes[i] > max_votes) {
            max_votes = votes[i];
            best_match = (movementType_t)i;
        }
    }

    return best_match;
}


/****************************************************************/
/*       MÓDULO 2: SENSORES (FLEX y EMG)                        */
/****************************************************************/

/**
 * @brief Procesa las lecturas de los sensores Flex y EMG.
 * Aplica un filtro de media móvil y determina si la tendencia es creciente o decreciente.
 */
void process_aux_sensors() {
    // 1. Leer valores raw de los ADC
    current_flex_value = analogRead(FLEX_PIN);
    current_emg_value = analogRead(EMG_PIN);

    // 2. Almacenar muestras en un buffer circular (media móvil)
    flex_samples[sensor_sample_index] = current_flex_value;
    emg_samples[sensor_sample_index] = current_emg_value;
    sensor_sample_index = (sensor_sample_index + 1) % SENSOR_TREND_WINDOW;

    // 3. Calcular la media de las muestras antiguas y las nuevas para ver la tendencia
    long old_sum_flex = 0, new_sum_flex = 0;
    long old_sum_emg = 0, new_sum_emg = 0;
    
    // Sumar las primeras N-1 muestras (las antiguas)
    for (int i = 0; i < SENSOR_TREND_WINDOW - 1; i++) {
        int index = (sensor_sample_index + i) % SENSOR_TREND_WINDOW;
        old_sum_flex += flex_samples[index];
        old_sum_emg += emg_samples[index];
    }
    // Sumar las últimas N-1 muestras (las nuevas)
    for (int i = 1; i < SENSOR_TREND_WINDOW; i++) {
        int index = (sensor_sample_index + i) % SENSOR_TREND_WINDOW;
        new_sum_flex += flex_samples[index];
        new_sum_emg += emg_samples[index];
    }
    
    float old_avg_flex = (float)old_sum_flex / (SENSOR_TREND_WINDOW - 1);
    float new_avg_flex = (float)new_sum_flex / (SENSOR_TREND_WINDOW - 1);
    float old_avg_emg = (float)old_sum_emg / (SENSOR_TREND_WINDOW - 1);
    float new_avg_emg = (float)new_sum_emg / (SENSOR_TREND_WINDOW - 1);

    // 4. Determinar la tendencia para el sensor de flexión
    if (new_avg_flex > old_avg_flex + SENSOR_TREND_HYSTERESIS) {
        flex_trend = TREND_RISING;
    } else if (new_avg_flex < old_avg_flex - SENSOR_TREND_HYSTERESIS) {
        flex_trend = TREND_FALLING;
    } else {
        flex_trend = TREND_STABLE;
    }

    // 5. Determinar la tendencia para el sensor EMG
    if (new_avg_emg > old_avg_emg + SENSOR_TREND_HYSTERESIS) {
        emg_trend = TREND_RISING;
    } else if (new_avg_emg < old_avg_emg - SENSOR_TREND_HYSTERESIS) {
        emg_trend = TREND_FALLING;
    } else {
        emg_trend = TREND_STABLE;
    }
}


/****************************************************************/
/*        MÓDULO 3: CONTROL DE MOTORES Y MOVIMIENTOS            */
/****************************************************************/
void hip_motor_control(int direction) {
    if (direction > 0) {
        digitalWrite(MOTOR_HIP_IN1, HIGH);
        digitalWrite(MOTOR_HIP_IN2, LOW);
    } else if (direction < 0) {
        digitalWrite(MOTOR_HIP_IN1, LOW);
        digitalWrite(MOTOR_HIP_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_HIP_IN1, HIGH);
        digitalWrite(MOTOR_HIP_IN2, HIGH);
    }
}

void knee_motor_control(int direction) {
    if (direction > 0) {
        digitalWrite(MOTOR_KNEE_IN1, HIGH);
        digitalWrite(MOTOR_KNEE_IN2, LOW);
    } else if (direction < 0) {
        digitalWrite(MOTOR_KNEE_IN1, LOW);
        digitalWrite(MOTOR_KNEE_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_KNEE_IN1, HIGH);
        digitalWrite(MOTOR_KNEE_IN2, HIGH);
    }
}

bool move_to_target(int targetPosition, volatile int* currentPosition, void (*motor_func)(int)) {
    int error = targetPosition - *currentPosition;
    if (abs(error) <= 2) {
        motor_func(0);
        return true;
    }
    motor_func(error > 0 ? 1 : -1);
    return false;
}

void movementAction(movementType_t type) {
    Serial.printf("MOTOR_DRIVER: Acción para %s no implementada.\n", movement_type_to_string(type));
    hip_motor_control(0);
    knee_motor_control(0);
}

/****************************************************************/
/*     MÓDULO 4: ISRs Y LÓGICA DE BAJO NIVEL (HARDWARE)         */
/****************************************************************/
void IRAM_ATTR onKneeEncoder_isr() {
    int clk = digitalRead(ENCODER_KNEE_CLK);
    int dt = digitalRead(ENCODER_KNEE_DT);
    int state = (clk << 1) | dt;
    positionKnee += dirTable[(lastStateKnee << 2) | state];
    lastStateKnee = state;
}

void IRAM_ATTR onHipEncoder_isr() {
    int clk = digitalRead(ENCODER_HIP_CLK);
    int dt = digitalRead(ENCODER_HIP_DT);
    int state = (clk << 1) | dt;
    positionHip += dirTable[(lastStateHip << 2) | state];
    lastStateHip = state;
}

void update_leds(DirectionSimple kneeDir, DirectionSimple hipDir) {
    digitalWrite(LED_PIN_KNEE_GREEN, kneeDir == FWD);
    digitalWrite(LED_PIN_KNEE_RED, kneeDir == BACK);
    digitalWrite(LED_PIN_KNEE_YELLOW, kneeDir == STOP);
    digitalWrite(LED_PIN_HIP_GREEN, hipDir == FWD);
    digitalWrite(LED_PIN_HIP_RED, hipDir == BACK);
    digitalWrite(LED_PIN_HIP_YELLOW, hipDir == STOP);
}


/****************************************************************/
/*      FUNCIÓN DE CONFIGURACIÓN Y CARGA DE DATOS (SETUP)       */
/****************************************************************/
void load_expert_trajectories() {
    const float knee_to_chest_data[][2] = {{0, 0}, {-4, 2}, {-9, 4}, {-12, 6}, {-15, 8}, {-18, 8}, {-16, 7}, {-11, 5}, {-8, 4}, {-4, 2}, {0, 0}};
    expert_trajectories[0].label = KNEE_TO_CHEST;
    expert_trajectories[0].num_points = sizeof(knee_to_chest_data) / sizeof(knee_to_chest_data[0]);
    for(int i=0; i < expert_trajectories[0].num_points; i++) {
        expert_trajectories[0].points[i] = { .hip = knee_to_chest_data[i][1], .knee = knee_to_chest_data[i][0] };
    }

    const float heel_to_butt_data[][2] = {{0, 0}, {-4, 0}, {-6, 0}, {-12, 0}, {-18, 0}, {-12, 0}, {-5, 0}, {0, 0}};
    expert_trajectories[1].label = HEEL_TO_BUTT;
    expert_trajectories[1].num_points = sizeof(heel_to_butt_data) / sizeof(heel_to_butt_data[0]);
    for(int i=0; i < expert_trajectories[1].num_points; i++) {
        expert_trajectories[1].points[i] = { .hip = heel_to_butt_data[i][1], .knee = heel_to_butt_data[i][0] };
    }
    
    const float leg_fwd_data[][2] = {{0, 0}, {0, 4}, {0, 8}, {0, 11}, {0, 8}, {0, 4}, {0, 0}};
    expert_trajectories[2].label = LEG_FWD_STRAIGHT;
    expert_trajectories[2].num_points = sizeof(leg_fwd_data) / sizeof(leg_fwd_data[0]);
    for(int i=0; i < expert_trajectories[2].num_points; i++) {
        expert_trajectories[2].points[i] = { .hip = leg_fwd_data[i][1], .knee = leg_fwd_data[i][0] };
    }

    const float leg_back_data[][2] = {{0, 0}, {-1, -2}, {-2, -4}, {-2, -6}, {-2, -4}, {-1, -2}, {0, 0}};
    expert_trajectories[3].label = LEG_BACK_EXTEND;
    expert_trajectories[3].num_points = sizeof(leg_back_data) / sizeof(leg_back_data[0]);
    for(int i=0; i < expert_trajectories[3].num_points; i++) {
        expert_trajectories[3].points[i] = { .hip = leg_back_data[i][1], .knee = leg_back_data[i][0] };
    }
    
    Serial.println("SETUP: Base de datos de trayectorias expertas cargada.");
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n--- NEUREXO CONTROL - DTW/k-NN FW CON REDUNDANCIA ---");

    int outputPins[] = {LED_PIN_KNEE_RED, LED_PIN_KNEE_GREEN, LED_PIN_KNEE_YELLOW, LED_PIN_HIP_RED, LED_PIN_HIP_GREEN, LED_PIN_HIP_YELLOW, MOTOR_HIP_IN1, MOTOR_HIP_IN2, MOTOR_KNEE_IN1, MOTOR_KNEE_IN2};
    for (int pin : outputPins) { pinMode(pin, OUTPUT); }

    pinMode(ENCODER_KNEE_CLK, INPUT_PULLUP);
    pinMode(ENCODER_KNEE_DT, INPUT_PULLUP);
    pinMode(ENCODER_HIP_CLK, INPUT_PULLUP);
    pinMode(ENCODER_HIP_DT, INPUT_PULLUP);
    
    lastStateKnee = (digitalRead(ENCODER_KNEE_CLK) << 1) | digitalRead(ENCODER_KNEE_DT);
    lastStateHip = (digitalRead(ENCODER_HIP_CLK) << 1) | digitalRead(ENCODER_HIP_DT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_KNEE_CLK), onKneeEncoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_KNEE_DT), onKneeEncoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_HIP_CLK), onHipEncoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_HIP_DT), onHipEncoder_isr, CHANGE);

    load_expert_trajectories();
    movementAction(MOVEMENT_UNKNOWN);
    update_leds(STOP, STOP);
    
    Serial.println("SETUP: Configuración completa. Iniciando bucle principal.");
}


/****************************************************************/
/*                   BUCLE PRINCIPAL (LOOP)                     */
/****************************************************************/
void loop() {
    static unsigned long last_record_time = 0;
    static unsigned long last_aux_sensor_read = 0;
    
    // --- 1. LECTURA Y PROCESAMIENTO DE SENSORES AUXILIARES ---
    if (millis() - last_aux_sensor_read > 20) { // Leer cada 20ms
        process_aux_sensors();
        last_aux_sensor_read = millis();
    }
    
    // --- 2. DETECCIÓN DE INICIO/FIN DE MOVIMIENTO (ENCODERS) ---
    Vector2D current_pos = {(float)positionHip, (float)positionKnee};
    
    if (detector_state == DETECTOR_STATE_IDLE) {
        float dist_sq = euclidean_distance_sq(current_pos, startPosition);
        if (dist_sq > START_TRACKING_DISTANCE_SQ) {
            detector_state = DETECTOR_STATE_TRACKING;
            startPosition = current_pos;
            current_user_trajectory.num_points = 0;
            current_user_trajectory.points[current_user_trajectory.num_points++] = startPosition;
            last_record_time = millis();
            Serial.println("\n--- Tracking INICIADO ---");
        }
    } else { // DETECTOR_STATE_TRACKING
        if (millis() - last_record_time > 50) {
            if (current_user_trajectory.num_points < MAX_TRAJECTORY_POINTS) {
                current_user_trajectory.points[current_user_trajectory.num_points++] = current_pos;
                last_record_time = millis();
            }
        }

        float dist_sq = euclidean_distance_sq(current_pos, startPosition);
        if (dist_sq < STOP_TRACKING_DISTANCE_SQ && current_user_trajectory.num_points > 5) {
            detector_state = DETECTOR_STATE_IDLE;
            startPosition = current_pos;
            Serial.println("\n--- Tracking FINALIZADO (vuelta a reposo) ---");
        } else if (current_user_trajectory.num_points >= MAX_TRAJECTORY_POINTS) {
             detector_state = DETECTOR_STATE_IDLE;
             startPosition = current_pos;
             Serial.println("\n--- Tracking FINALIZADO (buffer lleno) ---");
        }
    }
    
    // --- 3. PREDICCIÓN CON DTW/k-NN ---
    movementType_t predictedMove = MOVEMENT_UNKNOWN;
    if (detector_state == DETECTOR_STATE_TRACKING) {
        predictedMove = knn_classify(current_user_trajectory, K_NEIGHBORS);
    }
    
    // --- 4. LÓGICA DE ACCIÓN CON REDUNDANCIA ---
    bool flex_confirmed = (flex_trend == TREND_RISING && current_flex_value > FLEX_ACTIVATION_THRESHOLD);
    bool emg_confirmed = (emg_trend == TREND_RISING && current_emg_value > EMG_ACTIVATION_THRESHOLD);

    if (detector_state == DETECTOR_STATE_TRACKING &&
        predictedMove != MOVEMENT_UNKNOWN &&
        flex_confirmed &&
        emg_confirmed &&
        last_executed_move == MOVEMENT_UNKNOWN) {
        
        last_executed_move = predictedMove;
        Serial.printf("\n*** TRIPLE CONFIRMACIÓN RECIBIDA PARA: %s. ¡INICIANDO ACCIÓN! ***\n", movement_type_to_string(predictedMove));
        movementAction(predictedMove);
        Serial.println("*** ACCIÓN FINALIZADA. Volviendo a modo detección. ***\n");
        
    } else if (detector_state == DETECTOR_STATE_IDLE && last_detector_state == DETECTOR_STATE_TRACKING) {
        last_executed_move = MOVEMENT_UNKNOWN;
    }
    last_detector_state = detector_state;
    
    // --- 5. FEEDBACK Y DEPURACIÓN ---
    static unsigned long last_print_time = 0;
    if (millis() - last_print_time > 100) {
        static int lastKneePos = 0, lastHipPos = 0;
        DirectionSimple currentKneeDir = (positionKnee > lastKneePos) ? FWD : (positionKnee < lastKneePos) ? BACK : STOP;
        DirectionSimple currentHipDir = (positionHip > lastHipPos) ? FWD : (positionHip < lastHipPos) ? BACK : STOP;
        lastKneePos = positionKnee;
        lastHipPos = positionHip;
        update_leds(currentKneeDir, currentHipDir);
    
        Serial.printf("Time:%lu, Knee:%d, Hip:%d, Flex:%d(T:%d), EMG:%d(T:%d), Pred:%s, State:%s, Action:%s\n",
               millis(), positionKnee, positionHip,
               current_flex_value, flex_trend,
               current_emg_value, emg_trend,
               movement_type_to_string(predictedMove),
               detector_state == DETECTOR_STATE_IDLE ? "IDLE" : "TRACKING",
               movement_type_to_string(last_executed_move));
        last_print_time = millis();
    }

    delay(10);
}
