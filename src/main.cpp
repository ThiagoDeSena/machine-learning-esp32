#include <Arduino.h>
#include <sic-2026-ldr_inferencing.h>

const int PINO_LDR = 1; // Pino analógico do LDR (ADC)

// Pinos dos LEDs para cada classe detectada
const int PINO_LED_COBERTO  = 12; // LED indicador para "coberto"
const int PINO_LED_LANTERNA = 13; // LED indicador para "lanterna"
const int PINO_LED_OCIOSO   = 14; // LED indicador para "ocioso"

// O Edge Impulse define através do modelo:
// - EI_CLASSIFIER_INTERVAL_MS = 20 (50 Hz)
// - EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE = 100 amostras (2 segundos)
// - EI_CLASSIFIER_LABEL_COUNT = 3 ("coberto", "lanterna", "ocioso")

// Constantes para a janela deslizante
const size_t JANELA_TOTAL = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // 100 amostras (2,0 s)
const size_t AVANCO_AMOSTRAS = 25; // 0,5 s de avanço (500 ms / 20 ms = 25 amostras)

// Buffers de amostragem e inferência (duplo buffer)
static float sample_buffer[JANELA_TOTAL];
static float inference_buffer[JANELA_TOTAL];
static size_t sample_index = 0;
static bool buffer_cheio = false;

// Handles das Tarefas do FreeRTOS
TaskHandle_t xTaskSensorHandle = NULL;
TaskHandle_t xTaskInferenceHandle = NULL;

// Semáforos do FreeRTOS
SemaphoreHandle_t xInferenceSemaphore = NULL;
SemaphoreHandle_t xBufferMutex = NULL;

// =====================================================================
// CALLBACK DO EDGE IMPULSE
// =====================================================================
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, inference_buffer + offset, length * sizeof(float));
    return 0;
}

// =====================================================================
// TAREFA 1: LEITURA DO SENSOR COM JANELA DESLIZANTE (AVANÇO DE 0,5 s)
// =====================================================================
void vTaskSensor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(EI_CLASSIFIER_INTERVAL_MS);

    Serial.println("[FreeRTOS] Tarefa de amostragem iniciada (50 Hz, avanço de 0,5 s).");

    while (1) {
        // Temporização precisa de 20 ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        int valorLDR = analogRead(PINO_LDR);

        if (!buffer_cheio) {
            // Fase de aquecimento: preenche as primeiras 100 amostras (2 s)
            sample_buffer[sample_index++] = (float)valorLDR;

            if (sample_index >= JANELA_TOTAL) {
                buffer_cheio = true;

                if (xSemaphoreTake(xBufferMutex, 0) == pdTRUE) {
                    memcpy(inference_buffer, sample_buffer, sizeof(sample_buffer));
                    xSemaphoreGive(xBufferMutex);
                    xSemaphoreGive(xInferenceSemaphore);
                }
                sample_index = 0; // Passa a contar os passos de avanço (25 amostras)
            }
        } else {
            // Fase contínua com avanço de 0,5 s:
            // Desloca o buffer em 1 posição e insere a nova amostra no final
            memmove(&sample_buffer[0], &sample_buffer[1], (JANELA_TOTAL - 1) * sizeof(float));
            sample_buffer[JANELA_TOTAL - 1] = (float)valorLDR;
            sample_index++;

            // A cada 25 novas amostras (0,5 s de avanço), dispara nova inferência
            if (sample_index >= AVANCO_AMOSTRAS) {
                sample_index = 0;

                if (xSemaphoreTake(xBufferMutex, 0) == pdTRUE) {
                    memcpy(inference_buffer, sample_buffer, sizeof(sample_buffer));
                    xSemaphoreGive(xBufferMutex);
                    xSemaphoreGive(xInferenceSemaphore);
                }
            }
        }
    }
}

// =====================================================================
// FUNÇÃO PARA ACIONAR OS LEDS DE ACORDO COM A CLASSE DETECTADA
// =====================================================================
void acionarLeds(const char* label_detectado) {
    if (strcmp(label_detectado, "coberto") == 0) {
        digitalWrite(PINO_LED_COBERTO, HIGH);
        digitalWrite(PINO_LED_LANTERNA, LOW);
        digitalWrite(PINO_LED_OCIOSO, LOW);
    } else if (strcmp(label_detectado, "lanterna") == 0) {
        digitalWrite(PINO_LED_COBERTO, LOW);
        digitalWrite(PINO_LED_LANTERNA, HIGH);
        digitalWrite(PINO_LED_OCIOSO, LOW);
    } else if (strcmp(label_detectado, "ocioso") == 0) {
        digitalWrite(PINO_LED_COBERTO, LOW);
        digitalWrite(PINO_LED_LANTERNA, LOW);
        digitalWrite(PINO_LED_OCIOSO, HIGH);
    }
}

// =====================================================================
// TAREFA 2: PROCESSAMENTO DO MODELO DE ML (EDGE IMPULSE)
// =====================================================================
// Roda em prioridade inferior com stack suficiente (16KB)
// para não interferir na amostragem nem estourar a pilha do ESP32.
void vTaskInference(void *pvParameters) {
    ei_impulse_result_t result = { 0 };

    signal_t features_signal;
    features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    features_signal.get_data = &raw_feature_get_data;

    Serial.println("[FreeRTOS] Tarefa de inferência Edge Impulse iniciada.");

    while (1) {
        // Fica bloqueado até que o avanço da janela esteja pronto
        if (xSemaphoreTake(xInferenceSemaphore, portMAX_DELAY) == pdTRUE) {
            
            // Trava o mutex para garantir que o buffer de inferência está estável
            if (xSemaphoreTake(xBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

                // Executa a inferência Edge Impulse (DSP + Rede Neural TFLite)
                EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
                xSemaphoreGive(xBufferMutex);

                if (res != EI_IMPULSE_OK) {
                    Serial.printf("ERRO: Falha ao rodar classificador (%d)\r\n", res);
                    continue;
                }

                // Encontra a predição com maior probabilidade
                int melhor_indice = 0;
                float maior_score = 0.0f;

                Serial.println("\n-------------------------------------------");
                Serial.printf("Tempo: DSP: %d ms | Inferência: %d ms\r\n", 
                              result.timing.dsp, result.timing.classification);
                Serial.println("Predições:");

                for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    float valor = result.classification[i].value;
                    const char* label = result.classification[i].label;

                    Serial.printf("  [%s]: %.2f%% (%.4f)\r\n", label, valor * 100.0f, valor);

                    if (valor > maior_score) {
                        maior_score = valor;
                        melhor_indice = i;
                    }
                }

                const char* classe_detectada = result.classification[melhor_indice].label;

                Serial.printf(">>> CLASSE DETECTADA: [%s] (Confiança: %.2f%%)\r\n", 
                              classe_detectada, 
                              maior_score * 100.0f);
                Serial.println("-------------------------------------------");

                // Aciona o respectivo LED de acordo com a classe detectada
                acionarLeds(classe_detectada);
            }
        }
    }
}

// =====================================================================
// SETUP & LOOP
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Tempo para abrir o monitor serial

    Serial.println("\n==============================================");
    Serial.println("   Edge Impulse - Reconhecimento com LDR");
    Serial.println("         Framework: ESP32 + FreeRTOS");
    Serial.println("==============================================");
    Serial.printf("Frequência: %d Hz | Janela: %d ms (%d amostras)\r\n",
                  EI_CLASSIFIER_FREQUENCY,
                  EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT,
                  EI_CLASSIFIER_RAW_SAMPLE_COUNT);

    // Configuração do pino do LDR
    pinMode(PINO_LDR, INPUT);

    // Configuração dos pinos dos LEDs
    pinMode(PINO_LED_COBERTO, OUTPUT);
    pinMode(PINO_LED_LANTERNA, OUTPUT);
    pinMode(PINO_LED_OCIOSO, OUTPUT);

    // Inicializa todos os LEDs desligados
    digitalWrite(PINO_LED_COBERTO, LOW);
    digitalWrite(PINO_LED_LANTERNA, LOW);
    digitalWrite(PINO_LED_OCIOSO, LOW);

    // Criação dos primitivos de sincronização do FreeRTOS
    xInferenceSemaphore = xSemaphoreCreateBinary();
    xBufferMutex = xSemaphoreCreateMutex();

    if (xInferenceSemaphore == NULL || xBufferMutex == NULL) {
        Serial.println("Erro ao criar semáforos do FreeRTOS!");
        while (1);
    }

    // Criação da Tarefa de Amostragem do Sensor
    // Prioridade 2 (mais alta): Garante coleta a cada 20ms sem atrasos
    xTaskCreate(
        vTaskSensor,
        "SensorTask",
        4096,
        NULL,
        2,
        &xTaskSensorHandle
    );

    // Criação da Tarefa de Inferência de Machine Learning
    // Prioridade 1: Executa quando a CPU tem tempo livre após o envio dos dados
    // Stack 16384 bytes: Espaço confortável para DSP (FFT) e TFLite Micro
    xTaskCreate(
        vTaskInference,
        "InferenceTask",
        16384,
        NULL,
        1,
        &xTaskInferenceHandle
    );
}

void loop() {
    // O FreeRTOS gerencia as tarefas independentemente através do scheduler.
    // O loop padrão pode ficar em repouso.
    vTaskDelay(pdMS_TO_TICKS(1000));
}