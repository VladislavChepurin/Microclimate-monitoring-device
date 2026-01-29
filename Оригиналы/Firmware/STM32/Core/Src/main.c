/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"  // Для ПИД регуляторов
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    float humidity;
    uint32_t timestamp;
} SensorData;

typedef struct {
    float temperature_setpoint;
    float humidity_setpoint;
    uint8_t auto_mode;
    uint8_t heating_enabled;
    uint8_t humidification_enabled;
} SystemSettings;

typedef enum {
    HUM_STATUS_OK = 0,
    HUM_STATUS_ALARM,
    HUM_STATUS_RUNNING,
    HUM_STATUS_SERVICE
} HumidifierStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HISTORY_SIZE 48
#define MODBUS_ADDRESS 0x01
#define MODBUS_READ_HOLDING_REG 0x03
#define TEMP_REG_ADDR 0x0001
#define HUM_REG_ADDR 0x0002
#define MODBUS_CRC16_POLYNOMIAL 0xA001

// Константы точности датчика
#define TEMP_ACCURACY 0.5f
#define HUM_ACCURACY 3.0f

// Диапазоны измерений
#define TEMP_MIN -10.0f
#define TEMP_MAX 60.0f
#define HUM_MIN 0.0f
#define HUM_MAX 100.0f

// ПИД параметры для температуры
#define TEMP_KP 2.0f
#define TEMP_KI 0.05f
#define TEMP_KD 1.0f

// ПИД параметры для влажности
#define HUM_KP 1.5f
#define HUM_KI 0.03f
#define HUM_KD 0.8f

// Гистерезис для релейного управления
#define TEMP_HYSTERESIS 0.5f
#define HUM_HYSTERESIS 2.0f

// Максимальное время ожидания ответа от датчика (мс)
#define RS485_TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for readRS485 */
osThreadId_t readRS485Handle;
const osThreadAttr_t readRS485_attributes = {
  .name = "readRS485",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlPIDTemp */
osThreadId_t controlPIDTempHandle;
const osThreadAttr_t controlPIDTemp_attributes = {
  .name = "controlPIDTemp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controlPIDHum */
osThreadId_t controlPIDHumHandle;
const osThreadAttr_t controlPIDHum_attributes = {
  .name = "controlPIDHum",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for exchangeATComma */
osThreadId_t exchangeATCommaHandle;
const osThreadAttr_t exchangeATComma_attributes = {
  .name = "exchangeATComma",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for webInterface */
osThreadId_t webInterfaceHandle;
const osThreadAttr_t webInterface_attributes = {
  .name = "webInterface",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// Глобальные переменные системы
SensorData current_sensor_data = {0};
SensorData history_data[HISTORY_SIZE];
uint8_t history_index = 0;
SystemSettings system_settings = {
    .temperature_setpoint = 22.0f,
    .humidity_setpoint = 50.0f,
    .auto_mode = 1,  // Автоматический режим по умолчанию
    .heating_enabled = 0,
    .humidification_enabled = 0
};

// ПИД регуляторы
PID_HandleTypeDef pid_temp;
PID_HandleTypeDef pid_hum;

// Очереди для межпоточного взаимодействия
osMessageQueueId_t sensor_data_queue;
osMessageQueueId_t settings_queue;
osMessageQueueId_t web_command_queue;

// Семафоры и мьютексы
osMutexId_t sensor_data_mutex;
osMutexId_t settings_mutex;
osMutexId_t history_mutex;

// Флаги состояния
volatile uint8_t heating_active = 0;
volatile uint8_t humidification_active = 0;
volatile uint8_t wifi_ap_active = 0;
volatile uint8_t humidifier_alarm = 0;
volatile uint8_t humidifier_running = 0;
volatile uint8_t humidifier_service = 0;

// Буферы для связи
uint8_t rs485_rx_buffer[64];
uint8_t rs485_tx_buffer[64];
uint8_t esp_rx_buffer[256];
uint8_t esp_tx_buffer[512];
uint8_t modbus_frame[8];

// HTML страница веб-интерфейса
const char* html_page =
"<!DOCTYPE html>"
"<html lang=\"ru\">"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no\">"
"<title>Управление микроклиматом</title>"
"<style>"
"* { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }"
"body { background: linear-gradient(135deg, #1a2980, #26d0ce); color: #fff; min-height: 100vh; padding-bottom: 20px; }"
".container { max-width: 500px; margin: 0 auto; padding: 15px; }"
".navbar { display: flex; justify-content: space-between; margin-bottom: 20px; flex-wrap: wrap; }"
".nav-button { background-color: #0d1b48; color: white; border: none; width: 23%; height: 80px; margin-bottom: 10px; border-radius: 10px; display: flex; flex-direction: column; align-items: center; justify-content: center; font-weight: bold; cursor: pointer; transition: all 0.3s ease; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2); text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.8); -webkit-text-stroke: 0.3px #000; }"
".nav-button:hover { background-color: #1a2e6b; transform: translateY(-2px); }"
".nav-button.active { background-color: #2d46b9; box-shadow: 0 0 15px rgba(45, 70, 185, 0.7); }"
".nav-icon { font-size: 20px; margin-bottom: 5px; }"
".page { display: none; background-color: rgba(255, 255, 255, 0.1); backdrop-filter: blur(10px); border-radius: 15px; padding: 20px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.2); border: 1px solid rgba(255, 255, 255, 0.1); }"
".page.active { display: block; }"
".page-title { text-align: center; margin-bottom: 20px; font-size: 24px; color: #fff; text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5); }"
".sensor-data { display: flex; justify-content: space-between; margin-bottom: 30px; }"
".sensor-box { background: linear-gradient(145deg, rgba(255, 255, 255, 0.15), rgba(255, 255, 255, 0.05)); border-radius: 15px; padding: 20px; width: 48%; text-align: center; box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2); }"
".sensor-value { font-size: 36px; font-weight: bold; margin: 10px 0; }"
".temperature { color: #ff9966; }"
".humidity { color: #66ccff; }"
".unit { font-size: 18px; opacity: 0.8; }"
".control-section { margin-top: 20px; }"
".control-buttons { display: flex; justify-content: space-between; margin-bottom: 20px; }"
".control-button { background-color: #2e7d32; color: white; border: none; border-radius: 10px; padding: 15px; width: 48%; font-size: 16px; font-weight: bold; cursor: pointer; transition: all 0.3s ease; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2); }"
".control-button:hover { background-color: #3d8b40; transform: translateY(-2px); }"
".control-button.off { background-color: #c62828; }"
".control-button.off:hover { background-color: #d32f2f; }"
".indicators { display: flex; justify-content: space-between; }"
".indicator { background-color: rgba(0, 0, 0, 0.3); border-radius: 10px; padding: 15px; width: 48%; text-align: center; font-weight: bold; display: flex; flex-direction: column; align-items: center; }"
".indicator-light { width: 20px; height: 20px; border-radius: 50%; background-color: #555; margin-top: 10px; transition: all 0.3s ease; }"
".indicator-light.on { background-color: #4caf50; box-shadow: 0 0 15px #4caf50; }"
".history-table-container { overflow-x: auto; max-height: 400px; }"
".history-table { width: 100%; border-collapse: collapse; }"
".history-table th, .history-table td { padding: 12px 15px; text-align: center; border-bottom: 1px solid rgba(255, 255, 255, 0.1); }"
".history-table th { background-color: rgba(0, 0, 0, 0.3); position: sticky; top: 0; }"
".history-table tr:hover { background-color: rgba(255, 255, 255, 0.05); }"
".settings-option { background-color: rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 20px; margin-bottom: 20px; }"
".settings-title { font-size: 18px; margin-bottom: 15px; display: flex; align-items: center; }"
".slider-container { display: flex; align-items: center; justify-content: space-between; margin-bottom: 15px; }"
".slider-label { font-weight: bold; }"
".switch { position: relative; display: inline-block; width: 60px; height: 34px; }"
".switch input { opacity: 0; width: 0; height: 0; }"
".slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }"
".slider:before { position: absolute; content: \"\"; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }"
"input:checked + .slider { background-color: #2196F3; }"
"input:checked + .slider:before { transform: translateX(26px); }"
".setpoint-container { background-color: rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 20px; margin-bottom: 20px; opacity: 0.5; transition: all 0.3s ease; }"
".setpoint-container.active { opacity: 1; }"
".setpoint-container.disabled { pointer-events: none; }"
".setpoint-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; }"
".setpoint-label { font-weight: bold; width: 45%; }"
".setpoint-value { font-size: 20px; font-weight: bold; color: #ff9966; width: 25%; text-align: center; }"
".setpoint-controls { display: flex; align-items: center; width: 30%; }"
".setpoint-button { background-color: rgba(255, 255, 255, 0.2); color: white; border: none; border-radius: 50%; width: 30px; height: 30px; font-size: 18px; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: all 0.2s ease; }"
".setpoint-button:hover { background-color: rgba(255, 255, 255, 0.3); }"
".setpoint-button:disabled { opacity: 0.3; cursor: not-allowed; }"
".setpoint-input { margin: 0 10px; font-size: 16px; text-align: center; min-width: 50px; }"
".range-indicator { display: flex; justify-content: space-between; font-size: 12px; opacity: 0.7; margin-top: 5px; }"
".time-setter { display: flex; flex-wrap: wrap; gap: 10px; }"
".time-input { background-color: rgba(255, 255, 255, 0.2); border: 1px solid rgba(255, 255, 255, 0.3); border-radius: 5px; color: white; padding: 10px; width: calc(50% - 5px); font-size: 16px; }"
".time-input::placeholder { color: rgba(255, 255, 255, 0.7); }"
".save-button { background-color: #2196F3; color: white; border: none; border-radius: 5px; padding: 12px; width: 100%; font-size: 16px; font-weight: bold; cursor: pointer; margin-top: 10px; transition: all 0.3s ease; }"
".save-button:hover { background-color: #0b7dda; }"
".info-list { list-style-type: none; }"
".info-item { background-color: rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 15px; margin-bottom: 15px; display: flex; justify-content: space-between; align-items: center; }"
".info-label { font-weight: bold; color: #a5d6ff; }"
".info-value { text-align: right; max-width: 60%; }"
".footer { text-align: center; margin-top: 20px; font-size: 14px; opacity: 0.7; }"
"@media (max-width: 400px) {"
".nav-button { width: 48%; margin-bottom: 10px; }"
".sensor-box { width: 100%; margin-bottom: 15px; }"
".sensor-data { flex-direction: column; }"
".control-button { width: 100%; margin-bottom: 10px; }"
".control-buttons { flex-direction: column; }"
".setpoint-row { flex-direction: column; align-items: flex-start; }"
".setpoint-label, .setpoint-value, .setpoint-controls { width: 100%; margin-bottom: 10px; }"
"}"
"</style>"
"</head>"
"<body>"
"<div class=\"container\">"
"<div class=\"navbar\">"
"<button class=\"nav-button active\" data-page=\"home\"><div class=\"nav-icon\">🏠</div><div>Дом</div></button>"
"<button class=\"nav-button\" data-page=\"history\"><div class=\"nav-icon\">📊</div><div>История</div></button>"
"<button class=\"nav-button\" data-page=\"settings\"><div class=\"nav-icon\">⚙️</div><div>Настройки</div></button>"
"<button class=\"nav-button\" data-page=\"info\"><div class=\"nav-icon\">ℹ️</div><div>Сведения</div></button>"
"</div>"
"<div id=\"home\" class=\"page active\">"
"<h2 class=\"page-title\">Текущие показатели</h2>"
"<div class=\"sensor-data\">"
"<div class=\"sensor-box\"><div>Температура</div><div class=\"sensor-value temperature\" id=\"temperature-value\">%.1f<span class=\"unit\">°C</span></div><div>Диапазон: -10°C ... +60°C</div><div style=\"margin-top: 10px; font-size: 14px; color: #ffcc80;\">Уставка: <span id=\"current-heat-setpoint\">%.1f</span>°C</div></div>"
"<div class=\"sensor-box\"><div>Влажность</div><div class=\"sensor-value humidity\" id=\"humidity-value\">%.1f<span class=\"unit\">%%</span></div><div>Диапазон: 0%% ... 100%%</div><div style=\"margin-top: 10px; font-size: 14px; color: #80deea;\">Уставка: <span id=\"current-hum-setpoint\">%.1f</span>%%</div></div>"
"</div>"
"<div class=\"control-section\">"
"<div class=\"control-buttons\">"
"<button class=\"control-button\" id=\"heating-toggle\">%s</button>"
"<button class=\"control-button\" id=\"humidification-toggle\">%s</button>"
"</div>"
"<div class=\"indicators\">"
"<div class=\"indicator\"><div>Обогревается</div><div class=\"indicator-light\" id=\"heating-indicator\"></div></div>"
"<div class=\"indicator\"><div>Увлажняется</div><div class=\"indicator-light\" id=\"humidification-indicator\"></div></div>"
"</div>"
"<div style=\"margin-top: 20px; text-align: center; padding: 10px; background-color: rgba(0, 0, 0, 0.2); border-radius: 10px;\">"
"<div id=\"mode-status\">Режим: <span style=\"font-weight: bold; color: %s;\">%s</span></div>"
"</div>"
"</div>"
"</div>"
"<div id=\"history\" class=\"page\">"
"<h2 class=\"page-title\">История за сутки</h2>"
"<p style=\"text-align: center; margin-bottom: 15px; opacity: 0.8;\">Данные обновляются каждые 30 минут</p>"
"<div class=\"history-table-container\">"
"<table class=\"history-table\"><thead><tr><th>Дата и время</th><th>Температура</th><th>Влажность</th></tr></thead><tbody id=\"history-table-body\">%s</tbody></table>"
"</div>"
"</div>"
"<div id=\"settings\" class=\"page\">"
"<h2 class=\"page-title\">Настройки системы</h2>"
"<div class=\"settings-option\"><div class=\"settings-title\"><div>Режим работы</div></div>"
"<div class=\"slider-container\"><div class=\"slider-label\">Ручной режим</div><label class=\"switch\"><input type=\"checkbox\" id=\"mode-switch\" %s><span class=\"slider\"></span></label><div class=\"slider-label\">Автоматический</div></div>"
"<p style=\"margin-top: 10px; font-size: 14px; opacity: 0.8;\">В автоматическом режиме система управляет обогревом и увлажнением самостоятельно для поддержания заданных параметров.</p></div>"
"<div class=\"setpoint-container %s\" id=\"heat-setpoint-container\"><div class=\"settings-title\"><div>Уставка обогревателя</div></div>"
"<div class=\"setpoint-row\"><div class=\"setpoint-label\">Температура:</div><div class=\"setpoint-value\" id=\"heat-setpoint-value\">%.1f°C</div>"
"<div class=\"setpoint-controls\"><button class=\"setpoint-button\" id=\"heat-decrease\">-</button><div class=\"setpoint-input\" id=\"heat-setpoint-input\">%.1f</div><button class=\"setpoint-button\" id=\"heat-increase\">+</button></div></div>"
"<div class=\"range-indicator\"><span>Минимум: -10°C</span><span>Максимум: +60°C</span></div>"
"<div style=\"margin-top: 15px; font-size: 14px; opacity: 0.8;\">При снижении температуры ниже уставки будет включаться обогрев.</div></div>"
"<div class=\"setpoint-container %s\" id=\"hum-setpoint-container\"><div class=\"settings-title\"><div>Уставка увлажнителя</div></div>"
"<div class=\"setpoint-row\"><div class=\"setpoint-label\">Влажность:</div><div class=\"setpoint-value\" id=\"hum-setpoint-value\">%.1f%%</div>"
"<div class=\"setpoint-controls\"><button class=\"setpoint-button\" id=\"hum-decrease\">-</button><div class=\"setpoint-input\" id=\"hum-setpoint-input\">%.1f</div><button class=\"setpoint-button\" id=\"hum-increase\">+</button></div></div>"
"<div class=\"range-indicator\"><span>Минимум: 0%%</span><span>Максимум: 100%%</span></div>"
"<div style=\"margin-top: 15px; font-size: 14px; opacity: 0.8;\">При снижении влажности ниже уставки будет включаться увлажнитель.</div></div>"
"<div class=\"settings-option\"><div class=\"settings-title\"><div>Установка времени и даты</div></div>"
"<div class=\"time-setter\"><input type=\"date\" class=\"time-input\" id=\"date-input\" value=\"%s\"><input type=\"time\" class=\"time-input\" id=\"time-input\" value=\"%s\"><button class=\"save-button\" id=\"save-time\">Сохранить время</button></div></div>"
"<button class=\"save-button\" id=\"save-settings\" style=\"background-color: #4caf50;\">Сохранить все настройки</button></div>"
"<div id=\"info\" class=\"page\">"
"<h2 class=\"page-title\">Сведения об устройстве</h2>"
"<ul class=\"info-list\">"
"<li class=\"info-item\"><span class=\"info-label\">Название устройства:</span><span class=\"info-value\">Устройство управления микроклиматом</span></li>"
"<li class=\"info-item\"><span class=\"info-label\">Дата изготовления:</span><span class=\"info-value\">27 февраля 2026 г.</span></li>"
"<li class=\"info-item\"><span class=\"info-label\">Номер прошивки:</span><span class=\"info-value\">0001</span></li>"
"<li class=\"info-item\"><span class=\"info-label\">Учебное заведение:</span><span class=\"info-value\">СФ МЭИ</span></li>"
"<li class=\"info-item\"><span class=\"info-label\">Номер группы:</span><span class=\"info-value\">ПЭ-25з(Маг)</span></li>"
"<li class=\"info-item\"><span class=\"info-label\">Автор:</span><span class=\"info-value\">Чепурин Владислав</span></li>"
"</ul></div>"
"<div class=\"footer\">Устройство управления микроклиматом v1.0 | Связь: %s</div>"
"</div>"
"<script>"
"document.querySelectorAll('.nav-button').forEach(button => {"
"button.addEventListener('click', function() {"
"document.querySelectorAll('.nav-button').forEach(btn => btn.classList.remove('active'));"
"this.classList.add('active');"
"document.querySelectorAll('.page').forEach(page => page.classList.remove('active'));"
"document.getElementById(this.getAttribute('data-page')).classList.add('active');"
"});});"
"const heatingToggle = document.getElementById('heating-toggle');"
"const heatingIndicator = document.getElementById('heating-indicator');"
"const humidificationToggle = document.getElementById('humidification-toggle');"
"const humidificationIndicator = document.getElementById('humidification-indicator');"
"const modeSwitch = document.getElementById('mode-switch');"
"const heatSetpointValue = document.getElementById('heat-setpoint-value');"
"const heatSetpointInput = document.getElementById('heat-setpoint-input');"
"const heatDecreaseBtn = document.getElementById('heat-decrease');"
"const heatIncreaseBtn = document.getElementById('heat-increase');"
"const humSetpointValue = document.getElementById('hum-setpoint-value');"
"const humSetpointInput = document.getElementById('hum-setpoint-input');"
"const humDecreaseBtn = document.getElementById('hum-decrease');"
"const humIncreaseBtn = document.getElementById('hum-increase');"
"const saveTimeBtn = document.getElementById('save-time');"
"const saveSettingsBtn = document.getElementById('save-settings');"
"let heatSetpoint = %.1f;"
"let humSetpoint = %.1f;"
"let heatingOn = %s;"
"let humidificationOn = %s;"
"let autoMode = %s;"
"heatingToggle.addEventListener('click', function() {"
"heatingOn = !heatingOn;"
"fetch('/control?heating=' + (heatingOn ? '1' : '0'));"
"this.textContent = heatingOn ? 'Выключить обогрев' : 'Включить обогрев';"
"this.classList.toggle('off', !heatingOn);"
"heatingIndicator.classList.toggle('on', heatingOn);"
"});"
"humidificationToggle.addEventListener('click', function() {"
"humidificationOn = !humidificationOn;"
"fetch('/control?humidification=' + (humidificationOn ? '1' : '0'));"
"this.textContent = humidificationOn ? 'Выключить увлажнение' : 'Включить увлажнение';"
"this.classList.toggle('off', !humidificationOn);"
"humidificationIndicator.classList.toggle('on', humidificationOn);"
"});"
"modeSwitch.addEventListener('change', function() {"
"autoMode = this.checked;"
"fetch('/control?mode=' + (autoMode ? '1' : '0'));"
"document.getElementById('heat-setpoint-container').classList.toggle('active', autoMode);"
"document.getElementById('hum-setpoint-container').classList.toggle('active', autoMode);"
"heatingToggle.disabled = autoMode;"
"humidificationToggle.disabled = autoMode;"
"});"
"heatDecreaseBtn.addEventListener('click', function() { if(heatSetpoint > -10) { heatSetpoint -= 0.5; updateHeatSetpoint(); } });"
"heatIncreaseBtn.addEventListener('click', function() { if(heatSetpoint < 60) { heatSetpoint += 0.5; updateHeatSetpoint(); } });"
"humDecreaseBtn.addEventListener('click', function() { if(humSetpoint > 0) { humSetpoint -= 1.0; updateHumSetpoint(); } });"
"humIncreaseBtn.addEventListener('click', function() { if(humSetpoint < 100) { humSetpoint += 1.0; updateHumSetpoint(); } });"
"function updateHeatSetpoint() {"
"heatSetpointValue.textContent = heatSetpoint.toFixed(1) + '°C';"
"heatSetpointInput.textContent = heatSetpoint.toFixed(1);"
"document.getElementById('current-heat-setpoint').textContent = heatSetpoint.toFixed(1);"
"fetch('/settings?heat_setpoint=' + heatSetpoint.toFixed(1));"
"}"
"function updateHumSetpoint() {"
"humSetpointValue.textContent = humSetpoint.toFixed(1) + '%%';"
"humSetpointInput.textContent = humSetpoint.toFixed(1);"
"document.getElementById('current-hum-setpoint').textContent = humSetpoint.toFixed(1);"
"fetch('/settings?hum_setpoint=' + humSetpoint.toFixed(1));"
"}"
"saveTimeBtn.addEventListener('click', function() {"
"const date = document.getElementById('date-input').value;"
"const time = document.getElementById('time-input').value;"
"fetch('/settings?date=' + date + '&time=' + time);"
"alert('Время установлено');"
"});"
"saveSettingsBtn.addEventListener('click', function() {"
"fetch('/settings?save_all=1');"
"alert('Настройки сохранены');"
"});"
"setInterval(function() {"
"fetch('/data')"
".then(response => response.json())"
".then(data => {"
"document.getElementById('temperature-value').innerHTML = data.temp.toFixed(1) + '<span class=\"unit\">°C</span>';"
"document.getElementById('humidity-value').innerHTML = data.hum.toFixed(1) + '<span class=\"unit\">%%</span>';"
"if(autoMode) {"
"heatingIndicator.classList.toggle('on', data.heating_active);"
"humidificationIndicator.classList.toggle('on', data.humidification_active);"
"}"
"});"
"}, 3000);"
"</script>"
"</body>"
"</html>";

// JSON для API
const char* json_data_template =
"{\"temp\":%.1f,\"hum\":%.1f,\"heating_active\":%d,\"humidification_active\":%d,"
"\"heat_setpoint\":%.1f,\"hum_setpoint\":%.1f,\"auto_mode\":%d,"
"\"wifi\":%d,\"humidifier_alarm\":%d,\"humidifier_running\":%d,\"humidifier_service\":%d}";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
void StartReadRS485(void *argument);
void StartControlPIDTemp(void *argument);
void StartControlPIDHum(void *argument);
void StartExchangeATCommand(void *argument);
void StartWebInterface(void *argument);

/* USER CODE BEGIN PFP */
// Вспомогательные функции
static void RS485_EnableTX(void);
static void RS485_EnableRX(void);
static uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);
static void Send_Modbus_Request(uint8_t slave_addr, uint8_t function_code, uint16_t reg_addr, uint16_t reg_count);
static uint8_t Parse_Modbus_Response(uint8_t *response, uint16_t *temperature, uint16_t *humidity);
static void Update_History(SensorData data);
static void Update_LEDs(void);
static void Control_Heating(float current_temp, float setpoint);
static void Control_Humidification(float current_hum, float setpoint);
static void Process_Web_Command(char *command);
static void Send_AT_Command(const char *cmd);
static uint8_t Wait_AT_Response(const char *expected, uint32_t timeout);
static void ESP_Init(void);
static void Generate_HTML_Page(char *buffer, uint32_t size);
static void Check_WiFi_Status(void);
static void Generate_JSON_Data(char *buffer, uint32_t size);
static void Generate_History_HTML(char *buffer, uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // Инициализация ПИД регуляторов
  PID_Init(&pid_temp, TEMP_KP, TEMP_KI, TEMP_KD);
  PID_Init(&pid_hum, HUM_KP, HUM_KI, HUM_KD);

  // Установка пределов ПИД регуляторов
  PID_SetOutputLimits(&pid_temp, 0.0f, 1.0f);
  PID_SetOutputLimits(&pid_hum, 0.0f, 1.0f);

  // Инициализация истории
  memset(history_data, 0, sizeof(history_data));

  // Включаем ESP модуль
  HAL_GPIO_WritePin(EN_ESP_Out_GPIO_Port, EN_ESP_Out_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_ESP_Out_GPIO_Port, RST_ESP_Out_Pin, GPIO_PIN_SET);

  // Изначально отключаем реле
  HAL_GPIO_WritePin(Heat_Out_GPIO_Port, Heat_Out_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Humidification_Out_GPIO_Port, Humidification_Out_Pin, GPIO_PIN_RESET);

  // Настраиваем RS485 в режим приема
  RS485_EnableRX();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  // Создание мьютексов
  const osMutexAttr_t mutex_attributes = {
    .name = "SensorDataMutex",
    .attr_bits = osMutexRecursive
  };
  sensor_data_mutex = osMutexNew(&mutex_attributes);

  const osMutexAttr_t settings_mutex_attr = {
    .name = "SettingsMutex",
    .attr_bits = osMutexRecursive
  };
  settings_mutex = osMutexNew(&settings_mutex_attr);

  const osMutexAttr_t history_mutex_attr = {
    .name = "HistoryMutex",
    .attr_bits = osMutexRecursive
  };
  history_mutex = osMutexNew(&history_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // Создание очередей
  sensor_data_queue = osMessageQueueNew(10, sizeof(SensorData), NULL);
  settings_queue = osMessageQueueNew(10, sizeof(SystemSettings), NULL);
  web_command_queue = osMessageQueueNew(20, 64, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of readRS485 */
  readRS485Handle = osThreadNew(StartReadRS485, NULL, &readRS485_attributes);

  /* creation of controlPIDTemp */
  controlPIDTempHandle = osThreadNew(StartControlPIDTemp, NULL, &controlPIDTemp_attributes);

  /* creation of controlPIDHum */
  controlPIDHumHandle = osThreadNew(StartControlPIDHum, NULL, &controlPIDHum_attributes);

  /* creation of exchangeATComma */
  exchangeATCommaHandle = osThreadNew(StartExchangeATCommand, NULL, &exchangeATComma_attributes);

  /* creation of webInterface */
  webInterfaceHandle = osThreadNew(StartWebInterface, NULL, &webInterface_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Сброс сторожего таймера
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Heat_Out_Pin|Humidification_Out_Pin|DE_RS_Out_Pin|RE_RS_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RST_ESP_Out_Pin|EN_ESP_Out_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led_Alarm_Out_Pin|Led_Warning_Out_Pin|Led_WifI_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Heat_Out_Pin Humidification_Out_Pin DE_RS_Out_Pin RE_RS_Out_Pin */
  GPIO_InitStruct.Pin = Heat_Out_Pin|Humidification_Out_Pin|DE_RS_Out_Pin|RE_RS_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HumAlm_In_Pin HumRun_In_Pin HumServ_In_Pin */
  GPIO_InitStruct.Pin = HumAlm_In_Pin|HumRun_In_Pin|HumServ_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_ESP_Out_Pin EN_ESP_Out_Pin */
  GPIO_InitStruct.Pin = RST_ESP_Out_Pin|EN_ESP_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_Alarm_Out_Pin Led_Warning_Out_Pin Led_WifI_Out_Pin */
  GPIO_InitStruct.Pin = Led_Alarm_Out_Pin|Led_Warning_Out_Pin|Led_WifI_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Поток чтения данных с RS485 датчика
  */
void StartReadRS485(void *argument)
{
  uint32_t last_save_time = 0;
  SensorData sensor_data;

  for(;;)
  {
    // Отправка запроса на чтение температуры
    Send_Modbus_Request(MODBUS_ADDRESS, MODBUS_READ_HOLDING_REG,
                       TEMP_REG_ADDR, 1);
    RS485_EnableTX();
    HAL_UART_Transmit(&huart1, modbus_frame, 8, RS485_TIMEOUT);
    RS485_EnableRX();

    // Ожидание ответа
    if(HAL_UART_Receive(&huart1, rs485_rx_buffer, 7, RS485_TIMEOUT) == HAL_OK)
    {
      uint16_t temp_raw;
      if(Parse_Modbus_Response(rs485_rx_buffer, &temp_raw, NULL))
      {
        sensor_data.temperature = temp_raw / 10.0f; // Датчик возвращает значение * 10
      }
    }

    osDelay(10);

    // Отправка запроса на чтение влажности
    Send_Modbus_Request(MODBUS_ADDRESS, MODBUS_READ_HOLDING_REG,
                       HUM_REG_ADDR, 1);
    RS485_EnableTX();
    HAL_UART_Transmit(&huart1, modbus_frame, 8, RS485_TIMEOUT);
    RS485_EnableRX();

    // Ожидание ответа
    if(HAL_UART_Receive(&huart1, rs485_rx_buffer, 7, RS485_TIMEOUT) == HAL_OK)
    {
      uint16_t hum_raw;
      if(Parse_Modbus_Response(rs485_rx_buffer, NULL, &hum_raw))
      {
        sensor_data.humidity = hum_raw / 10.0f; // Датчик возвращает значение * 10
      }
    }

    // Получение текущего времени
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Создание timestamp
    sensor_data.timestamp =
        (sDate.Year + 2000) * 100000000 +
        sDate.Month * 1000000 +
        sDate.Date * 10000 +
        sTime.Hours * 100 +
        sTime.Minutes;

    // Обновление текущих данных с защитой мьютексом
    osMutexAcquire(sensor_data_mutex, osWaitForever);
    current_sensor_data = sensor_data;
    osMutexRelease(sensor_data_mutex);

    // Отправка данных в очередь для других потоков
    osMessageQueuePut(sensor_data_queue, &sensor_data, 0, 0);

    // Сохранение в историю каждые 30 минут
    uint32_t current_time = osKernelGetTickCount();
    if((current_time - last_save_time) >= (30 * 60 * 1000)) // 30 минут в миллисекундах
    {
      Update_History(sensor_data);
      last_save_time = current_time;
    }

    // Обновление светодиодов
    Update_LEDs();

    // Чтение состояния увлажнителя
    humidifier_alarm = HAL_GPIO_ReadPin(HumAlm_In_GPIO_Port, HumAlm_In_Pin);
    humidifier_running = HAL_GPIO_ReadPin(HumRun_In_GPIO_Port, HumRun_In_Pin);
    humidifier_service = HAL_GPIO_ReadPin(HumServ_In_GPIO_Port, HumServ_In_Pin);

    osDelay(5000); // Чтение данных каждые 5 секунд
  }
}

/**
  * @brief Поток управления температурой
  */
void StartControlPIDTemp(void *argument)
{
  SensorData sensor_data;
  float pid_output;

  for(;;)
  {
    // Получение текущих данных из очереди
    if(osMessageQueueGet(sensor_data_queue, &sensor_data, NULL, 100) == osOK)
    {
      // Получение текущих настроек с защитой мьютексом
      SystemSettings settings;
      osMutexAcquire(settings_mutex, osWaitForever);
      settings = system_settings;
      osMutexRelease(settings_mutex);

      if(settings.auto_mode)
      {
        // Автоматический режим - использование ПИД регулятора
        pid_output = PID_Compute(&pid_temp, sensor_data.temperature,
                                settings.temperature_setpoint);

        // Преобразование выхода ПИД в релейное управление
        if(pid_output > 0.5f)
        {
          heating_active = 1;
          HAL_GPIO_WritePin(Heat_Out_GPIO_Port, Heat_Out_Pin, GPIO_PIN_SET);
        }
        else
        {
          heating_active = 0;
          HAL_GPIO_WritePin(Heat_Out_GPIO_Port, Heat_Out_Pin, GPIO_PIN_RESET);
        }
      }
      else
      {
        // Ручной режим
        heating_active = settings.heating_enabled;
        HAL_GPIO_WritePin(Heat_Out_GPIO_Port, Heat_Out_Pin,
                         settings.heating_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

    osDelay(1000); // Управление каждую секунду
  }
}

/**
  * @brief Поток управления влажностью
  */
void StartControlPIDHum(void *argument)
{
  SensorData sensor_data;
  float pid_output;

  for(;;)
  {
    // Получение текущих данных из очереди
    if(osMessageQueueGet(sensor_data_queue, &sensor_data, NULL, 100) == osOK)
    {
      // Получение текущих настроек с защитой мьютексом
      SystemSettings settings;
      osMutexAcquire(settings_mutex, osWaitForever);
      settings = system_settings;
      osMutexRelease(settings_mutex);

      // Проверка аварии увлажнителя
      if(humidifier_alarm)
      {
        // Авария - выключаем увлажнение
        humidification_active = 0;
        HAL_GPIO_WritePin(Humidification_Out_GPIO_Port, Humidification_Out_Pin, GPIO_PIN_RESET);
        osDelay(1000);
        continue;
      }

      if(settings.auto_mode)
      {
        // Автоматический режим - использование ПИД регулятора
        pid_output = PID_Compute(&pid_hum, sensor_data.humidity,
                                settings.humidity_setpoint);

        // Преобразование выхода ПИД в релейное управление
        if(pid_output > 0.5f)
        {
          humidification_active = 1;
          HAL_GPIO_WritePin(Humidification_Out_GPIO_Port, Humidification_Out_Pin, GPIO_PIN_SET);
        }
        else
        {
          humidification_active = 0;
          HAL_GPIO_WritePin(Humidification_Out_GPIO_Port, Humidification_Out_Pin, GPIO_PIN_RESET);
        }
      }
      else
      {
        // Ручной режим
        humidification_active = settings.humidification_enabled;
        HAL_GPIO_WritePin(Humidification_Out_GPIO_Port, Humidification_Out_Pin,
                         settings.humidification_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

    osDelay(1000); // Управление каждую секунду
  }
}

/**
  * @brief Поток обмена AT командами с ESP
  */
void StartExchangeATCommand(void *argument)
{
    char command[64];

    // Инициализация ESP модуля
    ESP_Init();

    for(;;)
    {
        // Проверка очереди команд от веб-интерфейса
        if(osMessageQueueGet(web_command_queue, command, NULL, 0) == osOK)
        {
            Process_Web_Command(command);
        }

        // Проверка состояния Wi-Fi
        Check_WiFi_Status();

        osDelay(1000);
    }
}

/**
  * @brief Поток веб-интерфейса
  */
void StartWebInterface(void *argument)
{
    char http_request[512];
    char http_response[2048];
    uint8_t client_id = 0;
    char *ipd_marker = "+IPD,";

    // Начальная очистка буфера
    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));

    for(;;)
    {
        if(wifi_ap_active)
        {
            // Асинхронное чтение данных от ESP
            uint16_t bytes_available = 0;
            uint8_t tmp_buffer[256];

            // Проверяем, есть ли данные в UART
            if(HAL_UART_Receive(&huart6, tmp_buffer, sizeof(tmp_buffer), 10) == HAL_OK)
            {
                // Добавляем данные в буфер
                strncat((char*)esp_rx_buffer, (char*)tmp_buffer, sizeof(tmp_buffer));

                // Ищем маркер входящих данных "+IPD"
                char *ipd_start = strstr((char*)esp_rx_buffer, ipd_marker);
                if(ipd_start)
                {
                    // Извлекаем ID клиента и длину данных
                    // Формат: +IPD,<client_id>,<length>:<data>
                    char *comma = strchr(ipd_start, ',');
                    if(comma)
                    {
                        client_id = atoi(comma + 1);
                        char *colon = strchr(comma, ':');
                        if(colon)
                        {
                            // Копируем HTTP запрос
                            strncpy(http_request, colon + 1, sizeof(http_request) - 1);
                            http_request[sizeof(http_request) - 1] = '\0';

                            // Обработка HTTP запроса
                            if(strstr(http_request, "GET / "))
                            {
                                // Главная страница
                                Generate_HTML_Page(http_response, sizeof(http_response));
                            }
                            else if(strstr(http_request, "GET /index.html"))
                            {
                                // Главная страница
                                Generate_HTML_Page(http_response, sizeof(http_response));
                            }
                            else if(strstr(http_request, "GET /data"))
                            {
                                // JSON данные для AJAX
                                Generate_JSON_Data(http_response, sizeof(http_response));
                            }
                            else if(strstr(http_request, "GET /control"))
                            {
                                // Обработка команд управления
                                char *query = strstr(http_request, "?");
                                if(query)
                                {
                                    osMessageQueuePut(web_command_queue, query, 0, 0);
                                }
                                snprintf(http_response, sizeof(http_response),
                                         "HTTP/1.1 200 OK\r\n"
                                         "Content-Type: text/plain\r\n"
                                         "Access-Control-Allow-Origin: *\r\n"
                                         "\r\nOK");
                            }
                            else if(strstr(http_request, "GET /settings"))
                            {
                                // Обработка настроек
                                char *query = strstr(http_request, "?");
                                if(query)
                                {
                                    osMessageQueuePut(web_command_queue, query, 0, 0);
                                }
                                snprintf(http_response, sizeof(http_response),
                                         "HTTP/1.1 200 OK\r\n"
                                         "Content-Type: text/plain\r\n"
                                         "Access-Control-Allow-Origin: *\r\n"
                                         "\r\nOK");
                            }
                            else
                            {
                                // Страница не найдена
                                snprintf(http_response, sizeof(http_response),
                                         "HTTP/1.1 404 Not Found\r\n"
                                         "Content-Type: text/html\r\n"
                                         "\r\n<h1>404 Not Found</h1>");
                            }

                            // Отправка HTTP ответа
                            char send_cmd[64];
                            snprintf(send_cmd, sizeof(send_cmd),
                                     "AT+CIPSEND=%d,%d\r\n",
                                     client_id, strlen(http_response));
                            Send_AT_Command(send_cmd);

                            // Ждем приглашения ">"
                            if(Wait_AT_Response(">", 1000))
                            {
                                // Отправляем HTTP ответ
                                Send_AT_Command(http_response);

                                // Закрываем соединение
                                snprintf(send_cmd, sizeof(send_cmd),
                                         "AT+CIPCLOSE=%d\r\n", client_id);
                                Send_AT_Command(send_cmd);
                            }

                            // Очищаем буфер после обработки
                            memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
                        }
                    }
                }

                // Очистка буфера если он переполнен
                if(strlen((char*)esp_rx_buffer) > 400)
                {
                    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
                }
            }
        }
        else
        {
            // Если Wi-Fi неактивен, пробуем переинициализировать каждые 30 секунд
            static uint32_t last_retry = 0;
            if(osKernelGetTickCount() - last_retry > 30000)
            {
                ESP_Init();
                last_retry = osKernelGetTickCount();
            }
        }

        osDelay(50);
    }
}

/**
  * @brief Включение передачи RS485
  */
static void RS485_EnableTX(void)
{
  HAL_GPIO_WritePin(DE_RS_Out_GPIO_Port, DE_RS_Out_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RE_RS_Out_GPIO_Port, RE_RS_Out_Pin, GPIO_PIN_SET);
  osDelay(1); // Задержка для стабилизации
}

/**
  * @brief Включение приема RS485
  */
static void RS485_EnableRX(void)
{
  HAL_GPIO_WritePin(DE_RS_Out_GPIO_Port, DE_RS_Out_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE_RS_Out_GPIO_Port, RE_RS_Out_Pin, GPIO_PIN_RESET);
  osDelay(1); // Задержка для стабилизации
}

/**
  * @brief Расчет CRC16 для Modbus
  */
static uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;

  for(uint16_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for(uint8_t j = 0; j < 8; j++)
    {
      if(crc & 0x0001)
      {
        crc >>= 1;
        crc ^= MODBUS_CRC16_POLYNOMIAL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
  * @brief Формирование Modbus запроса
  */
static void Send_Modbus_Request(uint8_t slave_addr, uint8_t function_code,
                               uint16_t reg_addr, uint16_t reg_count)
{
  modbus_frame[0] = slave_addr;
  modbus_frame[1] = function_code;
  modbus_frame[2] = (reg_addr >> 8) & 0xFF;
  modbus_frame[3] = reg_addr & 0xFF;
  modbus_frame[4] = (reg_count >> 8) & 0xFF;
  modbus_frame[5] = reg_count & 0xFF;

  uint16_t crc = Modbus_CRC16(modbus_frame, 6);
  modbus_frame[6] = crc & 0xFF;
  modbus_frame[7] = (crc >> 8) & 0xFF;
}

/**
  * @brief Парсинг Modbus ответа
  */
static uint8_t Parse_Modbus_Response(uint8_t *response, uint16_t *temperature, uint16_t *humidity)
{
  // Проверка CRC
  uint16_t received_crc = (response[6] << 8) | response[5];
  uint16_t calculated_crc = Modbus_CRC16(response, 5);

  if(received_crc != calculated_crc)
    return 0;

  // Проверка адреса и функции
  if(response[0] != MODBUS_ADDRESS || response[1] != MODBUS_READ_HOLDING_REG)
    return 0;

  // Проверка длины данных
  if(response[2] != 2) // 2 байта на регистр
    return 0;

  // Извлечение данных
  if(temperature)
  {
    *temperature = (response[3] << 8) | response[4];
  }

  if(humidity)
  {
    *humidity = (response[3] << 8) | response[4];
  }

  return 1;
}

/**
  * @brief Обновление истории данных
  */
static void Update_History(SensorData data)
{
  osMutexAcquire(history_mutex, osWaitForever);

  // Сдвиг истории
  for(int i = HISTORY_SIZE - 1; i > 0; i--)
  {
    history_data[i] = history_data[i-1];
  }

  // Добавление новых данных
  history_data[0] = data;

  // Обновление индекса
  history_index = (history_index + 1) % HISTORY_SIZE;

  osMutexRelease(history_mutex);
}

/**
  * @brief Обновление светодиодов
  */
static void Update_LEDs(void)
{
  // Wi-Fi светодиод
  HAL_GPIO_WritePin(Led_WifI_Out_GPIO_Port, Led_WifI_Out_Pin,
		  wifi_ap_active ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // Аварийный светодиод
  uint8_t alarm = (humidifier_alarm ||
                  current_sensor_data.temperature < TEMP_MIN ||
                  current_sensor_data.temperature > TEMP_MAX ||
                  current_sensor_data.humidity < HUM_MIN ||
                  current_sensor_data.humidity > HUM_MAX);

  HAL_GPIO_WritePin(Led_Alarm_Out_GPIO_Port, Led_Alarm_Out_Pin,
                   alarm ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // Светодиод нормальной работы
  HAL_GPIO_WritePin(Led_Warning_Out_GPIO_Port, Led_Warning_Out_Pin,
                   (!alarm && heating_active) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief Обработка веб-команд
  */
static void Process_Web_Command(char *command)
{
  char *token;
  char *saveptr;

  token = strtok_r(command, "?&", &saveptr);
  while(token != NULL)
  {
    if(strstr(token, "mode="))
    {
      system_settings.auto_mode = atoi(token + 5);
    }
    else if(strstr(token, "heating="))
    {
      system_settings.heating_enabled = atoi(token + 8);
    }
    else if(strstr(token, "humidification="))
    {
      system_settings.humidification_enabled = atoi(token + 15);
    }
    else if(strstr(token, "heat_setpoint="))
    {
      system_settings.temperature_setpoint = atof(token + 14);
      system_settings.temperature_setpoint =
          CLAMP(system_settings.temperature_setpoint, TEMP_MIN, TEMP_MAX);
    }
    else if(strstr(token, "hum_setpoint="))
    {
      system_settings.humidity_setpoint = atof(token + 13);
      system_settings.humidity_setpoint =
          CLAMP(system_settings.humidity_setpoint, HUM_MIN, HUM_MAX);
    }
    else if(strstr(token, "date=") && strstr(token, "time="))
    {
      // Установка времени RTC
      // Парсинг даты и времени из команды
      // Реализация зависит от формата
    }

    token = strtok_r(NULL, "?&", &saveptr);
  }
}

/**
  * @brief Отправка AT команды
  */
static void Send_AT_Command(const char *cmd)
{
  HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 1000);
}

/**
  * @brief Ожидание AT ответа
  */
static uint8_t Wait_AT_Response(const char *expected, uint32_t timeout)
{
  uint32_t start_time = osKernelGetTickCount();
  uint16_t index = 0;

  while((osKernelGetTickCount() - start_time) < timeout)
  {
    if(HAL_UART_Receive(&huart6, &esp_rx_buffer[index], 1, 10) == HAL_OK)
    {
      index++;
      esp_rx_buffer[index] = '\0';

      if(strstr((char*)esp_rx_buffer, expected))
      {
        return 1;
      }

      if(index >= sizeof(esp_rx_buffer) - 1)
      {
        index = 0;
      }
    }
  }

  return 0;
}

/**
  * @brief Инициализация ESP модуля
  */
static void ESP_Init(void)
{
    uint8_t retry = 0;

    // 1. Аппаратный сброс
    HAL_GPIO_WritePin(RST_ESP_Out_GPIO_Port, RST_ESP_Out_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(RST_ESP_Out_GPIO_Port, RST_ESP_Out_Pin, GPIO_PIN_SET);
    osDelay(3000); // Увеличенная пауза для загрузки модуля

    // 2. Очистка буфера UART
    uint8_t dummy;
    while(HAL_UART_Receive(&huart6, &dummy, 1, 10) == HAL_OK);

    // 3. Проверка связи (попытки)
    for(retry = 0; retry < 5; retry++)
    {
        Send_AT_Command("AT\r\n");
        if(Wait_AT_Response("OK", 1000))
        {
            break;
        }
        osDelay(500);
    }

    if(retry >= 5)
    {
        // Ошибка связи с модулем
        wifi_ap_active = 0;
        return;
    }

    // 4. Сброс к заводским настройкам
    Send_AT_Command("AT+RESTORE\r\n");
    osDelay(2000);

    // Очистка буфера после сброса
    while(HAL_UART_Receive(&huart6, &dummy, 1, 10) == HAL_OK);

    // 5. Установка режима точки доступа (1=Station, 2=AP, 3=Both)
    Send_AT_Command("AT+CWMODE=2\r\n");
    if(!Wait_AT_Response("OK", 2000))
    {
        wifi_ap_active = 0;
        return;
    }

    // 6. Конфигурация точки доступа
    Send_AT_Command("AT+CWSAP=\"SVS_Kursov\",\"12345678\",1,3\r\n");
    if(!Wait_AT_Response("OK", 5000))
    {
        wifi_ap_active = 0;
        return;
    }

    // 7. Установка статического IP-адреса
    Send_AT_Command("AT+CIPAP=\"192.168.4.1\"\r\n");
    Wait_AT_Response("OK", 2000);

    // 8. Разрешение множественных подключений
    Send_AT_Command("AT+CIPMUX=1\r\n");
    Wait_AT_Response("OK", 2000);

    // 9. Запуск веб-сервера на порту 80
    Send_AT_Command("AT+CIPSERVER=1,80\r\n");
    if(Wait_AT_Response("OK", 2000))
    {
        wifi_ap_active = 1;

        // Дополнительные настройки
        Send_AT_Command("AT+CIPSTO=30\r\n"); // Таймаут соединения 30 секунд
        Wait_AT_Response("OK", 1000);

        // Включение автоматического принятия данных
        Send_AT_Command("AT+CIPDINFO=1\r\n");
        Wait_AT_Response("OK", 1000);

        // Получение информации о точке доступа
        Send_AT_Command("AT+CWSAP?\r\n");
        Wait_AT_Response("+CWSAP:", 2000);

        HAL_GPIO_WritePin(Led_WifI_Out_GPIO_Port, Led_WifI_Out_Pin, GPIO_PIN_SET);
    }
    else
    {
        wifi_ap_active = 0;
        HAL_GPIO_WritePin(Led_WifI_Out_GPIO_Port, Led_WifI_Out_Pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief Генерация HTML страницы
  */
static void Generate_HTML_Page(char *buffer, uint32_t size)
{
  char history_html[2048] = "";
  char date_str[11] = "2026-02-27";
  char time_str[6] = "14:30";
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  // Получение текущего времени
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  // Форматирование даты и времени
  sprintf(date_str, "%04d-%02d-%02d",
          sDate.Year + 2000, sDate.Month, sDate.Date);
  sprintf(time_str, "%02d:%02d", sTime.Hours, sTime.Minutes);

  // Генерация HTML истории
  Generate_History_HTML(history_html, sizeof(history_html));

  // Получение текущих данных и настроек
  osMutexAcquire(sensor_data_mutex, osWaitForever);
  float current_temp = current_sensor_data.temperature;
  float current_hum = current_sensor_data.humidity;
  osMutexRelease(sensor_data_mutex);

  osMutexAcquire(settings_mutex, osWaitForever);
  SystemSettings settings = system_settings;
  osMutexRelease(settings_mutex);

  // Формирование полной HTML страницы
  snprintf(buffer, size, html_page,
           current_temp, settings.temperature_setpoint,
           current_hum, settings.humidity_setpoint,
           settings.heating_enabled ? "Выключить обогрев" : "Включить обогрев",
           settings.humidification_enabled ? "Выключить увлажнение" : "Включить увлажнение",
           settings.auto_mode ? "#4fc3f7" : "#ff9800",
           settings.auto_mode ? "Автоматический" : "Ручной",
           history_html,
           settings.auto_mode ? "checked" : "",
           settings.auto_mode ? "active" : "",
           settings.temperature_setpoint, settings.temperature_setpoint,
           settings.auto_mode ? "active" : "",
           settings.humidity_setpoint, settings.humidity_setpoint,
           date_str, time_str,
		   wifi_ap_active ? "Wi-Fi подключен" : "Wi-Fi отключен",
           settings.temperature_setpoint, settings.humidity_setpoint,
           settings.heating_enabled ? "true" : "false",
           settings.humidification_enabled ? "true" : "false",
           settings.auto_mode ? "true" : "false");
}

/**
  * @brief Генерация JSON данных
  */
static void Generate_JSON_Data(char *buffer, uint32_t size)
{
  osMutexAcquire(sensor_data_mutex, osWaitForever);
  float current_temp = current_sensor_data.temperature;
  float current_hum = current_sensor_data.humidity;
  osMutexRelease(sensor_data_mutex);

  osMutexAcquire(settings_mutex, osWaitForever);
  SystemSettings settings = system_settings;
  osMutexRelease(settings_mutex);

  snprintf(buffer, size, json_data_template,
           current_temp, current_hum,
           heating_active, humidification_active,
           settings.temperature_setpoint, settings.humidity_setpoint,
           settings.auto_mode,
		   wifi_ap_active, humidifier_alarm,
           humidifier_running, humidifier_service);

  // Добавление HTTP заголовков
  char http_header[256];
  snprintf(http_header, sizeof(http_header),
           "HTTP/1.1 200 OK\r\n"
           "Content-Type: application/json\r\n"
           "Access-Control-Allow-Origin: *\r\n"
           "Content-Length: %d\r\n\r\n%s",
           strlen(buffer), buffer);

  strcpy(buffer, http_header);
}

/**
  * @brief Генерация HTML для истории
  */
static void Generate_History_HTML(char *buffer, uint32_t size)
{
  char temp[512] = "";
  osMutexAcquire(history_mutex, osWaitForever);

  for(int i = 0; i < HISTORY_SIZE; i++)
  {
    if(history_data[i].timestamp != 0)
    {
      uint32_t ts = history_data[i].timestamp;
      char row[128];

      // Форматирование даты и времени из timestamp
      snprintf(row, sizeof(row),
               "<tr><td>%04d-%02d-%02d %02d:%02d</td>"
               "<td>%.1f°C</td><td>%.1f%%</td></tr>",
               ts / 100000000, (ts % 100000000) / 1000000,
               (ts % 1000000) / 10000, (ts % 10000) / 100,
               ts % 100,
               history_data[i].temperature,
               history_data[i].humidity);

      strcat(temp, row);
    }
  }

  osMutexRelease(history_mutex);
  strncpy(buffer, temp, size);
}

/**
  * @brief Проверка состояния Wi-Fi
  */
static void Check_WiFi_Status(void)
{
    static uint32_t last_check = 0;
    uint32_t current_time = osKernelGetTickCount();

    if(current_time - last_check > 10000) // Проверка каждые 10 секунд
    {
        if(wifi_ap_active)
        {
            // Отправляем команду проверки клиентов
            Send_AT_Command("AT+CWLIF\r\n");
            Wait_AT_Response("+CWLIF:", 1000);
        }
        last_check = current_time;
    }
}

/**
  * @brief Callback обработки прерывания USART6
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART6)
  {
    // Обработка данных от ESP
    // Данные обрабатываются в потоке веб-интерфейса
  }
}

/**
  * @brief Callback обработки DMA
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART6)
  {
    // Передача данных на ESP завершена
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadRS485 */
/**
  * @brief  Function implementing the readRS485 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadRS485 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
