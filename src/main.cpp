#include <Arduino.h>
#include <QuickPID.h>
#include <TFT_eSPI.h>
#include <ESP32Encoder.h>
#include "Base_imagen.h"
// =========================================================================

// Pines de Hardware (ESP32-S3)
const int pinPWM = 15; // Salida segura
const int pinADC = 4;  // ADC1 seguro (sin conflicto WiFi)
const int pinSW = 18;  // Botón encoder
const int pinEncCLK = 16;
const int pinEncDT = 17;

// Parámetros del Termistor NTC
const float R_REF = 100000.0; 
const float BETA = 3950.0;
const float TEMP_NOMINAL = 25.0;

// Filtro ADC
float emFilteredADC = -1.0;
const float EMA_ALPHA = 0.15; 

// Variables PID
float setpoint = 100.0;
float input = 0.0;
float output = 0.0;
float Kp = 0.0, Ki = 0.0, Kd = 0.0;

QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, 
               QuickPID::pMode::pOnError, 
               QuickPID::dMode::dOnMeas, 
               QuickPID::iAwMode::iAwCondition, 
               QuickPID::Action::direct);

// Interfaces
TFT_eSPI tft = TFT_eSPI(); // Lectura de pines
ESP32Encoder encoder;

// Estados UI
enum MenuState { SET_SP, SET_KP, SET_KI, SET_KD };
MenuState currentState = SET_SP;

// Temporización
unsigned long lastPIDTime = 0;
const unsigned long pidInterval = 100; // 10Hz

// Filtros de Botón
unsigned long lastEncoderActivity = 0; 
int buttonState = HIGH;
int lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 15;

// Variables de UI
long lastEncoderCount = 0;
float lastUIInput = -999.0, lastUIOutput = -999.0;
float lastUISetpoint = -999.0, lastUIKp = -1.0, lastUIKi = -1.0, lastUIKd = -1.0;
MenuState lastUIState = SET_KD;

void setupStaticUI() {
  tft.setAddrWindow(0, 0, 240, 320);
  tft.startWrite();
  for (int i = 0; i < 76800; i++) {
    unsigned long hexColor = epd_bitmap_Proyecto_nuevo[i];
    uint8_t r = (hexColor >> 16) & 0xFF;
    uint8_t g = (hexColor >> 8) & 0xFF;
    uint8_t b = hexColor & 0xFF;
    tft.pushColor(tft.color565(r, g, b));
  }
  tft.endWrite();
  
  tft.setTextColor(TFT_BLACK);
  tft.drawString("Control PID 12V", 10, 10, 4);
  tft.drawString("Temp Real:", 10, 50, 4);
  tft.drawString("PWM Out:", 10, 90, 4);
  tft.drawLine(10, 130, 230, 130, TFT_DARKGREY); 
  tft.drawString("T°:", 10, 150, 4);
  tft.drawString("Kp:", 10, 190, 4);
  tft.drawString("Ki:", 10, 230, 4);
  tft.drawString("Kd:", 10, 270, 4);
}

void updateDynamicUI() {
  tft.setTextPadding(120); 
  
  if (abs(input - lastUIInput) > 0.1) {
	if (input <= -99.9) {
		tft.setTextColor(TFT_RED, TFT_WHITE);
		tft.drawString("Er", 130, 50, 4);
	} else {
		tft.setTextColor(TFT_NAVY, TFT_WHITE);
		tft.drawFloat(input, 1, 130, 50, 4);
	}
    lastUIInput = input;
  }
  
  if (abs(output - lastUIOutput) > 0.5) {
    tft.setTextColor(TFT_MAROON, TFT_WHITE); 
    tft.drawFloat(output, 0, 130, 90, 4);
    lastUIOutput = output;
  }

  if (setpoint != lastUISetpoint || currentState != lastUIState) {
    tft.setTextColor(currentState == SET_SP ? TFT_DARKGREEN : TFT_BLACK, TFT_WHITE);
    tft.drawFloat(setpoint, 1, 80, 150, 4);
    lastUISetpoint = setpoint;
  }
  if (Kp != lastUIKp || currentState != lastUIState) {
    tft.setTextColor(currentState == SET_KP ? TFT_DARKGREEN : TFT_BLACK, TFT_WHITE);
    tft.drawFloat(Kp, 2, 80, 190, 4);
    lastUIKp = Kp;
  }
  if (Ki != lastUIKi || currentState != lastUIState) {
    tft.setTextColor(currentState == SET_KI ? TFT_DARKGREEN : TFT_BLACK, TFT_WHITE);
    tft.drawFloat(Ki, 2, 80, 230, 4);
    lastUIKi = Ki;
  }
  if (Kd != lastUIKd || currentState != lastUIState) {
    tft.setTextColor(currentState == SET_KD ? TFT_DARKGREEN : TFT_BLACK, TFT_WHITE);
    tft.drawFloat(Kd, 2, 80, 270, 4);
    lastUIKd = Kd;
  }
  
  lastUIState = currentState;
  tft.setTextPadding(0);
}

void setup() {
  analogReadResolution(12);
  pinMode(pinADC, INPUT);
  pinMode(pinSW, INPUT_PULLUP);
  
  encoder.attachHalfQuad(pinEncCLK, pinEncDT); 
  encoder.setCount(0);

  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(pinPWM, 5000, 8);
    ledcWrite(pinPWM, 0);
  #else
    ledcSetup(0, 5000, 8); 
    ledcAttachPin(pinPWM, 0);
    ledcWrite(0, 0);
  #endif

  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTimeUs(pidInterval * 1000);
  myPID.SetMode(QuickPID::Control::automatic);

  tft.init();
  tft.setRotation(0); 
  setupStaticUI();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Lectura de Encoder 
  long encCount = encoder.getCount();
  if (encCount != lastEncoderCount) {
    lastEncoderActivity = currentMillis; 
    float diff = (encCount - lastEncoderCount);
    
    switch (currentState) {
      case SET_SP: setpoint += diff * 1.0; break;
      case SET_KP: Kp += diff * 0.1; if(Kp < 0) Kp = 0; myPID.SetTunings(Kp, Ki, Kd); break;
      case SET_KI: Ki += diff * 0.05; if(Ki < 0) Ki = 0; myPID.SetTunings(Kp, Ki, Kd); break;
      case SET_KD: Kd += diff * 0.05; if(Kd < 0) Kd = 0; myPID.SetTunings(Kp, Ki, Kd); break;
    }
    lastEncoderCount = encCount;
  }

  // 2. Detección de Botón Encoder
  int currentReading = digitalRead(pinSW);
  if (currentReading != lastButtonReading) {
    lastDebounceTime = currentMillis;
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    if (currentReading != buttonState) {
      buttonState = currentReading;
      
      if (buttonState == LOW && (currentMillis - lastEncoderActivity > 300)) {
        currentState = static_cast<MenuState>((currentState + 1) % 4);
        encoder.setCount(0); 
        lastEncoderCount = 0;
      }
    }
  }
  lastButtonReading = currentReading;

  // 3. Adquisición Filtrada, Ecuación Beta y Control PID
  // No c que es esto :)
  if (currentMillis - lastPIDTime >= pidInterval) {
    
    long adcSum = 0;
    for (int i = 0; i < 16; i++) {
      adcSum += analogRead(pinADC);
    }
    float currentADC = adcSum / 16.0;

    if (emFilteredADC < 0) {
      emFilteredADC = currentADC;
    } else {
      emFilteredADC = (EMA_ALPHA * currentADC) + ((1.0 - EMA_ALPHA) * emFilteredADC);
    }

    if (emFilteredADC > 10.0 && emFilteredADC < 4085.0) { 
      float resistenciaNTC = R_REF * emFilteredADC / (4095.0 - emFilteredADC); 
      float temperaturaCelsius = (1.0 / (1.0 / (TEMP_NOMINAL + 273.15) + (1.0 / BETA) * log(resistenciaNTC / R_REF))) - 273.15;
      
      input = temperaturaCelsius;
    } else {
      input = -99.9; // Por si falla
    }

    if (input > -90.0) {
      myPID.Compute();
      
      #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ledcWrite(pinPWM, (uint32_t)output);
      #else
        ledcWrite(0, (uint32_t)output);
      #endif
      
    } else {
      #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ledcWrite(pinPWM, 0);
      #else
        ledcWrite(0, 0); 
      #endif
      output = 0;
    }

    updateDynamicUI();
    lastPIDTime = currentMillis;
  }
}