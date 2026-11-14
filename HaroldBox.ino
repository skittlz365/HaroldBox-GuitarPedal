#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/i2s.h>
// ==========================================
// 1. I2S & HARDWARE CONFIGURATION
// ==========================================
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 128 // Number of samples per block
// Pin definitions for standard external I2S ADC/DAC connections
#define I2S_SCK 26 // Serial Clock (BCLK)
#define I2S_WS 25 // Word Select / LRCK
#define I2S_SD_IN 35 // Serial Data IN (from external ADC)
#define I2S_SD_OUT 22 // Serial Data OUT (to external DAC)
// Physical Bypass Toggle Button Pin (Connect button between GPIO 4 and GND)
#define EFFECT_BUTTON_PIN 4
// Global state for effect bypass toggle
volatile bool effectEnabled = true;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
// ==========================================
// 2. FAUST INTERFACE & BINDER STUBS
// ==========================================
#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif
struct UI {
virtual void openTabBox(const char* label) {}
virtual void openHorizontalBox(const char* label) {}
virtual void openVerticalBox(const char* label) {}
virtual void closeBox() {}
virtual void addButton(const char* label, float* zone) {}
virtual void addCheckButton(const char* label, float* zone) {}
virtual void addVerticalSlider(const char* label, float* zone, float init, float min, float max, float step) {}
virtual void addHorizontalSlider(const char* label, float* zone, float init, float min, float max, float step) {}
virtual void addNumEntry(const char* label, float* zone, float init, float min, float max, float step) {}
virtual void addHorizontalBarGraph(const char* label, float* zone, float min, float max) {}
virtual void addVerticalBarGraph(const char* label, float* zone, float min, float max) {}
virtual void declare(float* zone, const char* key, const char* value) {}
virtual ~UI() {}
};
class dsp {
public:
dsp() {}
virtual ~dsp() {}
virtual int getNumInputs() = 0;
virtual int getNumOutputs() = 0;
virtual void init(int sample_rate) = 0;
virtual void compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) = 0;
virtual void buildUserInterface(UI* ui_interface) = 0;
};
class PedalUI : public UI {
public:
FAUSTFLOAT* driveZone = nullptr;
FAUSTFLOAT* toneZone = nullptr;
FAUSTFLOAT* wahZone = nullptr;
void addHorizontalSlider(const char* label, float* zone, float init, float min, float max, float step) override {
if (strcmp(label, "Drive") == 0) {
driveZone = zone;
} else if (strcmp(label, "Tone") == 0) {
toneZone = zone;
} else if (strcmp(label, "Wah") == 0) {
wahZone = zone;
}
}
};
// ==========================================
// 3. FAUST DSP CLASS (mydsp) WITH AUTO-WAH
// ==========================================
#ifndef __mydsp_H__
#define __mydsp_H__
#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <math.h>
#ifndef FAUST_INT_WRAP
#define FAUST_INT_WRAP
inline int faust_wrap_add(int a, int b) { return int((unsigned int)a + (unsigned int)b); }
inline int faust_wrap_sub(int a, int b) { return int((unsigned int)a - (unsigned int)b); }
inline int faust_wrap_mul(int a, int b) { return int((unsigned int)a * (unsigned int)b); }
#endif
#ifndef FAUSTCLASS
#define FAUSTCLASS mydsp
#endif
#ifdef __APPLE__
#define exp10f __exp10f
#define exp10 __exp10
#endif
class mydsp : public dsp {
private:
FAUSTFLOAT fHslider0; // Wah / Sensitivity
FAUSTFLOAT fHslider1; // Drive
FAUSTFLOAT fHslider2; // Tone
int fSampleRate;
float fConst0;
float fRec1[3];
float fVec0[2];
float fRec0[2];
float fEnvelope; // Envelope follower state for Auto-Wah
public:
mydsp() {}
virtual ~mydsp() = default;
virtual int getNumInputs() { return 1; }
virtual int getNumOutputs() { return 1; }
virtual void init(int sample_rate) {
fSampleRate = sample_rate;
fConst0 = 3.1415927f / std::min<float>(1.92e+05f, std::max<float>(1.0f, static_cast<float>(fSampleRate)));
fHslider0 = static_cast<FAUSTFLOAT>(0.0f); // Wah
fHslider1 = static_cast<FAUSTFLOAT>(0.0f); // Drive
fHslider2 = static_cast<FAUSTFLOAT>(0.0f); // Tone
fEnvelope = 0.0f;
for (int l0 = 0; l0 < 3; l0++) fRec1[l0] = 0.0f;
for (int l1 = 0; l1 < 2; l1++) fVec0[l1] = 0.0f;
for (int l2 = 0; l2 < 2; l2++) fRec0[l2] = 0.0f;
}
virtual void buildUserInterface(UI* ui_interface) {
ui_interface->openVerticalBox("Effects");
ui_interface->addHorizontalSlider("Drive", &fHslider1, FAUSTFLOAT(0.0f), FAUSTFLOAT(0.0f), FAUSTFLOAT(1.0f), FAUSTFLOAT(0.01f));
ui_interface->addHorizontalSlider("Tone", &fHslider2, FAUSTFLOAT(0.0f), FAUSTFLOAT(0.0f), FAUSTFLOAT(1.0f), FAUSTFLOAT(0.01f));
ui_interface->addHorizontalSlider("Wah", &fHslider0, FAUSTFLOAT(0.0f), FAUSTFLOAT(0.0f), FAUSTFLOAT(1.0f), FAUSTFLOAT(0.01f));
ui_interface->closeBox();
}
virtual void compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) {
FAUSTFLOAT* input0 = inputs[0];
FAUSTFLOAT* output0 = outputs[0];
float driveVal = static_cast<float>(fHslider1);
float wahSens = static_cast<float>(fHslider0);
float toneVal = static_cast<float>(fHslider2);
float fSlow8 = 15.0f * driveVal + 1.0f;
// Stable tone coefficients calculated per audio block
float tanArgTone = fConst0 * (9.5e+03f * toneVal + 5e+02f);
float fSlow10 = 1.0f / std::tan(tanArgTone);
float fSlow11 = 1.0f - fSlow10;
float fSlow12 = 1.0f / (fSlow10 + 1.0f);
// Envelope tracking attack/release coefficients
float attackCoeff = 0.02f;
float releaseCoeff = 0.0005f;
for (int i0 = 0; i0 < count; i0++) {
float in_val = static_cast<float>(input0[i0]);
if (isnan(in_val) || isinf(in_val)) in_val = 0.0f;
// 1. Envelope Follower tracking plucking dynamics
float inputAbs = std::abs(in_val);
if (inputAbs > fEnvelope) {
fEnvelope += attackCoeff * (inputAbs - fEnvelope);
} else {
fEnvelope += releaseCoeff * (inputAbs - fEnvelope);
}
// 2. Dynamic filter cutoff driven by envelope + sensitivity slider
float dynamicWah = std::min(1.0f, std::max(0.0f, wahSens * fEnvelope * 6.0f));
float fSlow1 = std::tan(fConst0 * (2.2e+03f * dynamicWah + 2.5e+02f));
float fSlow2 = 2.0f * (1.0f - 1.0f / (fSlow1 * fSlow1));
float fSlow3 = 1.0f / (5.0f * dynamicWah + 2.0f);
float fSlow4 = 1.0f / fSlow1;
float fSlow5 = (fSlow4 - fSlow3) / fSlow1 + 1.0f;
float fSlow6 = (fSlow3 + fSlow4) / fSlow1 + 1.0f;
float fSlow7 = 1.0f / fSlow6;
float fSlow9 = 1.0f / (fSlow1 * fSlow6);
// 3. Drive & Resonant Filter Stage
fRec1[0] = std::tanh(fSlow8 * in_val) - fSlow7 * (fSlow5 * fRec1[2] + fSlow2 * fRec1[1]);
float fTemp0 = fSlow9 * (fRec1[0] - fRec1[2]);
fVec0[0] = fTemp0;
// 4. Tone Stage
fRec0[0] = -(fSlow12 * (fSlow11 * fRec0[1] - (fTemp0 + fVec0[1])));
float out_val = fRec0[0];
if (isnan(out_val) || isinf(out_val)) {
out_val = 0.0f;
for(int k=0; k<3; k++) fRec1[k] = 0.0f;
for(int k=0; k<2; k++) { fVec0[k] = 0.0f; fRec0[k] = 0.0f; }
}
output0[i0] = static_cast<FAUSTFLOAT>(out_val);
fRec1[2] = fRec1[1];
fRec1[1] = fRec1[0];
fVec0[1] = fVec0[0];
fRec0[1] = fRec0[0];
}
}
};
#endif // Close __mydsp_H__
// ==========================================
// 4. BLUETOOTH CONFIGURATION & CALLBACKS
// ==========================================
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
mydsp audioEffect;
PedalUI pedalUI;
class MyCallbacks : public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic* pCharacteristic) override {
String value = pCharacteristic->getValue();
if (value.length() > 0) {
String command = value;
if (command.startsWith("DRIVE:") || command.startsWith("GAIN:")) {
int colonIdx = command.indexOf(':');
float val = command.substring(colonIdx + 1).toFloat();
if (pedalUI.driveZone) (pedalUI.driveZone) = constrain(val, 0.0f, 1.0f);
}
else if (command.startsWith("WAH:")) {
float val = command.substring(4).toFloat();
if (pedalUI.wahZone) (pedalUI.wahZone) = constrain(val, 0.0f, 1.0f);
}
else if (command.startsWith("TONE:")) {
float val = command.substring(5).toFloat();
if (pedalUI.toneZone) (pedalUI.toneZone) = constrain(val, 0.0f, 1.0f);
}
}
}
};
// ==========================================
// 5. I2S AUDIO TASK (FreeRTOS)
// ==========================================
void audioTask(void* parameter) {
int32_t i2s_buffer[BUFFER_SIZE * 2]; // Stereo interleaved buffer (Left + Right)
float input_buffer[BUFFER_SIZE];
float output_buffer[BUFFER_SIZE];
FAUSTFLOAT* inputs[1] = { input_buffer };
FAUSTFLOAT* outputs[1] = { output_buffer };
size_t bytes_read = 0;
size_t bytes_written = 0;
while (true) {
// Read 32-bit audio frame from external ADC via I2S
i2s_read(I2S_PORT, (void)i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
int samples_read = bytes_read / sizeof(int32_t) / 2;
if (samples_read <= 0) continue;
for (int i = 0; i < samples_read; i++) {
int32_t raw_in = i2s_buffer[i * 2 + 1] >> 8;
float sample = (float)raw_in / 8388608.0f; // 2^23 scale
if (isnan(sample) || isinf(sample)) sample = 0.0f;
input_buffer[i] = sample;
}
// Run Faust DSP effect chain or bypass
if (effectEnabled) {
audioEffect.compute(samples_read, inputs, outputs);
} else {
// True bypass: copy input directly to output buffer
for (int i = 0; i < samples_read; i++) {
output_buffer[i] = input_buffer[i];
}
}
for (int i = 0; i < samples_read; i++) {
float out = output_buffer[i];
if (isnan(out) || isinf(out)) {
out = 0.0f;
} else if (out > 0.98f) {
out = 0.98f;
} else if (out < -0.98f) {
out = -0.98f;
}
int32_t raw_out = (int32_t)(out * 8388607.0f) << 8;
i2s_buffer[i * 2] = raw_out; // Left channel output
i2s_buffer[i * 2 + 1] = raw_out; // Right channel output
}
// Write processed audio frame out to external DAC via I2S
i2s_write(I2S_PORT, (void)i2s_buffer, samples_read 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);
}
}
// ==========================================
// 6. SETUP & MAIN LOOP
// ==========================================
void setup() {
Serial.begin(115200);
delay(1000);
// Configure physical button pin with internal pull-up resistor
pinMode(EFFECT_BUTTON_PIN, INPUT_PULLUP);
// Initialize Faust DSP and bind slider parameters
audioEffect.init(SAMPLE_RATE);
audioEffect.buildUserInterface(&pedalUI);
// Configure I2S Driver for External ADC/DAC (Full-Duplex Mode)
i2s_config_t i2s_config = {
.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
.sample_rate = SAMPLE_RATE,
.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
.communication_format = I2S_COMM_FORMAT_STAND_I2S,
.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
.dma_buf_count = 4,
.dma_buf_len = BUFFER_SIZE,
.use_apll = true,
.tx_desc_auto_clear = true,
.fixed_mclk = 0
};
i2s_pin_config_t pin_config = {
.mck_io_num = 0, // Master Clock output on GPIO 0
.bck_io_num = I2S_SCK, // Pin 26 (Bit Clock)
.ws_io_num = I2S_WS, // Pin 25 (Word Select)
.data_out_num = I2S_SD_OUT, // Pin 22 (To DAC)
.data_in_num = I2S_SD_IN // Pin 35 (From ADC)
};
i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
i2s_set_pin(I2S_PORT, &pin_config);
i2s_zero_dma_buffer(I2S_PORT);
// Explicitly start the I2S driver
i2s_start(I2S_PORT);
// Initialize BLE Server
BLEDevice::init("HackathonPedal");
BLEServer* pServer = BLEDevice::createServer();
BLEService* pService = pServer->createService(SERVICE_UUID);
BLECharacteristic* pCharacteristic = pService->createCharacteristic(
CHARACTERISTIC_UUID,
BLECharacteristic::PROPERTY_READ |
BLECharacteristic::PROPERTY_WRITE
);
pCharacteristic->setCallbacks(new MyCallbacks());
pService->start();
BLEAdvertising* pAdvertising = pServer->getAdvertising();
pAdvertising->addServiceUUID(SERVICE_UUID);
pAdvertising->setScanResponse(true);
pAdvertising->start();
// Start audio task on Core 1
xTaskCreatePinnedToCore(
audioTask,
"AudioTask",
4096,
NULL,
3,
NULL,
1
);
Serial.println("ESP32 External ADC/DAC Pedal (Drive + Tone + Auto-Wah + Bypass Button) Running cleanly!");
}
void loop() {
// Non-blocking button polling with software debounce
int reading = digitalRead(EFFECT_BUTTON_PIN);
if (reading != lastButtonState) {
lastDebounceTime = millis();
}
if ((millis() - lastDebounceTime) > debounceDelay) {
if (reading == LOW && lastButtonState == HIGH) { // Button pressed (falling edge)
effectEnabled = !effectEnabled;
Serial.print("Pedal State: ");
Serial.println(effectEnabled ? "EFFECTS ACTIVE" : "TRUE BYPASS");
}
}
lastButtonState = reading;
vTaskDelay(50 / portTICK_PERIOD_MS); // poll every 50ms
}
