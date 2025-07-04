#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <stdint.h>

#include "Arduino.h"
#include "config.h"
#include "io_define.h"
#include "pin_funcs.h"
#include "util.h"
#include "ws2812.pio.h"
uint16_t adcReading[NUM_ANALOG_INPUTS];
bool first = true;
uint16_t adc(uint8_t pin) {
    adc_select_input(pin);
    return adc_read() << 4;
}

PIO ws2812Pio;
uint ws2812Sm;
uint ws2812Offset;
void putWs2812(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t w = 0;
#if WS2812W
    w = MIN(MIN(a, b), c);
    a -= w;
    b -= w;
    c -= w;
#endif
    pio_sm_put_blocking(ws2812Pio, ws2812Sm,
                        ((uint32_t)(a) << 24) |
                            ((uint32_t)(b) << 16) |
                            ((uint32_t)(c) << 8) |
                            ((uint32_t)(w)));
}

void initPins(void) {
    adc_init();
    PIN_INIT;
#ifdef WS2812_PIN
    pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &ws2812Pio, &ws2812Sm, &ws2812Offset, WS2812_PIN, 1, true);
    ws2812_program_init(ws2812Pio, ws2812Sm, ws2812Offset, WS2812_PIN, 800000, WS2812W);
#endif
}

uint8_t digital_read(uint8_t port, uint8_t mask) {
    port = port * 8;
    uint32_t mask32 = mask << port;
    for (uint i = 0; i < NUM_BANK0_GPIOS; i++) {
        if (mask32 & (1 << i)) {
            gpio_init(i);
            gpio_set_pulls(i, true, false);
        }
    }
    uint32_t ret = sio_hw->gpio_in;
    PIN_INIT;
    return ((ret >> port) & 0xff);
}

void digital_write(uint8_t port, uint8_t mask, uint8_t activeMask) {
    port = port * 8;
    uint32_t mask32 = mask << port;
    uint32_t activeMask32 = activeMask << port;
    for (uint i = 0; i < NUM_BANK0_GPIOS; i++) {
        if (mask32 & (1 << i)) {
            gpio_init(i);
            gpio_set_dir(i, true);
        }
    }
// If someone attempts to write to pin 25, assume they want to control the led and proxy
// the call to the cyw43
#if BLUETOOTH
    if (mask32 & 1 << 25) {
        cyw43_arch_gpio_put(0, activeMask32 & 1 << 25);
    }
#endif
    gpio_put_masked(mask32, activeMask32);
}

void set_led(bool val) {
    // TODO
    #if BLUETOOTH
        cyw43_arch_gpio_put(0, val); // for Pico W
    #else
        gpio_put_masked(1 << 25, ((uint32_t)val) << 25); // for Pico
    #endif
}

void toggle_led() {
    static bool state = true;
    set_led(state);
    state = !state;
}

uint16_t adc_read(uint8_t pin, uint8_t mask) {
    bool detecting = pin & (1 << 7);
    if (detecting) {
        pin = pin & ~(1 << 7);
        gpio_init(pin + PIN_A0);
        gpio_set_pulls(pin + PIN_A0, true, false);
        gpio_set_input_enabled(pin + PIN_A0, false);
    }
    adc_select_input(pin);
    uint16_t data = adc_read() << 4;
    if (detecting) {
        PIN_INIT;
    }
    return data;
}

uint16_t multiplexer_read(uint8_t pin, uint32_t mask, uint32_t bits) {
    if (!disable_multiplexer) {
        gpio_put_masked(mask, bits);
#ifdef CD4051BE
        sleep_us(50);
#endif
        adc_select_input(pin);
        return adc_read() << 4;
    }
    return 0;
}

uint8_t matrix_read(uint8_t pin, uint8_t outPin) {
    if (!disable_multiplexer) {
        gpio_put(outPin, 0);
        sleep_us(1);
        uint8_t ret = gpio_get(pin);
        gpio_put(outPin, 1);
        return ret;
    }
    return 1;
}

#define DECODE_JVC
#define DECODE_KASEIKYO
#define DECODE_PANASONIC    // alias for DECODE_KASEIKYO
#define DECODE_LG
#define DECODE_NEC          // Includes Apple and Onkyo
#define DECODE_SAMSUNG
#define DECODE_SONY
#define DECODE_RC5
#define DECODE_RC6
#define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width protocols
#define DECODE_HASH         // special decoder for all protocols

#define EXCLUDE_EXOTIC_PROTOCOLS

#include "IRremote.hpp"

#define IR_PIN 28
#define HOLD_TIME 100 // milliseconds
#define BLOCK_REPEAT_TIME 50 // milliseconds
#define LED_BLINK_PERIOD 1000 // milliseconds

#define ASOC(a,b) case a: set_pin(b); break;

enum MEDIA_REMOTE_BUTTONS {
    PLAY_IR_BTN = 0x16,
    STOP_IR_BTN = 0x19,
    PAUSE_IR_BTN = 0x18,
    REWIND_IR_BTN = 0x15,
    FAST_FORWARD_IR_BTN = 0x14,
    PREV_IR_BTN = 0x1B,
    NEXT_IR_BTN = 0x1A,
    DISPLAY_IR_BTN = 0x4F,
    TITLE_IR_BTN = 0x51,
    DVD_MENU_IR_BTN = 0x24,
    INFO_IR_BTN = 0xF,
    REC_IR_BTN = 0x17,

    LEFT_IR_BTN = 0x20,
    RIGHT_IR_BTN = 0x21,
    UP_IR_BTN = 0x1e,
    DOWN_IR_BTN = 0x1f,
    BACK_IR_BTN = 0x23,
    SELECT_IR_BTN = 0x22,
    A_IR_BTN = 0x66,
    B_IR_BTN = 0x25,
    X_IR_BTN = 0x68,
    Y_IR_BTN = 0x26,
};

enum PICO_BUTTON_PINS {
    PLACEHOLDER = 0,
    DPAD_LEFT_PIN = 1,
    DPAD_RIGHT_PIN,
    DPAD_UP_PIN,
    DPAD_DOWN_PIN,
    A_PIN,
    B_PIN,
    LEFT_TRIGGER,
    RIGHT_TRIGGER,
    BACK_PIN,
    OPTIONS_START,
    X_PIN,
    Y_PIN,
    XBOX_PIN,
};

enum STATE {
    IR_CONTROL_OFF,
    IR_TRANSLATION,
    IR_FULL_PASSTHROUGH,
};

uint32_t ir_pins = 0xFFFFFFFF;
unsigned long last_press[32] = {0}, last_LED = 0;
STATE state = IR_TRANSLATION;

void setup_IR() {
    IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
}

inline bool set_pin(int pin, int block_repeat_time=BLOCK_REPEAT_TIME) {
    unsigned long now = millis();
    if (last_press[pin] + block_repeat_time < now) {
        ir_pins &= ~((uint32_t)(1) << pin);
        last_press[pin] = now;
        return true;
    }
    return false;
}

void check_IR() {
    unsigned long now = millis();

    switch (state) {
        case IR_TRANSLATION:
            if (now - last_LED > LED_BLINK_PERIOD) {
                toggle_led();
                last_LED = now;
            }
            break;
        case IR_CONTROL_OFF:
            set_led(false);
            break;
        case IR_FULL_PASSTHROUGH:
            set_led(true);
            break;
        default:
            break;
    }

    for (uint8_t i = 0; i < 17; ++i) // idk how far to go
        if (last_press[i] + HOLD_TIME < now) {
            ir_pins |= (1 << i);
            if (i != PLACEHOLDER)
                last_press[i] = 0;
        }

    if (IrReceiver.decode()) {  // Check if the IR receiver has received a signal
        if (IrReceiver.decodedIRData.address == 0xF4 || IrReceiver.decodedIRData.address == 0x74) { // addresses used by the remote
            if (state != IR_CONTROL_OFF)
                switch (IrReceiver.decodedIRData.command)
                {
                    case PLAY_IR_BTN:
                        set_pin(A_PIN);
                        break;
                    case STOP_IR_BTN:
                        set_pin(B_PIN);
                        break;
                    case PREV_IR_BTN:
                        set_pin(DPAD_LEFT_PIN);
                        break;
                    case NEXT_IR_BTN:
                        set_pin(DPAD_RIGHT_PIN);
                        break;
                    case TITLE_IR_BTN:
                        set_pin(DPAD_UP_PIN);
                        break;
                    case INFO_IR_BTN:
                        set_pin(DPAD_DOWN_PIN);
                        break;
                    case REWIND_IR_BTN:
                        set_pin(LEFT_TRIGGER);
                        break;
                    case FAST_FORWARD_IR_BTN:
                        set_pin(RIGHT_TRIGGER);
                        break;
                    case DISPLAY_IR_BTN:
                        set_pin(X_PIN);
                        break;
                    case PAUSE_IR_BTN:
                        set_pin(Y_PIN);
                        break;
                    case DVD_MENU_IR_BTN:
                        set_pin(OPTIONS_START);
                        break; 
                    case LEFT_IR_BTN:
                    case RIGHT_IR_BTN:
                    case UP_IR_BTN:
                    case DOWN_IR_BTN:
                    case SELECT_IR_BTN:
                    case BACK_IR_BTN:
                    case A_IR_BTN:
                    case B_IR_BTN:
                    case X_IR_BTN:
                    case Y_IR_BTN:
                        if (state == IR_FULL_PASSTHROUGH)
                            switch (IrReceiver.decodedIRData.command)
                            {
                            case LEFT_IR_BTN:
                                set_pin(DPAD_LEFT_PIN);
                                break;
                            case RIGHT_IR_BTN:
                                set_pin(DPAD_RIGHT_PIN);
                                break;
                            case UP_IR_BTN:
                                set_pin(DPAD_UP_PIN);
                                break;
                            case DOWN_IR_BTN:
                                set_pin(DPAD_DOWN_PIN);
                                break;
                            case SELECT_IR_BTN:
                                set_pin(A_PIN);
                                break;
                            case BACK_IR_BTN:
                                set_pin(BACK_PIN);
                                break;

                            case A_IR_BTN:
                                set_pin(A_PIN);
                                break;
                            case B_IR_BTN:
                                set_pin(B_PIN);
                                break;
                            case X_IR_BTN:
                                set_pin(X_PIN);
                                break;
                            case Y_IR_BTN:
                                set_pin(Y_PIN);
                                break;
                            default:
                                break;
                            }
                    default:
                        break;
                }
            if (IrReceiver.decodedIRData.command == REC_IR_BTN &&
                set_pin(PLACEHOLDER, 500)) { // TODO come up with something better
                sleep_us(50);
                switch (state)
                {
                    case IR_CONTROL_OFF:
                        state = IR_TRANSLATION;
                        break;
                    case IR_TRANSLATION:
                        state = IR_FULL_PASSTHROUGH;
                        break;
                    case IR_FULL_PASSTHROUGH:
                        state = IR_CONTROL_OFF;
                        break;
                    default:
                        break;
                }
            }
        }
        IrReceiver.resume();  // Prepare the IR receiver to receive the next signal
    }
}

unsigned long sMillisOfFirstReceive;
bool sLongPressJustDetected;
/**
 * True once we received the consecutive repeats for more than aLongPressDurationMillis milliseconds.
 * The first frame, which is no repeat, is NOT counted for the duration!
 * @return true once after the repeated IR command was received for longer than aLongPressDurationMillis milliseconds, false otherwise.
 */
bool detectLongPress(uint16_t aLongPressDurationMillis) {
    if (!sLongPressJustDetected && (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
        /*
         * Here the repeat flag is set (which implies, that command is the same as the previous one)
         */
        if (millis() - aLongPressDurationMillis > sMillisOfFirstReceive) {
            sLongPressJustDetected = true; // Long press here
        }
    } else {
        // No repeat here
        sMillisOfFirstReceive = millis();
        sLongPressJustDetected = false;
    }
    return sLongPressJustDetected; // No long press here
}
