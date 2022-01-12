#ifndef __CONFIG_H_
#define __CONFIG_H_

#define PWM_A_PIN            PA8
#define PWM_B_PIN            PA9
#define PWM_C_PIN            PA10

#define DRIVER_EN_PIN        PB12

#define CURRENT_SENSE_A_PIN  PA0
#define CURRENT_SENSE_B_PIN  PA1


#define LED_PIN              PA15
#define LED_ON               digitalWrite(LED_PIN, LOW);
#define LED_OFF              digitalWrite(LED_PIN, HIGH);

#define RS485_EN_PIN         PB5
#define RS485_TX_PIN         PB6
#define RS485_RX_PIN         PB7
#define RS485_ON             digitalWrite(RS485_EN_PIN, HIGH);
#define RS485_OFF            digitalWrite(RS485_EN_PIN, LOW);

void initPorts() {
    pinMode(RS485_EN_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}





#endif // __CONFIG_H_
