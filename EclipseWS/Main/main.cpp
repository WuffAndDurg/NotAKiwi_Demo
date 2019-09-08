
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "XasLibs/TIMER/Timer1.h"

#include "XasLibs/Communication/ESPUart/ESPUART.h"

#include "XasLibs/Actuators/Steppers/OmniwheelStepper.h"
#include "XasLibs/Movement/X3/X3-Locomotor.h"

using namespace X3;

OmniwheelStepper stepA = OmniwheelStepper(&PORTD, 3, 2, 102, 32, 35, 45, 140);
OmniwheelStepper stepB = OmniwheelStepper(&PORTD, 4, 2, 102, 32, 35, 135, 140);
OmniwheelStepper stepC = OmniwheelStepper(&PORTD, 5, 2, 102, 32, 35, 225, 140);
OmniwheelStepper stepD = OmniwheelStepper(&PORTD, 6, 2, 102, 32, 35, 315, 140);

Locomotor motor = Locomotor(50);

struct MotorData {
	int8_t status;
	float   rotation;
	float   speed;
};
MotorData motorData[4];
ESPComs::Source  motorSource = ESPComs::Source(10, &motorData, sizeof(motorData));

void setMotorPower() {
	if(ESPComs::Endpoint::pubBuffer[0] == '1')
		PORTB &= ~(1<<PB3);
	else
		PORTB |=  (1<<PB3);
}
ESPComs::Endpoint motorToggle = ESPComs::Endpoint(12, &ESPComs::Endpoint::pubBuffer, 1, &setMotorPower);

struct PositionData {
	float xPos;
	float yPos;
	float rPos;
} currentPosition;
ESPComs::Source positionSource = ESPComs::Source(11, &currentPosition, sizeof(currentPosition));

void updatePositionData() {
	currentPosition.xPos = motor.getX();
	currentPosition.yPos = motor.getY();
	currentPosition.rPos = motor.getR();

	positionSource.fire();
}

struct JoystickData {
	int16_t x;
	int16_t y;
	int16_t r;
};
volatile JoystickData joystickData;
ESPComs::Endpoint joystickControl = ESPComs::Endpoint(13, (void *)&joystickData, sizeof(joystickData), 0);

void updateMotorData() {
	int8_t status = ((PORTB && (1<<PB3)) == 0 ) ? 0 : 1;

	motorData[0].status 	= status;
	motorData[0].rotation	= stepA.getPosition() * 360 / (200*32);
	motorData[0].speed		= stepA.getSpeed() * 5000 / (200*32);

	motorData[1].status 	= status;
	motorData[1].rotation	= stepB.getPosition() * 360 / (200*32);
	motorData[1].speed		= stepB.getSpeed() * 5000 / (200*32);

	motorData[2].status 	= status;
	motorData[2].rotation	= stepC.getPosition() * 360 / (200*32);
	motorData[2].speed		= stepC.getSpeed() * 5000 / (200*32);

	motorData[3].status 	= status;
	motorData[3].rotation	= stepD.getPosition() * 360 / (200*32);
	motorData[3].speed		= stepD.getSpeed() * 5000 / (200*32);

	motorSource.fire();
}

uint8_t 	update_presc_a = 1;
uint16_t 	update_presc_b = 20000;
uint16_t	update_presc_c = 20000;
ISR(TIMER1_COMPA_vect) {
	stepA.update();
	stepB.update();
	stepC.update();
	stepD.update();

	ESPComs::increaseTimeout(5);

	if(--update_presc_a == 0) {
		motor.update();
		update_presc_a = 100;
	}
	if(--update_presc_b == 0) {
		updateMotorData();
		update_presc_b = 5000 / 10;
	}
	if(--update_presc_c == 0) {
		updatePositionData();
		update_presc_c = 2000;
	}
}

int main() {
	DDRB  |= (1<<PB3);
	//PORTB |= (1<<PB3);

	DDRD  |= (1<<PD1);

	Timer1::enable_CTC(5000);

	_delay_ms(1500);
	ESPComs::init();

	sei();

	motor.setSpeed(100);
	motor.setRotationSpeed(30);
	motor.setAcceleration(10000);

	while(1) {
		_delay_ms(1000);

		motor.rotateBy(30);
		motor.flush();
		motor.rotateBy(-30);
		motor.flush();

		_delay_ms(300);
		motor.moveBy(100, 100);
		motor.flush();
		motor.moveTo(0, 0);
		motor.flush();
		motor.moveBy(100, -100);
		motor.flush();
		motor.moveTo(0, 0);
		motor.flush();

		_delay_ms(300);
		for(uint16_t i=0; i<360; i++) {
			motor.moveTowards(3, i);
			_delay_ms(10);
		}
		motor.flush();
	}

	return 1;
}
