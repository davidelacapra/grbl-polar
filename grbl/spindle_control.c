/*
  spindle_control.c - spindle control methods
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

int32_t last_pwm;

/* 
   this rework of the spindle_control.c is aimed at obtaining a smoother movement of the servo motor. Smoothness is increased from the original version by:
   	- increasing the number of steps that divide 0° to 180°
	- moving the servo by stepping through all the possible steps
	- inserting a little pause (controlled by $29 parameter) before moving to the next step
   RC-Servo PWM modification: switch between 3.9% and 27.5% duty cycle  at 122Hz
   Prescaler 256 --> 16MHz / 256 / 255 / 2 = 122.549 Hz		look at https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM for frequency calculation
   PWM is controlled by TCNT2 in PWM phase correct mode (TOP: 0xFF)
*/

#ifdef RC_SERVO
  #define RC_SERVO_UP 10	// 10/255 = 3.9% duty cycle
  #define RC_SERVO_MIN 10	// 70/255 = 27.5% duty cycle
  #define RC_SERVO_MAX 70	// 70/255 = 27.5% duty cycle
  //#define RC_SERVO_INVERT     1     // Uncomment to invert servo direction
#endif

void spindle_init()
{    
  // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
  // combined unless configured otherwise.
  #ifdef VARIABLE_SPINDLE
    DDRB |= (1 << PB3);						// Configure PB3 (pin 11) as output pin
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);	// Configure Waveform Generation Mode: 1 (PWM, Phase Correct) and set the Compare Output Mode
    TCCR2B = (1 << CS22) | (1 << CS21);				// Set prescaler to 256
    OCR2A = RC_SERVO_UP;					// initialize the counter with the servo up position
    last_pwm = RC_SERVO_UP;					// update last pwm
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #endif     
  // Configure no variable spindle and only enable pin.
  #else  
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  #endif
  
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
  #endif
  spindle_stop();
}

#ifdef RC_SERVO
void spindle_stop()
{     // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
       #ifdef RC_SERVO_INVERT
    	  OCR2A = RC_SERVO_MAX;
    	  last_pwm = RC_SERVO_MAX;
      #else
    	  OCR2A = RC_SERVO_UP;
    	  last_pwm = RC_SERVO_UP;
      #endif
}

#else
void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      #ifdef INVERT_SPINDLE_ENABLE_PIN
		SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
	  #else
		SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
	  #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
	  SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
	#else
	  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
	#endif
  #endif
}
#endif

#ifdef RC_SERVO
void spindle_run(uint8_t direction, float rpm)
{
    int32_t actual_servo_delay;
    int32_t original_servo_delay;
    original_servo_delay = settings.servo_delay;
    actual_servo_delay = 1000 * original_servo_delay;
	
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  protocol_buffer_synchronize();

  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {
	#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN

    if (direction == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }
	#endif


      #ifdef RC_SERVO_INVERT
          //current_pwm = floor( RC_SERVO_LONG - rpm*(RC_SERVO_RANGE/SPINDLE_RPM_RANGE));
          //OCR_REGISTER = current_pwm;
      #else
        uint16_t rpm_tmp;

        rpm = min(max(rpm, RC_SERVO_MIN), RC_SERVO_MAX); // ensure that rpm does not exceeds min and max boundaries

	 // desired rpm is reached through steps and delay, in this way the movement get way smoother
        if (attuale_pwm > rpm) {
            for (rpm_tmp = last_pwm; rpm_tmp > rpm; rpm_tmp--) {
                OCR2A = rpm_tmp;					//overwrite timer 
                delay_us(actual_servo_delay);				// wait
            }        
        }
        else {
            for (rpm_tmp = last_pwm; rpm_tmp < rpm; rpm_tmp++) {
                OCR2A = rpm_tmp;					//overwrite timer
                delay_us(actual_servo_delay);				// wait
            }
        }
        last_pwm = rpm;
	  
	  
      #endif
	  #ifdef MINIMUM_SPINDLE_PWM
        if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
	     OCR_REGISTER = current_pwm;
      #endif
    #endif
  }
}
#else
void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state, rpm);
}
#endif




#ifdef RC_SERVO
void spindle_set_state(uint8_t state, float rpm){
}
#endif





