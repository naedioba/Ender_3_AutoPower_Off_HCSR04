/*
Author:Mamadou BA Neige from MBA PRO TECH
Email: naedioba1@gmail.com   / mamadou.ba@ugb.edu.sn
Date: Latest version 19 November 2020
My YouTube Channel:https://www.youtube.com/channel/UCzC1VqRaFkubLkrMLdj2lmA

This is a modified version of my previous system available here: https://youtu.be/gNGvbSk3R0k
I'm using an infra-red proximity sensor as a detector when the print job is done. Then
after 20 to 30 seconds, the printer turns off autimatically.

===👇Le GüeroLoco 👇===
Fermeture automatique de l'imprimante à moins de 5$ !
https://youtu.be/1jNsF7NxjuU

*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x38, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // creaing the instance for the LCD display

int distance;                   // distance measured by the ultrasonic sensor
int distance_new;               // new distance measured by the ultrasonic sensor for confirmation
int distance_detection = 5;     // the printer is turned off if the distance measured by the ultrasonic sensor is inferior or equal to 5 centimeters
                                // the printer is turned on if the distance measured by the ultrasonic sensor is superiorto 5 centimeters
int relayEnder3Pro = 4;         // pin to which the relay controling the power is connected on the Arduino board
           
const int echoPin = 8;          // the ECHO pin of the ultrasonic sensor is connected to pin 8 of the Arduino board      
const int trigPin = 9;          // the TRIF pin of the ultrasonic sensor is connected to pin 9 of the Arduino board 

long duration;                  // duration of the pulse

unsigned long initialTime_E3Pro = 0;  	// initialization time
unsigned long previousMillisE3Pro = 0;
unsigned long currentMillisE3Pro = 0;       

unsigned long currentMillis;
long delayLCD = 3000;
long printer_TurnOn_Confirmation = 10000;   	// delay to confirm the state of the sensor to avoid untimely power on = 10 seconds
long printer_TurnOff_Confirmation = 1800000;  	// maximum delay to power off = 30 minutes = 1 800 000 milli secondes

//========= for the 3 LEDs (red, white, green) =========================
int LED_red = 10;                             // the red LED is connected to pin 10 of the Arduino board
int State_LED_red = LOW;                      // the red LED is initially set to LOW (turned off)
unsigned long previousMillis_LED_red = 0;     // will store the last time the red LED was updated
long OnTime_LED_red = 250;                    // milliseconds of on-time for the red LED
long OffTime_LED_red = 2000;                  // milliseconds of off-time for the red LED

int LED_white = 11;                           // the white LED is connected to pin 11 of the Arduino board
int State_LED_white = LOW;                    // the white LED is initially set to LOW (turned off)     
unsigned long previousMillis_LED_white = 0;   // will store the last time the white LED was updated
long OnTime_LED_white = 5000;                 // milliseconds of on-time for the white LED
long OffTime_LED_white = 250;                 // milliseconds of off-time for the white LED

int LED_green = 12;                           // the green LED is connected to pin 10 of the Arduino board
int State_LED_green = LOW;                    // the green LED is initially set to LOW (turned off)
unsigned long previousMillis_LED_green = 0;   // will store the last time the green LED was updated
long OnTime_LED_green = 300;                  // milliseconds of on-time for the green LED
long OffTime_LED_green = 1200;                // milliseconds of off-time for the green LED

 // creating the class controling the flashing of the 3 LEDs
class Flasher
{
    // Class Member Variables. 
    // These variables are initialized at startup
    int ledPin;           // the number of the LED pin
    long OnTime;          // milliseconds of on-time
    long OffTime;         // milliseconds of off-time
    
    // These variables maintain the current state
    int ledState;                   // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated
    
// Constructor: creates a Flasher and initializes the member variables and state
public:
    Flasher(int pin, long on, long off)
    {
        ledPin = pin;
        pinMode(ledPin, OUTPUT);
        OnTime = on;
        OffTime = off;
        ledState = LOW;
        previousMillis = 0;
    } // end Flasher
    void Update()
    {
        // check to see if it is time to change the state of the LED
        unsigned long currentMillis = millis();
        if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
        {
            ledState = LOW;                   // Turn it off
            previousMillis = currentMillis;   // Remember the time
            digitalWrite(ledPin, ledState);   // Update the actual LED
        }
        else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
        {
            ledState = HIGH;                  // turn it on
            previousMillis = currentMillis;   // Remember the time
            digitalWrite(ledPin, ledState);   // Update the actual LED
        }
    } // end void Update
}; // End of constructor

Flasher red_LED(LED_red, OnTime_LED_red, OffTime_LED_red);
Flasher white_LED(LED_white, OnTime_LED_white, OffTime_LED_white);
Flasher green_LED(LED_green, OnTime_LED_green, OffTime_LED_green);

//---------------------------------------------------------------
void setup() {

  Serial.begin(9600);                 // initializes the serial monitor at 9600 bauds
  lcd.begin(16,2);                    // initializes the 16x2 LCD screen
  pinMode(trigPin, OUTPUT);           // defines the TRIG pin of the ultrasonic sensor as OUTPUT
  pinMode(echoPin, INPUT);            // defines the ECHO pin of the ultrasonic sensor as INPUT

  pinMode(relayEnder3Pro,OUTPUT);     // defines the pin of the relay as OUTPUT

  digitalWrite(relayEnder3Pro, LOW);  // sets the initial state of the relay to LOW (tuned off)

} // end of setup

//---------------------------------------------------------------
void loop(){

  digitalWrite(trigPin, LOW);         // sets the TRIG pin to LOW
  delayMicroseconds(2);               // waits for 2 microseconds
  digitalWrite(trigPin, HIGH);        // sets the TRIG pin to HIGH
  delayMicroseconds(10);              // waits for 10 microseconds
  digitalWrite(trigPin, LOW);         // sets the TRIG pin to LOW
  
  duration = pulseIn(echoPin, HIGH);  // sets the ECHO pin to HIGH and reads the time put by the ultrasound to go back and forth
  distance= duration*0.034/2;         // calculate the distance
  
  Serial.print("Distance: ");         // prints "distance" on the serial monitor
  Serial.print(distance);             // prints the value of the distance on the serial monitor
  Serial.println(" cm");              // prints "cm" after the distance

// Displaying the distance and status of the printer on the LCD screen
  lcd.clear();           
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.setCursor(10, 0);
  lcd.print(distance);
  lcd.setCursor(14, 0);
  lcd.println("cm");  
  delay(1000);

  if(distance > distance_detection){  //01: if the distance measured is superior to the preset distance... 
    white_LED.Update();               // flashes the white LED
    green_LED.Update();               // flashes the green LED
    digitalWrite(LED_red, LOW);       // turns of the red LED
    
    currentMillisE3Pro = millis();    // remembers the time

    lcd.setCursor(0, 1);  
    lcd.println("Printer is ON   ");
    delay(1000);
    
    if(currentMillisE3Pro - previousMillisE3Pro >= printer_TurnOn_Confirmation){ //02: if the time is higher or equal to the confirmation time for turn on...
      distance_new = distance;                // checks again the distance measured by the ultrasonic sensor
      
      if(distance_new > distance_detection){  //03: if the new distance measured is superior to the preset distance...
        digitalWrite(relayEnder3Pro, HIGH);   // turns on the relay (printer turned on)
        Serial.println("Printer is ON");      // prints the status of the printer on the serial monitor
        
        lcd.setCursor(0, 1);  
        lcd.println("Printer is ON   ");      // displays the status of the printer on the second line of the LCD screen
        delay(1000);
    
        previousMillisE3Pro = currentMillisE3Pro;   // remembers the time
      } //03
    } //02
  }  //01
 
  if(distance <= distance_detection){ //04: if the distance measured is inferior or equal to the preset distance... 
    red_LED.Update();                 // flashes the red LED
    
    digitalWrite(LED_white, LOW);     // turns of the white LED
    digitalWrite(LED_green, LOW);     // turns of the green LED
    
    currentMillisE3Pro = millis();    // remembers the time

    lcd.setCursor(0, 1);  
    lcd.println("Printer is OFF  ");
    delay(1000);
    
    if(currentMillisE3Pro - previousMillisE3Pro >= printer_TurnOff_Confirmation){ //05: if the time is higher or equal to the confirmation time for turn off... 
      distance_new = distance;                      // checks again the distance measured by the ultrasonic sensor
      
      if(distance_new <= distance_detection){       //06: if the new distance measured is inferior or equal to the preset distance...
        digitalWrite(relayEnder3Pro, LOW);          // turns off the relay (printer turned off)
        Serial.println("Printer is OFF");           // prints the status of the printer on the serial monitor

        lcd.setCursor(0, 1);   
        lcd.println("Printer is OFF  ");            // displays the status of the printer on the second line of the LCD screen
        delay(1000);
        
        previousMillisE3Pro = currentMillisE3Pro;   // remembers the time
      } //06
    } //05
  } //04
} // end of loop
