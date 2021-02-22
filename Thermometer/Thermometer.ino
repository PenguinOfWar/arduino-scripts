//www.elegoo.com
//2016.12.9

#include <LiquidCrystal.h>
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11
int tempPin = 0;

// i'm gonna customise this to add some new features
// starting with status pins demonstrating temperature

int blueLedPin = 2;
int redLedPin = 3;

// lets add a button

int buttonPin = 4;

//  temp and humidity sensor
static const int DHT_SENSOR_PIN = 5;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//                BS  E  D4 D5  D6 D7
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
void setup()
{
  //  setup our pins for output
  pinMode(blueLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  //  start the lcd
  lcd.begin(16, 2);
  /*
   * Initialize the serial port.
   */

   Serial.begin( 9600);
}

/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available.
 */
static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}

void loop()
{
  int tempReading = analogRead(tempPin);
  // This is OK
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  float tempR = tempF + 459.67; // Convert Fahrenheit to Rankine

  //  lets handle our button press
  //  if the button is pushed it will send the signal low
  //  we should show freedom units instead in this case

  if(digitalRead(buttonPin) == LOW) {
    // Display Temperature in F
    lcd.setCursor(0, 0);
    lcd.print("Temp         F  ");
    lcd.setCursor(6, 0);
    lcd.print(tempF);
  
    lcd.setCursor(0, 1);
    lcd.print("Temp         R  ");
    lcd.setCursor(6, 1);
    lcd.print(tempR);
  } else {
    // Display Temperature in C
    lcd.setCursor(0, 0);
    lcd.print("Temp         C  ");
    lcd.setCursor(6, 0);
    lcd.print(tempC);
  
    lcd.setCursor(0, 1);
    lcd.print("Temp         K  ");
    lcd.setCursor(6, 1);
    lcd.print(tempK);
  }

  // now we will set the pins with an if

  if (tempC > 20) {
    //    it's hot!
    digitalWrite(redLedPin, HIGH);
    digitalWrite(blueLedPin, LOW);
  } else {
    //    it's cold!
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, HIGH);
  }

  float temperature;
  float humidity;

  Serial.print("Hi\n");
  Serial.print(temperature, 1);
   Serial.print( humidity, 1 );

  if( measure_environment( &temperature, &humidity ) == true )
  {
    Serial.print( "T = " );
    Serial.print( temperature, 1 );
    Serial.print( " deg. C, H = " );
    Serial.print( humidity, 1 );
    Serial.println( "%" );
  }
  
  delay(500);
}
