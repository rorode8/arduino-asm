
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

#include <Servo.h>
#include <LiquidCrystal.h>
Servo myservo;

//Assembly and arduino variables
volatile uint16_t myCounter asm("counter") __attribute__ ((used)) = 0;
volatile uint8_t segs asm("segs") __attribute__ ((used)) = 0;

extern "C" {
/*
 * function prototypes - the names declared here so we can
 * reference them in the code below without getting errors.
 */
void start();
void blink();
}


int lowerThreshold = 100;
int upperThreshold = 142;

//water pins
#define sensorPower 51
#define sensorPin A10

// Value for storing water level
int val = 0;

// Declare pins to which LEDs are connected
int redLED = 35;
int yellowLED = 33;
int greenLED = 31;



static const int DHT_SENSOR_PIN = 6;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
const int PBPIN = 38;
const int FANPIN = 39;
const int enableA = 5;  //A motor is in charge of humidity
const int pos = 48;
const int neg = 49;
const int posB = 45;  //B motor is in charge of temp
const int negB = 44;
const int enableB = 2;  
const int servo=3;
const int idealT = 30;
const int lowH = 40;
const int highH = 50;
const int cable = 39; //represents Kanthal cable

int cc=0;

/*
 * Initialize the serial port.
 */
void setup( )
{
  //water sensor
  pinMode(sensorPower, OUTPUT);
  digitalWrite(sensorPower, LOW);
  
  // Set LED pins as an OUTPUT
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // Initially turn off all LEDs
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  //main
  myservo.attach(servo);
  pinMode(cable,OUTPUT);
  pinMode(posB,OUTPUT);
  pinMode(negB,OUTPUT);
  pinMode(pos,OUTPUT);
  pinMode(neg,OUTPUT);
  pinMode(enableB,OUTPUT);
  
  pinMode(PBPIN,INPUT);
  pinMode(FANPIN,OUTPUT);
  lcd.begin(16, 2);
  Serial.begin( 9600);


  //arduino
  start();
  //all done
  Serial.println("Setup completed");
}

//water sensor reading function
int readSensor() {
  digitalWrite(sensorPower, HIGH);
  delay(10);
  val = analogRead(sensorPin);
  digitalWrite(sensorPower, LOW);
  return val;
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



/*
 * Main program loop.
 */
void loop( )
{
  float temperature;
  float humidity;

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  if(cc>4){
  int stepp =500;
  while(stepp>0 and measure_environment( &temperature, &humidity ) == false){
  Serial.println(stepp);
  
    
    

  
  
  stepp--;
  }
  Serial.print( "T = " );
    Serial.print( temperature, 1 );
    Serial.print( " deg. C, H = " );
    Serial.print( humidity, 1 );
    Serial.println( "%" );
    lcd.clear();
    
    int t = int(temperature);
    int h = int(humidity);
    String temp = "T = "+String(t)+" deg. C";
    String hum = "H = "+String(h)+"%";
    lcd.setCursor(0, 0);
    lcd.print(temp);
    lcd.setCursor(0, 1);
    lcd.print(hum);

    Serial.print("PB: ");
    Serial.println(digitalRead(PBPIN));
    digitalWrite(FANPIN,digitalRead(PBPIN));
    if(highH < humidity){
      
      myservo.write(90);
      //fan goes here
      digitalWrite(pos, HIGH);
      analogWrite(enableA,100);
      digitalWrite(neg, LOW);
      
    
    }else{
      myservo.write(0);
      //turn off fan
      digitalWrite(pos, LOW);
      analogWrite(enableA,0);
    }
    if(lowH > humidity){
      digitalWrite(cable,HIGH);
    }else{
      digitalWrite(cable,LOW);
    }
    
    if(temperature<idealT){
      digitalWrite(posB,HIGH);
      digitalWrite(negB,LOW);
      digitalWrite(enableB,HIGH);
    }else{
      //analogWrite(enableB,0);
      digitalWrite(posB,LOW);
      digitalWrite(enableB,LOW);
    }
  cc=0;
  }
  cc++;
  
  Serial.println(digitalRead(PBPIN));
  if(digitalRead(PBPIN) == 1){
      digitalWrite(25,LOW);
      segs = 0;
  }
  //water sensor
  int level = readSensor();
  Serial.print("Level: ");
  Serial.println(level);

  if (level <= lowerThreshold) {
    Serial.println("Water Level: Low");
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, LOW);
  }
  else if (level > lowerThreshold && level <= upperThreshold) {
    Serial.println("Water Level: Medium");
    digitalWrite(redLED, LOW);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  else if (level > upperThreshold) {
    Serial.println("Water Level: High");
    digitalWrite(redLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, HIGH);
  }
  //call arduino function
  Serial.println(myCounter);
  Serial.println(segs);
  blink();
  cc++;

  
    

  
  
  /*
   * 
   
  if(digitalRead(PBPIN) == 1){
    Serial.println("*");
    digitalWrite(FANPIN,HIGH);
  }
  */
  
}
