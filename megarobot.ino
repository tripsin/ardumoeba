#include <IRremote.h>
#include <Servo.h>

//motor shield M2
#define IN1 7
#define IN2 8
#define E1 6
//motor shield M1
#define E2 5
#define IN3 2
#define IN4 4

#define BUZZER 9

// IR remote
#define TSOP 10
// philips
#define IR_LEFT       90
#define IR_RIGHT      91
#define IR_UP         88
#define IR_DOWN       89
#define IR_OK         92
#define IR_PLAY_PAUSE 44
#define IR_1          1 // line tracking
#define IR_2          2 // ir remote
#define IR_3          3
#define IR_4          4
#define IR_5          5
#define IR_6          6
#define IR_TIMEOUT  100
IRrecv irrecv(TSOP);

//ultrasound sensor
#define TRIG 13
#define ECHO 12
#define SERVO 3
#define CENTER 105

Servo hServ;

int speedWeel[11] = {255, 111, 127, 143, 159, 175, 191, 201, 223, 239, 255};

void fwd(int l,int r)
{
  //left motor
  if (l > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (l < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (l == 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(E1, speedWeel[abs(l)]);
  
  //right motor
  if (r > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (r < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (r == 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(E2, speedWeel[abs(r)]);

  //Крутит башкой
  if (l < r) {
    hServ.write(CENTER + 30);
  } else if (l > r) {
    hServ.write(CENTER - 30);
  } else hServ.write(CENTER);
}

void stp()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


void beep(int ton)
{
    analogWrite(BUZZER, ton);
    delay(50);
    analogWrite(BUZZER, 0);
}

void setup() {
  //line tracker init
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  //TSOP (IR) init
  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(false); // отключить мигание светодиода (13) при приеме

  //servo init
  hServ.attach(SERVO);
  hServ.write(CENTER);

  //motor init
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  stp();

  // HC-SR04 init
  pinMode(TRIG, OUTPUT); //инициируем как выход 
  pinMode(ECHO, INPUT); //инициируем как вход 

  //buzzer init
  pinMode(BUZZER, OUTPUT);
  //beep(127);
  Serial.begin(9600);
}

int readLine()
{
  static int lastResult;
  int result;  
  //Reading from line-sensor
  byte newLine = (PINC & B00111110) >> 1;
  switch (newLine){
    case B10000: {result = 5; break;}
    case B11000: {result = 4; break;}
    case B11110: {result = 3; break;}    
    case B11100: {result = 2; break;}
    case B01100: {result = 1; break;}
    case B00100: {result = 0; break;}
    case B00110: {result = -1; break;}
    case B00111: {result = -2; break;}
    case B01111: {result = -3; break;}
    case B00011: {result = -4; break;}
    case B00001: {result = -5; break;}
    case B00000: {
        if (lastResult > 0) {
          result = 6; 
        } else {
          result = -6; 
        }
        break;
      }
    default: {result = lastResult; break;}
  }
  lastResult = result; 
  return result; 
}

void mode_linetracking()
{
static byte lastLine;
byte newLine = 0;
  
  //Reading from line-sensor
  newLine = (PINC & B00111110) >> 1;
  if (newLine == lastLine) return;
  
  switch (newLine) 
  {
    //////////////////
    case B10000: 
      fwd(-2,2);
      break;
    case B11000:
      fwd(-1,2);
      break;
    case B01000:
      fwd(0,2);
      break;
    case B01100:
      fwd(1,2);
      break;
    ////////////////////  
    case B00100:
      fwd(2,2);
      break;
    ////////////////////  
    case B00110:
      fwd(2,1);
      break;
    case B00010:  
      fwd(2,0);
      break;
    case B00011:
      fwd(2,-2);
      break;
    case B00001:
      fwd(2,-3); //правый двигатель слабый
      break;     
    ////////////////////   
    case B00000:
      fwd(0,0);
      //beep(128);
      break;
    default:
      //fwd(0,0);
      //beep(20);
      break;
  }
  lastLine = newLine;
  //Serial.println(newLine, BIN);  
}

/////////////////////////////////////////////////////////////
void mode_irremote()
{
  static unsigned long command_time;
  static decode_results results;
  
  if ((millis() - command_time) > IR_TIMEOUT) fwd(0,0);

  if (irrecv.decode(&results)) {
    command_time = millis();
    switch (results.value)
    {
      case IR_LEFT :
        fwd(-4,4);
        break;
      case IR_RIGHT :
        fwd(4,-4);
        break;
      case IR_UP :
        fwd(6,6);
        break;
      case IR_DOWN :
        fwd(-6,-6);
        break;
    }
    irrecv.resume(); // Receive the next value
  }
}
////////////////////////////////////////////////////////////

byte lastLine = 0;
byte newLine = 0;

enum Robot_Mode
{
  IR_REMOTE,
  LINE_TRACKING,
  FREE_RIDE,
  SEEK_LINE,
  STAND_BY
};

Robot_Mode mode = STAND_BY;
decode_results results;

void loop() {

  if (irrecv.decode(&results)) {
    Serial.println(results.value & B11111111, DEC);
    irrecv.resume(); // Receive the next value
  }

  switch (results.value & B11111111)
  {
    case IR_OK: 
    {
      mode = STAND_BY; break;
    }
    case IR_1: 
    {
      mode = LINE_TRACKING; break;
    }
    case IR_2: 
    {
      mode = IR_REMOTE; break;
    }    
  }
 
  //Reading from line-sensor
  newLine = (PINC & B00111110) >> 1;
  if (newLine == lastLine) return;
  
  switch (newLine) 
  {
    //////////////////
    case B10000: 
      fwd(-2,2);
      break;
    case B11000:
      fwd(-1,2);
      break;
    case B01000:
      fwd(0,2);
      break;
    case B01100:
      fwd(1,2);
      break;
    ////////////////////  
    case B00100:
      fwd(2,2);
      break;
    ////////////////////  
    case B00110:
      fwd(2,1);
      break;
    case B00010:  
      fwd(2,0);
      break;
    case B00011:
      fwd(2,-2);
      break;
    case B00001:
      fwd(2,-3); //правый двигатель слабый
      break;     
    ////////////////////   
    case B00000:
      fwd(0,0);
      //beep(128);
      break;
    default:
      //fwd(0,0);
      //beep(20);
      break;
  }
  lastLine = newLine;
  //Serial.println(newLine, BIN);
}
