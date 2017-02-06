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

///////////////////////////////////////////////////////
// Stop motors
///////////////////////////////////////////////////////
void stp()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}
/////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// BEEP function TODO: not working
//////////////////////////////////////////////////////
void beep(int ton) //TODO not work
{
  analogWrite(BUZZER, ton);
  delay(50);
  analogWrite(BUZZER, 0);
}
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//    SETUP FUNCTION
//////////////////////////////////////////////////////
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
  //Serial.begin(9600);
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
// LOW LEVEL MOTORS ACTIVATION
///////////////////////////////////////////////
void fwd(int l, int r)
{
  //left motor
  if (l > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(E1, l);
  } else if (l < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(E1, abs(l));
  } else if (l == 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(E1, 255);
  }      

  //right motor
  if (r > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(E2, r);
  } else if (r < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(E2, abs(r));
  } else if (r == 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(E2, 255);
  }

  //Крутит башкой
  if (l < r) {
    hServ.write(CENTER + 30);
    } else if (l > r) {
      hServ.write(CENTER - 30);
      } else hServ.write(CENTER);
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
//  Robot moving. Line follow mode
///////////////////////////////////////////////////
#define MIN_PWM 110
#define MAX_PWM 176
#define MEDIUM_PWM (MAX_PWM - MIN_PWM)/2
#define STEPS 8 // 0-7 from readLine
#define ONESTEP (MAX_PWM-MIN_PWM)/STEPS

void line_follow()
{
  int l;
  int r;
  static int last_line = 0xFF;
  int line = readLine();
  if (line != last_line)
  {
    if (line > 5)
    {
        l = 0;
        r = MAX_PWM;
    } 
    else if (line < -5) 
    {
      l = MAX_PWM;
      r = 0;
    } 
    else 
    {
      l = MAX_PWM - (constrain(line, 0, STEPS)) * ONESTEP; //left motor
      r = MAX_PWM - (abs(constrain(line, -STEPS, 0))) * ONESTEP; //right motor
    }
    //Serial.print(l);
    //Serial.print(" - ");
    //Serial.println(r);
    fwd(l,r);
    last_line = line;
  }
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//  Line follow sensor reading (5 lines)
//////////////////////////////////////////////////////
int readLine()
{
  static int lastResult;
  int result;  
  byte newLine = (PINC & B00111110) >> 1;
  switch (newLine){
    case B10000: result = 6; break;
    case B11000: result = 5; break;
    case B11110: result = 4; break;    
    case B11100: result = 3; break;
    case B01000: result = 2; break;
    case B01100: result = 1; break;
    case B00100: result = 0; break;
    case B00110: result = -1; break;
    case B00010: result = -2; break;
    case B00111: result = -3; break;
    case B01111: result = -4; break;
    case B00011: result = -5; break;
    case B00001: result = -6; break;
    case B00000: if (lastResult > 0){ // out of line
      result = 7; 
      } else {
        result = -7; 
      }
      break;

    default: {result = lastResult; break;} //skip undefined situation
  }
  lastResult = result; 
  return result; 
}
///////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
// IR-remote moving
////////////////////////////////////////////////////////////
void mode_irremote(int code)
{
  static unsigned long command_time;
  
  if ((millis() - command_time) > IR_TIMEOUT) stp();

  if (code != 0) {
    command_time = millis();
    switch (code)
    {
      case IR_LEFT :
        fwd(-(MIN_PWM+4*ONESTEP),(MIN_PWM+4*ONESTEP));
      break;
      case IR_RIGHT :
        fwd((MIN_PWM+4*ONESTEP),-(MIN_PWM+4*ONESTEP));
      break;
      case IR_UP :
        fwd((MIN_PWM+6*ONESTEP),(MIN_PWM+6*ONESTEP));
      break;
      case IR_DOWN :
        fwd(-(MIN_PWM+6*ONESTEP),-(MIN_PWM+6*ONESTEP));
      break;
    }
  }
}
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////
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

int last_ir_code = 0;

void loop() {

  int ir_code = 0;
  if (irrecv.decode(&results)) {
    ir_code = results.value & B11111111;
    //Serial.println(ir_code);
    switch (ir_code)
    {
      case IR_OK: mode = STAND_BY;      break;
      case IR_1:  mode = LINE_TRACKING; break;
      case IR_2:  mode = IR_REMOTE;     break;   
    }
    last_ir_code = ir_code;  
    irrecv.resume(); // Receive the next value
  }

  switch (mode)
  {
    case STAND_BY:
      stp();
      break;
    case IR_REMOTE:
      mode_irremote(ir_code);
      break;
    case LINE_TRACKING:
      line_follow();
      break;
    default:
      break;
  }  
}
/////////////////////////////////////////////////////////
