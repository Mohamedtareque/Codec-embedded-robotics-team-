/* 
* Authors : Kotoz , Tota, Azazi ,Abd elrahman  
* date : 31/7/2017 
* description : 
* Maze robot Algorithim with L293D Driver . 
*/

/*
**************************************************************
************ L239D Driver Description and test code***********
* ************************************************************
*                          L293D
*                    En 1,2 <-****O*****-> VCC1
*                    Input1 <-**********-> Input4
*                   Output1 <-**********-> Output4
*                       GND <-**********-> GND
*                       GND <-**********-> GND
*                   Ouptput2<-**********-> Outout3
*                     Input2<-**********-> Input3
*                       VCC2<-**********-> En 3 ,4
*  Vcc 2 is a voltage to motors 
*  Vcc 1 is a voltage to IC 
*  connect all GND with each others  
*  connect Input terminals to Arduino Output pins 
*  connect Output terminals to Motors Wires 
*/

/*
******************************************************************
************ Sensor reading values Description and test code******
* ****************************************************************
* We use IR TX and IR RX to sense difference between colors 
* follow this schematic
* 
*/

/*
******************************************************************
************ Maze Solver Algorithim explanation ******************
* ****************************************************************
* comming soon ............... 
*/
/*
*************************************************************
************Motor pins configration to arduino***************
************************************************************
*/
#define           right_motor_1       2
#define           right_motor_2       3
#define           left_motor_1        4
#define           left_motor_2        5
#define           enable1             9
#define           enable2             10
/*
*************************************************************
************ Additions maters ******************************
************************************************************
*/
#define       button           6 
#define       led              7
#define       leap_time        50

/*
*************************************************************
************ types definations *****************************
************************************************************
*/
typedef unsigned char uint8_t ;
typedef unsigned int uint16_t ;
/*
*************************************************************
************Sensor pins configration to arduino***************
************************************************************
*/
#define       analog_sensor_right        A0 // Right sensor 
#define       analog_sensor_middle       A1 // middle Sensor 
#define       analog_sensor_left         A2 // Left Sensor 
/*
*************************************************************
************ analog sensor reading confgration *************
*************************************************************
*/
#define          WHITE          200
#define          BLACK          500 
#define          comp_val       150
/*
*************************************************************
************ analog Motor Write confgration PWM *************
*************************************************************
*/
#define         SPEED           255
#define         TURN_SPEED      200
#define         NO_SPEED        0
/*
*************************************************************
************ Black and white States  ************************
*************************************************************
*/
#define black(sensor_reading,comp_val)(sensor_reading>comp_val? 1 : 0 )
#define white(sensor_reading,comp_val)(sensor_reading>comp_val? 1 : 0 )

/*
*************************************************************
************ Motor control functions states ***************
************************************************************
*/
void forward(){                     // Forward Function state
    digitalWrite(right_motor_1,LOW);
    digitalWrite(right_motor_2,HIGH);
    digitalWrite(left_motor_1,LOW);
    digitalWrite(left_motor_2,HIGH);
    analogWrite(enable1,SPEED);
    analogWrite(enable2,SPEED);
   // Serial.println("Forward") ;
  }
void left(){                      // Left function state
    digitalWrite(right_motor_1,LOW);
    digitalWrite(right_motor_2,HIGH);
    digitalWrite(left_motor_1,HIGH);
    digitalWrite(left_motor_2,LOW);
    analogWrite(enable1,SPEED);
    analogWrite(enable2,TURN_SPEED);
   // Serial.println("left") ;
  }

void right(){                     // Right function state
    digitalWrite(right_motor_1,HIGH);
    digitalWrite(right_motor_2,LOW);
    digitalWrite(left_motor_1,LOW);
    digitalWrite(left_motor_2,HIGH);
     analogWrite(enable1,TURN_SPEED);
    analogWrite(enable2,SPEED);
 //   Serial.println("right") ;
  }

void halt(){                  // Stop function state
    digitalWrite(right_motor_1,LOW);
    digitalWrite(right_motor_2,LOW);
    digitalWrite(left_motor_1,LOW);
    digitalWrite(left_motor_2,LOW);
    analogWrite(enable1,NO_SPEED);
    analogWrite(enable2,NO_SPEED);
 //   Serial.println("Stop") ;
  }
/*
*************************************************************
********************** Algorithm States  ********************
*************************************************************
*/
static boolean state = false; 
/*void state_update(state){
    boolean st = state; 
    static uint8_t num = 0 ;
    if(st == true){
        num++;
         
    }else{
        num = 0 ;
        }
  } */
uint8_t path[30];
static uint8_t pathlength = 0 ; 
void blinking(){
  for(int i = 0 i<5 ; i++){
  digitalWrite(led,HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  }
}
void handle(){
    state = false; 
    right();
    delay(leap_time);
    state =true ;
    if(!state){
        halt(); 
        blinking();
      }
  }
void reduce_map(uint8_t map_len){
    for(int i =0 ; i<= map_len ; i++){
        if(path[i]=='B'&&path[i+1]=='R' && path[i-1]=='R'){
            path[i-1]= 0;
            path[i+1]= 0;
            path[i]= 'S' ;
          }if(path[i]=='B'&&path[i+1]=='S' && path[i-1]=='R'){
            path[i-1]= 0;
            path[i+1]= 0;
            path[i]= 'L' ;
          }if(path[i]=='B'&&path[i+1]=='R' && path[i-1]=='S'){
            path[i-1]= 0;
            path[i+1]= 0;
            path[i]= 'L' ;
          }
      }
  }
void check_states(uint8_t right_sens_reading, uint8_t middle_sens_reading ,uint8_t left_sens_reading){// forward state
    if(white(right_sens_reading,comp_val) && white(left_sens_reading,comp_val)&&black(middle_sens_reading,comp_val) ){
          forward();
          state = true ;
          if(!state){
              handle();
            }
          path[pathlength]='F';
          pathlength++;
          reduce_map(pathlength);
      }else if(black(right_sens_reading,comp_val) && white(left_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){// right state
            state = false ;
            right();
            state = true ;
            if(!state){
              handle();
            }
            path[pathlength]='R';
            pathlength++;
            reduce_map(pathlength);
        }else if(black(right_sens_reading,comp_val) && white(right_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){//left state
              state = false ;
              left();
              state = true ;
              if(!state){
                  handle();
                }
              path[pathlength]='L';
              pathlength++;
              reduce_map(pathlength);
          }else if(black(right_sens_reading,comp_val) && black(right_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){
              state = false ;
              right();
              state = true ;
              if(!state){
                  handle();
                }
              path[pathlength]='R';
              pathlength++;
              reduce_map(pathlength);
            }else if(white(right_sens_reading,comp_val) && white(right_sens_reading,comp_val)&&white(middle_sens_reading,comp_val)){
                state = false;
                halt();
                right();
                state = true; 
                if(!state){
                    handle();
                  }
                path[pathlength]='B';
                pathlength++;
                reduce_map(pathlength);
              }else if(white(right_sens_reading,comp_val) && black(right_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){
                  state=false; 
                  forward();
                  state = true ; 
                  if(!state){
                      handle();
                    }
                  path[pathlength]='S';
                  pathlength++;
                  reduce_map(pathlength);
                }
  }


/*
*************************************************************
********************** Safety switch *****************
*************************************************************
*/
boolean switch_read(){
    uint8_t temp = digitalRead(button);
    delay(15);
    temp = digitalRead(button);
    return temp ;
  }

/*
*************************************************************
********************** sensor reading values*****************
*************************************************************
*/
uint8_t sensor_reading(uint8_t sensor){
    uint8_t temp = analogRead(sensor);
    return temp;
  }

void setup() { 
  // confg pins mode for motor 
  pinMode(right_motor_1,OUTPUT);
  pinMode(right_motor_2,OUTPUT);
  pinMode(left_motor_1,OUTPUT);
  pinMode(left_motor_2,OUTPUT);
  // confg sensors pins 
  pinMode(analog_sensor_right,INPUT);
  pinMode(analog_sensor_left,INPUT);
  pinMode(analog_sensor_middle,INPUT);
  //enable pins confg 
  pinMode(enable1,OUTPUT);
  pinMode(enable2,OUTPUT);
  // additional pins 
  pinMode(button,INPUT);
  pinMode(led,OUTPUT);
  Serial.begin(9600);
}


void loop() {
  while(switch_read()== HIGH){
  uint8_t right_reading = sensor_reading(analog_sensor_right);
  uint8_t left_reading = sensor_reading(analog_sensor_left);
  uint8_t middle_reading = sensor_reading(analog_sensor_middle);
  check_states(right_reading,middle_reading,left_reading);//check_states(right_sens_reading,middle_sens_reading ,left_sens_reading)
  }
}
