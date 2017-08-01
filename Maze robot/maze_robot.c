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
* 
*/

/*
******************************************************************
************ Sensor reading values Description and test code******
* ****************************************************************
* 
*/

/*
******************************************************************
************ Maze Solver Algorithim explanation ******************
* ****************************************************************
* 
*/
/*
*************************************************************
************Motor pins configration to arduino***************
************************************************************
*/
#define right_motor_1 2
#define right_motor_2 3
#define left_motor_1 4
#define left_motor_2 5
#define enable1 9
#define enable2 10

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
#define analog_sensor_right A0 // Right sensor 
#define analog_sensor_middle A1 // middle Sensor 
#define analog_sensor_left A2 // Left Sensor 
/*
*************************************************************
************ analog sensor reading confgration *************
*************************************************************
*/
#define WHITE 200
#define BLACK 500 
#define comp_val 150
/*
*************************************************************
************ analog Motor Write confgration PWM *************
*************************************************************
*/
#define SPEED 255
#define TURN_SPEED 200
#define NO_SPEED 0
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
void check_states(uint8_t right_sens_reading, uint8_t middle_sens_reading ,uint8_t left_sens_reading){// forward state
    if(white(right_sens_reading,comp_val) && white(left_sens_reading,comp_val)&&black(middle_sens_reading,comp_val) ){
          forward();
      }else if(black(right_sens_reading,comp_val) && white(left_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){// right state
            right();
        }else if(black(right_sens_reading,comp_val) && white(right_sens_reading,comp_val)&&black(middle_sens_reading,comp_val)){//left state
              left();
          }
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
  Serial.begin(9600);
}


void loop() {
  uint8_t right_reading = sensor_reading(analog_sensor_right);
  uint8_t left_reading = sensor_reading(analog_sensor_left);
  uint8_t middle_reading = sensor_reading(analog_sensor_middle);
  check_states(right_reading,middle_reading,left_reading);//check_states(right_sens_reading,middle_sens_reading ,left_sens_reading)
}
