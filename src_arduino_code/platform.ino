/* <Controlling code for Arduino Controlled Rotary Stewart Platform>
    Copyright (C) <2014>  <Tomas Korgo>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

#include <Servo.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

//define values of IrDA codes send from used remote control
#define MUTE_BUTTON 0x807F906F
#define STANDBY_ON 0x807F20DF
#define VOL_UP 0x807F609F
#define VOL_DOWN 0x807FA05F
#define DVD 0x807F08F7
#define TV 0x807F8877
#define GAME 0x807F48B7
#define CD 0x807FC837

//define of characters used for control of serial communication ['0'-'8']
#define SETBACKOFF 48
#define SETBACKON 49
#define SETPOSITIONS 50
#define PRINTPOS 51
#define STOPPRINTPOS 52
#define SWITCHIRDA 53
#define SETPOSITIONSINMS 54
#define SWITCHIRDAOFF 55
#define GEPOSITION 56
//defines of LCD pin numbers, most probably they dont have to be changed except of I2C_ADDR which value is neccessary and have to be changed.
#define I2C_ADDR 0x27
#define LCD 1
#define IrDA 1
#define BACKLIGHT_PIN 3
#define En 2
#define Rw 1
#define Rs 0
#define D4 4
#define D5 5
#define D6 6
#define D7 7
//MIN and MAX PWM pulse sizes, they can be found in servo documentation
#define MAX 2200
#define MIN 800

//Positions of servos mounted in opposite direction
#define INV1 1
#define INV2 3
#define INV3 5

//constants for computation of positions of connection points
#define pi  3.14159
#define deg2rad 180/pi
#define deg30 pi/6
//variables used for proper show of positions on LCD
char shown=0, showPos=0, useIrda=0;
unsigned long time;

//variable to store connected LCD
LiquidCrystal_I2C lcd(I2C_ADDR, En, Rw, Rs, D4,D5,D6,D7);
//IrReciever variables
IRrecv irrecv(12); // Receive Ir on digital pin n 12
decode_results results;

//Array of servo objects
Servo servo[6];
//Zero positions of servos, in this positions their arms are perfectly horizontal, in us
static int zero[6]={1475,1470,1490,1480,1460,1490};
//In this array is stored requested position for platform - x,y,z,rot(x),rot(y),rot(z)
static float arr[6]={0,0.0,0, radians(0),radians(0),radians(0)};
//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6]={0.0,0.0,0.0, 0.0,0.0,0.0};
//Array of current servo positions in us
static int servo_pos[6];
//rotation of servo arms in respect to axis x
const float beta[] = {pi/2,-pi/2,-pi/6, 5*pi/6,-5*pi/6,pi/6},
//maximum servo positions, 0 is horizontal position
      servo_min=radians(-80),servo_max=radians(80),
//servo_mult - multiplier used for conversion radians->servo pulse in us
//L1-effective length of servo arm, L2 - length of base and platform connecting arm
//z_home - height of platform above base, 0 is height of servo arms
servo_mult=400/(pi/4),L1 = 0.79,L2 = 4.66, z_home = 4.05;
//RD distance from center of platform to attachment points (arm attachment point)
//RD distance from center of base to center of servo rotation points (servo axis)
//theta_p-angle between two servo axis points, theta_r - between platform attachment points
//theta_angle-helper variable
//p[][]=x y values for servo rotation points
//re[]{}=x y z values of platform attachment points positions
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position
const float RD = 2.42,PD =2.99,theta_p = radians(37.5),
theta_angle=(pi/3-theta_p)/2, theta_r = radians(8),
      p[2][6]={
          {
            -PD*cos(deg30-theta_angle),-PD*cos(deg30-theta_angle),
            PD*sin(theta_angle),PD*cos(deg30+theta_angle),
            PD*cos(deg30+theta_angle),PD*sin(theta_angle)
         },
         {
            -PD*sin(deg30-theta_angle),PD*sin(deg30-theta_angle),
            PD*cos(theta_angle),PD*sin(deg30+theta_angle),
            -PD*sin(deg30+theta_angle),-PD*cos(theta_angle)
         }
      },
      re[3][6] = {
          {
              -RD*sin(deg30+theta_r/2),-RD*sin(deg30+theta_r/2),
              -RD*sin(deg30-theta_r/2),RD*cos(theta_r/2),
              RD*cos(theta_r/2),-RD*sin(deg30-theta_r/2),
          },{
              -RD*cos(deg30+theta_r/2),RD*cos(deg30+theta_r/2),
              RD*cos(deg30-theta_r/2),RD*sin(theta_r/2),
              -RD*sin(theta_r/2),-RD*cos(deg30-theta_r/2),
          },{
              0,0,0,0,0,0
          }
};
//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move
static float M[3][3], rxp[3][6], T[3], H[3] = {0,0,z_home};

void setup(){
//LCD inicialisation and turning on the backlight
#if LCD
   lcd.begin(16,2);
   lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
   lcd.setBacklight(HIGH);
   lcd.home();
#endif

//attachment of servos to PWM digital pins of arduino
   servo[0].attach(3, MIN, MAX);
   servo[1].attach(5, MIN, MAX);
   servo[2].attach(6, MIN, MAX);
   servo[3].attach(9, MIN, MAX);
   servo[4].attach(10, MIN, MAX);
   servo[5].attach(11, MIN, MAX);
//begin of serial communication
   Serial.begin(9600);
//putting into base position
   setPos(arr);
}

//function calculating needed servo rotation value
float getAlpha(int *i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[*i];
   while(n<20){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = L1*cos(th)*cos(beta[*i]) + p[0][*i];
      q[1] = L1*cos(th)*sin(beta[*i]) + p[1][*i];
      q[2] = L1*sin(th);
    //calculation of distance between according platform attachment point and base attachment point
      dl[0] = rxp[0][*i] - q[0];
      dl[1] = rxp[1][*i] - q[1];
      dl[2] = rxp[2][*i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);
    //if this distance is the same as leg length, value of theta_a is corrent, we return it
      if(abs(L2-dl2)<0.01){
         return th;
      }
    //if not, we split the searched space in half, then try next value
      if(dl2<L2){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
}

//function calculating rotation matrix
void getmatrix(float pe[])
{
   float psi=pe[5];
   float theta=pe[4];
   float phi=pe[3];
   M[0][0] = cos(psi)*cos(theta);
   M[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   M[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);

   M[0][1] = sin(psi)*cos(theta);
   M[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   M[2][1] = cos(theta)*sin(phi);

   M[0][2] = -sin(theta);
   M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
   M[2][2] = cos(theta)*cos(phi);
}
//calculates wanted position of platform attachment poins using calculated rotation matrix
//and translation vector
void getrxp(float pe[])
{
   for(int i=0;i<6;i++){
      rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i]);
      rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i]);
      rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i]);
   }
}
//function calculating translation vector - desired move vector + home translation vector
void getT(float pe[])
{
   T[0] = pe[0]+H[0];
   T[1] = pe[1]+H[1];
   T[2] = pe[2]+H[2];
}

unsigned char setPos(float pe[]){
    unsigned char errorcount;
    errorcount=0;
    for(int i = 0; i < 6; i++)
    {
        getT(pe);
        getmatrix(pe);
        getrxp(pe);
        theta_a[i]=getAlpha(&i);
        if(i==INV1||i==INV2||i==INV3){
            servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, MIN,MAX);
        }
        else{
            servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, MIN,MAX);
        }
    }

    for(int i = 0; i < 6; i++)
    {
        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MIN||servo_pos[i]==MAX){
            errorcount++;
        }
        servo[i].writeMicroseconds(servo_pos[i]);
    }
    return errorcount;
}

//functions used for displaying actual platform position on 16x2 LCD display
#if LCD
void showRot(){
   lcd.setCursor(0,0);
   lcd.print("Rot");
   lcd.setCursor(12,0);
   lcd.print((int)(arr[3]*deg2rad));
   lcd.setCursor(3,1);
   lcd.print((int)(arr[4]*deg2rad));
   lcd.setCursor(11,1);
   lcd.print((int)(arr[5]*deg2rad));
}
void showComm(){
   if(shown==0){
      shown=1;
      lcd.setCursor(3,0);
      lcd.print("ation x: ");
      lcd.setCursor(0,1);
      lcd.print("y: ");
      lcd.setCursor(8,1);
      lcd.print("z: ");
   }
}
void clearNr(){
   lcd.setCursor(12,0);
   lcd.print("    ");
   lcd.setCursor(3,1);
   lcd.print("     ");
   lcd.setCursor(11,1);
   lcd.print("     ");

}
void showLoc(){
   lcd.setCursor(0,0);
   lcd.print("Loc");
   lcd.setCursor(12,0);
   lcd.print((int)(arr[0]*25.4));
   lcd.setCursor(3,1);
   lcd.print((int)(arr[1]*25.4));
   lcd.setCursor(11,1);
   lcd.print((int)(arr[2]*25.4));
}
#endif

//main control loop, obtain requested action from serial connection, then execute it
void loop()
{

   if(Serial.available()>0){
      int input=Serial.read();
      switch(input){
//action to turn backlight off
         case SETBACKOFF:
#if LCD
            lcd.setBacklight(LOW);
#endif
            break;
//action to turn backlight on
         case SETBACKON:
#if LCD
            lcd.setBacklight(HIGH);
#endif
            break;
//action to change position of platform, obtain 6 values representing desired position
         case SETPOSITIONS:
            for(int i=0;i<6;i++){
               long kk;
               while(Serial.available()<4){
                  ;
               }
               kk=(long)Serial.read();
               kk=kk+(Serial.read()<<8);
               kk=kk+(Serial.read()<<16);
               kk=kk+(Serial.read()<<24);
               if(i<3){
                  arr[i]=(kk/100)/25.4;
               }else{
                  arr[i]=radians(kk/100.0);
               }
            }
            Serial.write(setPos(arr));
            Serial.flush();
            break;
//enable of showing current position on LCD
         case PRINTPOS:
#if LCD
            showPos=PRINTPOS;
            time=millis();
#endif
            break;
//enable of controlling platformy by IrDA remote
        case SWITCHIRDA:
#if IrDA
            irrecv.enableIRIn();
            useIrda=SWITCHIRDA;
#endif
            break;
//reserved for future use - possiblity to send just servo timing values
//main control would be executed on communicating partner
         case SETPOSITIONSINMS:
            for(int i=0;i<6;i++){
               long kk;
               while(Serial.available()<4){
                  ;
               }
               kk=(long)Serial.read();
               kk=kk|(Serial.read()<<8);
               kk=kk|(Serial.read()<<16);
               kk=kk|(Serial.read()<<24);
               servo[i].writeMicroseconds(kk);
            }
            break;
//disable of showing current position on LCD
         case STOPPRINTPOS:
            showPos=STOPPRINTPOS;
            shown=0;
            break;
//disable of controlling platformy by IrDA remote
         case SWITCHIRDAOFF:
            useIrda=SWITCHIRDAOFF;
            break;
//return current position of platform
         case GEPOSITION:
            retPos();
            break;
         default:
            break;
      }
   }
//helping subroutine to print current position
#if LCD
   if(showPos==PRINTPOS){
      static byte act=0;
      showComm();
      if(millis()-time<1500){
         act=0;
      }else if(millis()-time<3000){
         if(act==0){
            act=1;
            clearNr();
            showRot();
         }
      }else{
         time=millis();
         clearNr();
         showLoc();
      }
   }
#endif
//this part is used for IrDA position control
#if IrDA
   if(useIrda==SWITCHIRDA&&irrecv.decode(&results)){
      static byte val=0;
      switch(results.value){
         case MUTE_BUTTON:
            val=1;
            break;
         case STANDBY_ON:
            val=0;
            break;
         case VOL_UP:
            if(val<=2){
               arr[val]+=1;
            }else{
               arr[val]=radians((arr[val]*deg2rad)+0.4);
            }
            break;
         case VOL_DOWN:
            if(val<=2){
               arr[val]-=1;
            }else{
               arr[val]=radians((arr[val]*deg2rad)-0.4);
            }
            break;
         case DVD:
            val=2;
            break;
         case TV:
            val=3;
            break;
         case GAME:
            val=4;
            break;
         case CD:
            val=5;
            break;
      }
      setPos(arr);
      irrecv.resume(); // Continue receiving
   }
#endif

}

void retPos(){
   for(int i=0;i<6;i++){
       long val;
       if(i<3){
           val=(long)(arr[i]*100*25.4);
       }else{
           val=(long)(arr[i]*100*deg2rad);
       }
       Serial.write(val);
       Serial.write((val>>8));
       Serial.write((val>>16));
       Serial.write((val>>24));
       Serial.flush();
   }
}
