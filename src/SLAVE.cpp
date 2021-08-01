
//SLAVE - Arduino Uno //


#include <Arduino.h>

//----------------------------Comunicação i2c----------------------------//
  #include <Wire.h>
  byte byte1, byte2, byte3, byte4;    //Bytes do sinal
  int Erro_bytes = 4;                 //Quantidade de bytes esperados no sinal
  int Erro;           

//----------------------------Driver Setup----------------------------//
  #include <Motor_Comands.h>
  #define LPWM 6    //lpwm
  #define RPWM 5    //rpwm
  #define ENABLE 9  //pwm enable
  
  Driver_Setup Motor;

//----------------------------PID Settings----------------------------//

  #include <PID.h>
  int Setpoint, Input, Output;         //Define Variables we'll be connecting to
  PID_Control PID_Calculator;             //PID object


//----------------------------SD Data Settings----------------------------//
  
 
  
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://

void setup()
{ 
  Serial.begin(9600);
  
  //----------------------------Driver Configurations----------------------------//
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(ENABLE, OUTPUT);
   
  }

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://


void loop()
{ 
  //i2c Parameters//
  Wire.begin(44);
  Wire.onReceive(Erro_Read);
  delay(200);

  
  
}
   
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://
  

//------------------------------------------------------------------------//
void Erro_Read(int Erro_bytes) //Lê os bytes recebidos pela comunicação Master-Slave
{
  
  byte1 = Wire.read();   //16 bits do Setpoint tipo int            
  byte2 = Wire.read();  
  byte3 = Wire.read();   //16 bits do KalAngleX tipo int
  byte4 = Wire.read();
  
     
  Setpoint =  (byte1 << 8)| byte2 ; 
  Input =  (byte3 << 8)| byte4 ; 
   
  Setpoint = 0; //constrain(Setpoint,-80,80);
  Input = constrain(Input,-80,80);
   
  Erro = Setpoint - Input;  
  Output = PID_Calculator.PID(Erro); 
  
  Output = map(Output,-216,216,110,230);
  
  Serial.print("Erro = ");Serial.print(Erro,DEC);Serial.print("\t");
  Serial.print("Setpoint = ");Serial.print(int(Setpoint),DEC);Serial.print("\t");
  Serial.print("Output = ");Serial.print(Output);
  Serial.println("\t");  
    
  Motor_Direction(Erro,Output,Input);
  
  }
//------------------------------------------------------------------------//



//------------------------------------------------------------------------//
void Motor_Direction(int erro, int PWM, int input){
  
  int Threshold_Min = -2;
  int Threshold_Max = 2;
  int Max_Angle_Limit = 85;
  int Min_Angle_Limit = -85;
    
  if (90 > input*-1 > -90 && erro < Threshold_Min) //Girar no sentido horário
  { 
    analogWrite(ENABLE, PWM); //0-255
    digitalWrite(LPWM, LOW);
    digitalWrite(RPWM, HIGH);
    Serial.print("Input: ");Serial.print(input);Serial.print("\t");
    Serial.print("Sentido: "); Serial.print("Hor"); Serial.print("\t");   
    }
    
  else if ( -90 < input < 90 && erro > Threshold_Max) //Girar no sentido antihorário
  {
    analogWrite(ENABLE, PWM); //0-255
    digitalWrite(LPWM, HIGH);
    digitalWrite(RPWM, LOW);
    Serial.print("Input: ");Serial.print(input);Serial.print("\t");
    Serial.print("Sentido: ");Serial.print("AntiHor"); Serial.print("\t");
      
    }
    
   else if (Threshold_Min < erro < Threshold_Max)  //Não Girar
  {
    analogWrite(ENABLE, PWM); //0-255
    digitalWrite(LPWM, LOW);
    digitalWrite(RPWM, LOW);
    Serial.print("Input: ");Serial.print(input);Serial.print("\t");
    Serial.print("Sentido: ");Serial.print("Parado"); Serial.print("\t");   
    }
  
  }
//------------------------------------------------------------------------//

int mapeamento(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//------------------------------------------------------------------------//

void Motor_Stop()
{
    digitalWrite(LPWM, LOW);
    digitalWrite(RPWM, LOW);
    Serial.println("Fim de curso!");
  
  }
