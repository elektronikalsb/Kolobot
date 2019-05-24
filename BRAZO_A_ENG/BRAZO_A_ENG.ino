/********************************************************************************
     Program to control Dynamixel of robotic arm by urko_18
 ********************************************************************************/
#include <ax12.h> //Include ArbotiX DYNAMIXEL library
#include <SPI.h>
#include <SD.h>
#include <avr/interrupt.h>
#define EMERGENCY_BUTTON (PINA&0x04)
const int SERVO_ID[] = {1, 2, 3, 4, 5, 6}; 
const int servo_count = sizeof(SERVO_ID) / sizeof(*SERVO_ID);
int repetitions, interrupt_state;
double Speed;
int m = 200; //Number of positions of each motor that will save in the SD
int pos2[] = {2125, 1330, 1630, 3400, 100, 680}; //rest position of the dynamixel motors
File myFile;

void setup()
{
  dxlInit(1000000); //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600 for reporting data.

  for (int i = 0; i < servo_count; ++i)
  {
    Relax(SERVO_ID[i]);
    Serial.print(F("ID: "));
    Serial.println(SERVO_ID[i]);
  }
  PORTA |= 0x1E;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= 0x1E;
  interrupt_state = 0;
  interrupts();
  delay(1000);
  
  Serial.print("Initializating SD ...");
  if (!SD.begin(4)) {
    Serial.println("It failed to initialize");
    return;
  }
  Serial.println("successful initialization");

  delay(1000);

}

void loop()
{

  Serial.println(F("How many times do you want to repeat the sequence?"));

  while (Serial.available() == 0) {}
  repetitions = Serial.parseInt();
  Serial.print(F("repetitions = "));
  Serial.println(repetitions);
  delay(3000);

  Serial.println(F("How fast do you want to repeat the sequence?"));
  Serial.println(F(" 1 = Slow     2  = Normal     3 = Fast"));

  while (Serial.available() == 0) {}
  Speed = Serial.parseInt();

  if (Speed <= 0 || Speed >= 4)
  {
    Serial.println(F("You have entered a wrong value"));
    return;
  }
  if (Speed == 1)
  {
    Speed = 1000;
  }
  else if (Speed == 2)
  {
    Speed = 400;
  }
  else if (Speed == 3)
  {
    Speed = 200;
  }
  Serial.print(F("Speed = "));
  Serial.print(Speed, 0);
  Serial.println(F(" microseconds"));
  delay(3000);

  int positionn[servo_count][m]; //Matrix of movements
  
  myFile = SD.open("Prueba.txt", FILE_WRITE); // Note

    if(myFile)
        Serial.println("file is open");
    else
        Serial.println("Error opening the file");
        
  Serial.print(F("Positions vector "));
  Serial.print(F(": ["));
 
  for (int i = 0; i < m; i++) // structure to create columns
  {
    for (int j = 0; j < servo_count; j++) // structure to create columns
    {
      int pos = dxlGetPosition(SERVO_ID[j]);
      myFile.println(pos); //read and save the actual position

      Serial.print(pos); //Display the vector
      Serial.print(F(", "));
    }
    delay(5);
  }
  
  Serial.print(F("]\n"));
  delay(3000);
  myFile.close(); //close the file

  
  Serial.println(F("The servos will move to the initial position."));
  /***The servos will move according to registered movements***/

    for (int a = 0; a < repetitions; a++) //Repetition of the process (a = number of sequences)
  {
    Serial.print(F("SEQUENCE "));
    Serial.println(a + 1);

    int pos_from_file[servo_count]; // Note
    int position[servo_count];
    int turns[servo_count];
    int pos1[servo_count];
    int pos2[servo_count];
    int current[servo_count];

  myFile = SD.open("Prueba.txt", FILE_READ); // Note
    
    for (int i = 0; i < servo_count; i++)
    {
      pos_from_file[i] = myFile.parseInt(); // Note
      current[i] = pos_from_file[i];
      position[i] = current[i];
      turns[i] = 0;
      pos1[i] = dxlGetPosition(SERVO_ID[i]); //Actual servo position
      pos2[i] = position[i]; //Initial position of the movement (objective)
    }
    for (int servo = 0; servo < servo_count; ++servo)
    {
      go_to_position(pos1, pos2, servo); //Function that moves the robot to the initial position
    }

    Serial.println(F("Now the servos will do the registered movements."));
    delay(1000);

    for (int movement = 0; movement < m; movement++)
    {
      for (int servo = 0; servo < servo_count; servo++)
      {
        if (pos_from_file[servo] != current[servo])
        {
          int next_pos = 1;
          if (pos_from_file[servo] < current[servo]) // Note
            next_pos = -1;
          while (pos_from_file[servo] != current[servo]) // Note
          {
            dxlSetGoalPosition(SERVO_ID[servo], current[servo]);
            current[servo] += next_pos;
            delayMicroseconds(Speed);

            if (current[servo] == position[servo] + MX_MAX_POSITION_VALUE)
            {
              position[servo] = current[servo];
              turns[servo]++;
            }
            else if (current[servo] == position[servo] - MX_MAX_POSITION_VALUE)
            {
              position[servo] = current[servo];
              turns[servo]--;
            }
          }
        }
        pos_from_file[servo] = myFile.parseInt(); // Note
      }
    }
    for (int i = 0; i < servo_count; i++)
    {
      Serial.print(F("Turns engine "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.println(turns[i]);
      Serial.println(" ");
    }
  }
  myFile.close(); //close the file
  delay(2000);

  /****REST POSITION****/
  Serial.println(F("The robot will move to the resting position."));
  int pos1[servo_count];
  for (int i = 0; i < servo_count; i++)
  {
    pos1[i] = dxlGetPosition(SERVO_ID[i]); //Actual servo position
  }
  for (int servo = 0; servo < servo_count; ++servo)
  {
    go_to_position(pos1, pos2, servo);  //Function that moves the robot to the initial position
  }
  delay(1000);
  dxlTorqueOffAll();
  
   if (SD.exists("Prueba.txt")) {        // SI EXISTE EL ARCHIVO
    SD.remove("Prueba.txt");           // ELIMINAR ARCHIVO
    Serial.println("SD FILE REMOVED");
  } else {
    Serial.println("THE FILE DOESN´T EXIST");
  }
  Serial.println(F("END!"));
}
void go_to_position(int pos1[], int pos2[], int servo)//function
{
  while (pos1[servo] != pos2[servo])
  {
    if (pos2[servo] < pos1[servo])
    {
      dxlSetGoalPosition(SERVO_ID[servo], pos1[servo]);
      pos1[servo]--;
      delayMicroseconds(800);
    }
    else if (pos2[servo] > pos1[servo])
    {
      dxlSetGoalPosition(SERVO_ID[servo], pos1[servo]);
      pos1[servo]++;
      delayMicroseconds(800);
    }
  }
}
ISR(PCINT0_vect) {
  interrupt_state = (PINA & 0x04) >> 1;
  Serial.println("EMERGENCY BUTTON!");
}
