//String dataFromClient = "-31,-48,-64,-80,-95,-107,-115,-116,-109,-93,-68,-36,0,36,68,93,109,116,115,107,95,80,64,48,31,32,47,57,63,64,56,41,16,-16,-57,-102,-146,-184,-212,-226,-228,-217,-198,-173,-145,-117,-91,-68,-47,-29,-29,-47,-68,-91,-117,-145,-173,-198,-217,-228,-226,-212,-184,-146,-102,-57,-16,16,41,56,64,63,57,47,32,7,10,14,17,21,24,26,26,24,19,12,4,4,13,19,24,26,26,24,21,17,14,10,7,4,13,16,17,16,10,1,12,30,52,76,100,122,138,147,147,139,126,108,88,69,51,36,24,15,8,15,24,36,51,68,88,107,125,139,147,146,138,122,100,75,51,29,11,1,10,16,17,16,13,8";
#include <Stepper.h>

const int vibuoc = 16;

String dataFromClient;

#define numberStepper 3
#define numberSplit 35
#define MAXSPEED 20
#define offset_0do -1 // sai số tại 0 độ
const double Tiso = 36.0 / 20.0;

const int home_limit[numberStepper] = {A3, A2, A1};

const int Maybom = 12;


//int theta[numberStepper][numberSplit];
//int vel[numberStepper][numberSplit];

//int data[numberStepper * 2 * numberSplit] = { -31,-47,-63,-78,-92,-103,-110,-110,-102,-85,-58,-25,11,47,78,102,116,121,118,109,95,80,63,47,30,32,47,58,64,65,58,43,18,-14,-55,-100,-145,-184,-213,-228,-229,-219,-199,-173,-145,-117,-91,-67,-46,-29,-29,-46,-67,-91,-118,-147,-176,-203,-224,-236,-237,-223,-195,-156,-110,-64,-21,13,38,55,63,63,57,47,33,2,3,4,5,6,7,8,8,7,6,3,1,2,5,7,8,9,9,8,7,5,4,3,2,1,4,5,5,5,3,1,4,10,17,25,33,41,46,49,49,47,42,36,29,23,17,12,8,5,2,5,8,12,17,23,29,36,43,47,50,50,48,42,34,26,18,10,4,1,3,5,5,5,4,2};
//int data[numberStepper * 2 * numberSplit] = { -20,-28,-36,-45,-53,-60,-67,-73,-77,-79,-79,-75,-68,-58,-45,-29,-11,7,26,44,59,71,79,85,86,85,82,76,69,62,54,45,37,28,20,22,29,36,41,45,46,46,42,36,26,13,-2,-22,-44,-67,-90,-112,-132,-147,-158,-164,-164,-160,-153,-142,-129,-115,-101,-86,-72,-59,-48,-37,-27,-19,-19,-27,-37,-48,-60,-73,-87,-102,-117,-131,-145,-156,-165,-169,-170,-165,-154,-139,-120,-98,-74,-50,-28,-7,9,23,33,40,44,45,44,41,36,29,22,8,11,14,18,21,25,28,31,33,35,35,33,30,25,18,10,1,7,15,23,30,34,37,38,38,36,33,30,26,22,18,15,11,8,5,17,21,23,24,23,20,14,6,5,20,38,59,82,106,131,154,175,191,202,208,207,200,189,173,155,135,115,96,78,62,47,35,25,17,11,17,25,35,47,62,78,96,116,136,156,175,191,203,210,212,207,196,180,159,135,110,85,61,40,21,6,5,14,20,23,24,23,21,17,12};
long int data[numberStepper * 2 * numberSplit];
const int stepsPerRevolution = 800 * vibuoc;


// initialize the Stepper library on pins 8 through 11:
Stepper myStepper1(1600 * vibuoc, 5, 2);
Stepper myStepper2(1600 * vibuoc, 6, 3);
Stepper myStepper3(1600 * vibuoc, 7, 4);

int calStep(float theta1, float theta2)
{
  float tmpTheta = theta2 - theta1;
  float tmpNumSteps = tmpTheta / ((float)360 / (float)800 / vibuoc) * Tiso;

  int stepsToGo = tmpNumSteps; // số bước cần di chuyển

  //  Serial.print("So goc: ");
  //  Serial.println(tmpTheta);
  //  Serial.print("So buoc: ");
  //  Serial.println(stepsToGo);
  return stepsToGo;
}

long calSpeed(float vel)
{
  if (vel < 0) // do/giay
  {
    vel = -vel;
  }
  //  vel *= numberSplit;
  vel = int(vel * numberSplit);

  //  Serial.print("Van toc vel: ");
  //  Serial.println(vel);
  return vel;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  pinMode(Maybom, OUTPUT);
}

bool kt = true;

void loop() {
  // put your main code here, to run repeatedly:

  //  delay(1000);

  In_Output();

  if (kt)
  {
    Homming();
    Restart();
    float startTheta[3] = { -36.4134, 20.5132, -50.4478};// toạ độ (100,-100,-450);
    //  float startTheta[3] = { -38.43, 52.22, -22.78}; // toạ độ (-100,100,-450);
    //float startTheta[3] = { -35.1953, 35.1953, -35.1953}; // toạ độ băng chuyền -450
    Run(startTheta);
    delay(1000);
    kt = false;
  }
  RunMotor();

  //  while (1);

}

void In_Output()
{
  while (Serial.available() == 0)
  {
  }
  String dataFromClient = Serial.readStringUntil('\n');
  unsigned int data_num = 0;
  // loop as long as a comma is found in the string
  while (dataFromClient.indexOf(",") != -1) {
    // take the substring from the start to the first occurence of a comma, convert it to int and save it in the array
    data[ data_num ] = dataFromClient.substring(0, dataFromClient.indexOf(",")).toInt();
    data_num++; // increment our data counter
    //cut the data string after the first occurence of a comma
    dataFromClient = dataFromClient.substring(dataFromClient.indexOf(",") + 1);
  }
  // get the last value out of the string, which as no more commas in it
  data[ data_num ] = dataFromClient.toInt();


  for (int i = 0; i < numberStepper * 2 * numberSplit; i++)
    Serial.println(data[i]);


}

void RunMotor()
{
  for (int i = 0; i < numberSplit; i++)
  {
    //    Serial.print("indexTheta: ");
    //    Serial.println(i);
    //    Serial.println("");
    //
    Serial.println("1");

    myStepper1.setSpeed(data[numberSplit * numberStepper + 0 * numberSplit + i]);
    Serial.println(data[numberSplit * numberStepper + 0 * numberSplit + i]);
    //    myStepper1.step(data[0 * numberSplit + i]);

    //    Serial.println("\n");
    //
    //    Serial.println("2");

    myStepper2.setSpeed(data[numberSplit * numberStepper + 1 * numberSplit + i]);
    Serial.println(data[numberSplit * numberStepper + 1 * numberSplit + i]);
    //    myStepper2.step(data[1 * numberSplit + i]);
    //    Serial.println("\n");

    //    Serial.println("3");

    myStepper3.setSpeed(data[numberSplit * numberStepper + 2 * numberSplit + i]);
    Serial.println(data[numberSplit * numberStepper + 2 * numberSplit + i]);
    //    myStepper3.step(data[2 * numberSplit + i]);
    //    Serial.println("\n");
    RunOneStep(data[0 * numberSplit + i], data[1 * numberSplit + i], data[2 * numberSplit + i] );
    //    Serial.println(data[0 * numberSplit + i]);
    //    Serial.println(data[1 * numberSplit + i]);
    //    Serial.println(data[2 * numberSplit + i]);
    delayMicroseconds(5000);

    //    Serial.println("end");
    Serial.println("\n\n\n");
  }
  //  digitalWrite(Maybom, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1200);
  for (int i = 0; i < numberSplit; i++)
  {
    //      Serial.print("indexTheta: ");
    //      Serial.println(i);
    //      Serial.println("");
    //
    //      Serial.println("1");

    myStepper1.setSpeed(data[numberSplit * numberStepper + 0 * numberSplit + i]);
    //    Serial.println(data[numberSplit * numberStepper + 0 * numberSplit + i]);
    //    myStepper1.step(calStep(theta[0][i], theta[0][i + 1]));

    //    Serial.println("\n");
    //
    //    Serial.println("2");

    myStepper2.setSpeed(data[numberSplit * numberStepper + 2 * numberSplit + i]);
    //    Serial.println(data[numberSplit * numberStepper + 1 * numberSplit + i]);
    //    //    myStepper2.step(calStep(theta[1][i], theta[1][i + 1]));
    //    Serial.println("\n");
    //
    //    Serial.println("3");

    myStepper3.setSpeed(data[numberSplit * numberStepper + 1 * numberSplit + i]);
    //    Serial.println(data[numberSplit * numberStepper + 2 * numberSplit + i]);
    //    //    myStepper3.step(calStep(theta[2][i], theta[2][i + 1]));
    //    Serial.println("\n");
    RunOneStep(data[0 * numberSplit + i], -data[2 * numberSplit + i], -data[1 * numberSplit + i] );
    //    Serial.println(data[0 * numberSplit + i]);
    //    Serial.println(data[1 * numberSplit + i]);
    //    Serial.println(data[2 * numberSplit + i]);
    delayMicroseconds(5000);

    //    Serial.println("end");
    Serial.println("\n\n\n");
  }
  //  digitalWrite(Maybom, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(500);

}

void RunOneStep(float theta1, float theta2, float theta3)
{
  int thetaMin = min(min(abs(theta1), abs(theta2)), abs(theta3));
  int thetaMax = max(max(abs(theta1), abs(theta2)), abs(theta3));
  if (thetaMax - thetaMin > 80)
  {
    for (int i = 0; i < 20; i++)
    {
      myStepper1.step(theta1 / 20);
      myStepper2.step(theta2 / 20);
      myStepper3.step(theta3 / 20);
    }
  }
  else
  {
    for (int i = 0; i < 5; i++)
    {
      myStepper1.step(theta1 / 5);
      myStepper2.step(theta2 / 5);
      myStepper3.step(theta3 / 5);
    }
  }

}

void Homming()
{
  Serial.println("X, Y, Z plane is homing ...");

  myStepper1.setSpeed(MAXSPEED);
  myStepper2.setSpeed(MAXSPEED);
  myStepper3.setSpeed(MAXSPEED);
  long flagHome = -1; // Biến di chuyển
  while ((digitalRead(home_limit[0]) || digitalRead(home_limit[1]) || digitalRead(home_limit[2])))
  {
    Serial.println(digitalRead(home_limit[0]));

    if (digitalRead(home_limit[0]))
    {
      myStepper1.step( calStep(0, flagHome));

    }
    if (digitalRead(home_limit[1]))
    {
      myStepper2.step( calStep(0, -flagHome));
    }
    if (digitalRead(home_limit[2]))
    {
      myStepper3.step(calStep(-flagHome, 0));
    }




    //flagHome--;
  }
  Serial.println("Homing is completed!!");
  Serial.println("");
}

void Run(float theta[])
{
  for (int j = 0; j < numberSplit * 2; j++)
  {

    myStepper1.setSpeed(MAXSPEED / 2 );
    myStepper2.setSpeed(MAXSPEED / 2 );
    myStepper3.setSpeed(MAXSPEED / 2 );

    myStepper1.step(calStep(theta[0] / (2 * numberSplit), 0));
    myStepper2.step(calStep(theta[1] / (2 * numberSplit), 0));
    myStepper3.step(calStep(theta[2] / (2 * numberSplit), 0));
  }
}
void Restart()
{
  Serial.println("->->->->->");
  float RS[3] = { -30 - offset_0do, 30 + offset_0do, -30 - offset_0do};
  //double RS[3] = {30,30,30};
  Run(RS);
  delay(100);


  float RS1[3] = { 22, -22, 22};
  Run(RS1);
  Serial.println("<-<-<-<-<-");
  delay(100);


}
