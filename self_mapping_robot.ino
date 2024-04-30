 
// Include NewPing Library for HC-SR04 sensor
#include <NewPing.h>
// Include Arduino Wire library for I2C
#include <Wire.h>
#define SLAVE_ADDR 9
// Room size (limited by RAM)
const int N=30, M=30; //N,M  room length and width

// Unit of room can be: unkwown, path or obstacle
const char u = 'u'; // unknown: not explored yet
const char p = 'p'; // path: free to explore
const char o = 'o'; // obstacle: robot cannot go there

// Direction for robot backracing 
const char n='n', e='e', s='s', w='w'; //north, south, east, west: tells the direction from which Robot visits a new unit of surface

// Room to explore
char room[N][M];

// Robot initial position
int x = 15; //N/2;
int y = 15; //M/2;

// Robot orientation, to know which sonars point to the North, East, South and West after that robot has turned
char orientation[4] = {'f','r','d','l'}; //Like a clock: forward, right, down, left

boolean finished_acquisition_sonars = 0; // True when all sonars end acquisition. If true, robot can move


//// ARDUINO BOARD ////

int motor1fwd = 2;
int motor1bwd = 3;
int motor2fwd = 4;
int motor2bwd = 5;

int motor3fwd = 8;
int motor3bwd = 9;
int motor4fwd = 10;
int motor4bwd = 11;

const int ledPinUp = 6;
const int ledPinDown = 7;
const int ledPinLeft = 12; 
const int ledPinRight = 13;

void setup() {
  // put your setup code here, to run once:
  // Initialize Room matrix: initially all unit of surfaces are unknown
  
  for (int i=0; i<N; i++)
  {
    for (int j=0; j<M; j++)
    {
    room[i][j]=u;
    }
  }

  // while robot initial position is path
  room[x][y] = p;

  Serial.begin(9600);
  Wire.begin();
  pinMode(ledPinRight, OUTPUT);
  pinMode(ledPinUp, OUTPUT);
  pinMode(ledPinDown, OUTPUT);
  pinMode(ledPinLeft, OUTPUT);

  Hurra();

  //Print map once before starting of exploration
//  PrintMapPretty();
}

void loop() {
  // put your main code here, to run repeatedly:
 //Check if robot is out of room, which is not desiderable due to limited RAM. If out of room, it will stop.
 if (x == 0 || y == 0 || x == N || y == M)
 {
   Serial.print("Robot out of matrix!");
   SLEEP();
 }
 
 if (Goal() != 1) // Room not completely explored yet
 {
  finished_acquisition_sonars = 0; // distances not acquired yet
  See(); // look around robot's position (x,y) and update map
  
  if (finished_acquisition_sonars == 1)
  {
   Move(); // explore
  }
 }
  
 else if (Goal() == 1) // Room completely explored
 {
   Serial.println("Room completly explored!"); //F to use less ram memory, since string is saved on Flash
   Hurra();
   Hurra();
   Hurra();
   //PrintMap();
   SLEEP(); 
 }
 
// PrintMapPretty(); //just a print of saved map, but in a fashionable way
}

byte readI2C(int address) {
  // Define a variable to hold byte of data
  byte bval ;
  long entry = millis();
  // Read one byte at a time
  Wire.requestFrom(address, 1); 
  // Wait 100 ms for data to stabilize
  while (Wire.available() == 0 && (millis() - entry) < 100)  Serial.print("Waiting");
  // Place data into byte
  if  (millis() - entry < 100) bval = Wire.read();
  return bval;
}

int Goal() 
{
  // loop to check if there are clear places close to unknown places, so that goal is not reached yet (returns 0)

  for(int i=1; i<N; i++) //N,M room length and width
  {
    for(int j=1; j<M; j++)
    {
      if (room[i][j] == p) //consider only clear positions and see if the neighbor positions are unexplored, i.e. there are others ways to explore
      {
        if ( (room[i-1][j] == u) || (room[i-1][j] == n) || (room[i-1][j] == e) || (room[i-1][j] == s) || (room[i-1][j] == w) )//North
        {return 0;}
        
        else if ( (room[i][j+1] == u) || (room[i][j+1] == n) || (room[i][j+1] == e) || (room[i][j+1] == s) || (room[i][j+1] == w) ) //East
        {return 0;}
        
        else if ( (room[i+1][j] == u) || (room[i+1][j] == n) || (room[i+1][j] == e) || (room[i+1][j] == s) || (room[i+1][j] == w) ) //South
        {return 0;}

        else if ( (room[i][j-1] == u) || (room[i][j-1] == n) || (room[i][j-1] == e) || (room[i][j-1] == s) || (room[i][j-1] == w) ) //West
        {return 0;}
      }
    }
  }

  // if did not return anything before, the room is completely explored! Hurra!
 
  //  we return 1 to say that this is the end (...with the voice of Jim Morrison) to the main function
  return 1;
}

//SEE: look at one place around the current positions (x,y) and update the room matrix
void See() 
{
  int bcount;
  byte distance[8];
    while (readI2C(SLAVE_ADDR) < 255) {
  // Until first byte has been received print a waiting message
    Serial.println("Waiting"); 
  }

  for(bcount = 0;bcount<8;bcount++){
     distance[bcount] = readI2C(SLAVE_ADDR);
     
  }
   for (int i = 0; i < 8; i++) {
    Serial.print(distance[i]);
    Serial.print("\t");
  }
  Serial.println();
  if (room[x-1][y] == u) //see North
  {
    room[x-1][y] = checkLayout(orientation[0], distance[0], distance[1]); // ACTIVATE_SONAR(orientation[0]); //save on map 'p' if position (x-1,y) is to be visited and 'o' if obstacle
    //PrintMap();
  }
  
  if (room[x][y+1] == u) //see East
  {
    room[x][y+1] = checkLayout(orientation[1], distance[2], distance[3]); // ACTIVATE_SONAR(orientation[1]); //save on map 'p' if position (x,y+1) is to be visited and 'o' if obstacle
    //PrintMap();
  }
  
  if (room[x+1][y] == u) //see South
  {
    room[x+1][y] = checkLayout(orientation[2], distance[4], distance[5]); // ACTIVATE_SONAR(orientation[2]); //save on map 'p' if position (x+1,y) is to be visited and 'o' if obstacle
    //PrintMap();
  }
  
  if (room[x][y-1] == u) //see West
  {    
    room[x][y-1] = checkLayout(orientation[3], distance[6], distance[7]); // ACTIVATE_SONAR(orientation[3]); //save on map 'p' if position (x,y-1) is to be visited and 'o' if obstacle
    //PrintMap();
  }

  finished_acquisition_sonars = 1;
}

char checkLayout(char which_sonar, byte distance1, byte distance2){
   int maximumRange = 60; //43.5 cm: size of one unit plus half unit

       if ( int(distance1) > maximumRange || int(distance2) > maximumRange ) //case of path, which meaans that the obsvered unit of surface is free
    {
      return 'p'; // p: "path"
    }
    
    else //case of obstacle, robot cannot go there
    {
     if(which_sonar=='f'){
       digitalWrite(ledPinUp, HIGH); // corresponing led turns on to allert: in this direction there is an obstacle
      delay(150);
      digitalWrite(ledPinUp, LOW);
     }else if(which_sonar=='r'){
        digitalWrite(ledPinRight, HIGH);
      delay(150);
      digitalWrite(ledPinRight, LOW);
     }else if(which_sonar=='d'){
       digitalWrite(ledPinDown, HIGH);
      delay(150);
      digitalWrite(ledPinDown, LOW);
     }else if(which_sonar=='l'){
       digitalWrite(ledPinLeft, HIGH);
      delay(150);
      digitalWrite(ledPinLeft, LOW);
     }
      return 'o'; // o: "obstacle"
    }
  
}
void Move()
{
  
  /* Move rules:
  if a position around is path (not visited yet and free) go there and assign direction to say where you come from:
  s: from South
  w: form West
  n: from North
  e: from East

  The order of priority for moving to is arbitrarily chosen as: Nort, East, South, West. 
    
  If no position free around (only visited (n,e,s,w) and obstacles), robot comes back following the previously assigned direction. 
  */

  // case of at least 1 path position
  if (room[x-1][y] == p)
  {
    room[x-1][y] = n;
    
    x--; //this updates robot position (global variable)
    MoveNorth();
  }
  
  else if (room[x][y+1] == p)
  {
    room[x][y+1] = e;
    
    y++;
    MoveEast();
  }

  else if (room[x+1][y] == p)
  {
    room[x+1][y] = s;
    
    x++;
    MoveSouth();
  }
  
  else if (room[x][y-1] == p)
  {
    room[x][y-1] = w;

    y--;
    MoveWest();
  }

  
  // case of no free path, so robot comes back using the direction previously saved
  else if ((room[x-1][y] != p) && (room[x][y+1] != p) && (room[x+1][y] != p) && (room[x][y-1] != p))
  {
    //read value in the (x,y) position and move following the rules to come back
    if (room[x][y] == n)
    {
      x++; //update robot position
      MoveSouth(); //turn robot 180°, turn orientation, go one step forward
    }
    
    else if (room[x][y] == e)
    {
      y--; //update robot position
      MoveWest(); //turn robot 90° anticlockwise, turn orientation, go one step forward 
    }
    
    else if (room[x][y] == s)
    {
      x--; //update robot position
      MoveNorth(); //go one step forward
    }
    
    else if (room[x][y] == w)
    {
      y++; //update robot position
      MoveEast(); //turn robot 90° clockwise, turn orientation, go one step forward
    }
  }
}
// Update orientation state (array) of robot after it rotates right, left or backwards (2 consecutive left turnings)
void rotate_to_left() //left rotation 
{
  char temp = orientation[0];
  for (int i=0; i<4; i++) 
  {
    orientation[i] = orientation[i+1];
  }
  orientation[3] = temp;
}

void rotate_to_right() //right rotation
{
  char temp = orientation[3];
  for(int i=3; i>=0; i--)
  {
    orientation[i] = orientation[i-1]; 
  }
  orientation[0] = temp;
}

//MOVEMENT
void MoveNorth()
{
  turn(orientation[0]);
}

void MoveEast()
{
  turn(orientation[1]);
}

void MoveSouth()
{
  turn(orientation[2]);
}

void MoveWest()
{
  turn(orientation[3]);
}

// Robot can turn right, left or backwards
void turn(char rotation_value)
{
  if (rotation_value == 'f') 
  {
    GO(); //do not turn, just go
    STOP();
  }
  
  if (rotation_value == 'r') //turn right
  {
    TURN_RIGHT();
    STOP();
    GO();
    STOP();
    rotate_to_right(); //update orientation state (array) of robot
  }
  
  else if (rotation_value == 'd') //turn backwards
  {
    TURN_LEFT();
    TURN_LEFT();
    STOP();
    GO();
    STOP();
    rotate_to_left(); 
    rotate_to_left(); //update orientation state (array) of robot 
  }
  
  else if (rotation_value == 'l')  //turn left
  {
    TURN_LEFT();
    STOP();
    GO();
    STOP();
    rotate_to_left(); //update orientation state (array) of robot
  }
}
void GO(){
  unsigned long timeStartRotation = millis(); // save rotation starting time  
  const unsigned long timeUp = 1151; // 1710 1823 Time to go ahead for 1 unit of room (30 cm by 30 cm). old: 1780
  while((millis() - timeStartRotation) < timeUp) //  
  {
  FORWARD();
  }
}

void TURN_RIGHT(){
  unsigned long timeStartRotationRight = millis(); // save rotation starting time  
  const unsigned long timeUpRight = 1650; //  1040
  while((millis() - timeStartRotationRight) < timeUpRight)  
  {
 ROTATE_RIGHT();
  }
}

void TURN_LEFT(){
  unsigned long timeStartRotation1 = millis(); // save rotation starting time  
  const unsigned long timeUp1 = 1650; // 995
  while((millis() - timeStartRotation1) < timeUp1) 
  {
  ROTATE_LEFT();
  }
}

void STOP(){
  moveMotor(1, 0);
  moveMotor(2, 0);
  moveMotor(3, 0);
  moveMotor(4, 0);
  delay(10);
}

void GO_BACKWARDS()
{
 //because when it turns it's not around its central axis.. damn servos
 unsigned long timeStartRotation = millis(); // save rotation starting time  
 const unsigned long timeUp = 1151; //Time to go ahead for 1 unit of room (30 cm by 30 cm). old: 1870
 while((millis() - timeStartRotation) < timeUp)
 {
  BACKWARD(); 
 }
}

void FORWARD(){
  moveMotor(1, 1);
  moveMotor(2, 1);
  moveMotor(3, 1);
  moveMotor(4, 1);
}

void BACKWARD(){
  moveMotor(1, 2);
  moveMotor(2, 2);
  moveMotor(3, 2);
  moveMotor(4, 2);
}

void RIGHT(){
  moveMotor(1, 1);
  moveMotor(2, 2);
  moveMotor(3, 2);
  moveMotor(4, 1);
}

void LEFT(){
  moveMotor(1, 2);
  moveMotor(2, 1);
  moveMotor(3, 1);
  moveMotor(4, 2);
}

void RIGHT_FORWARD(){
  moveMotor(1, 1);
  moveMotor(2, 0);
  moveMotor(3, 0);
  moveMotor(4, 1);
}

void RIGHT_BACKWARD(){
  moveMotor(1, 0);
  moveMotor(2, 2);
  moveMotor(3, 2);
  moveMotor(4, 0);
  }

  void LEFT_FORWARD(){
  moveMotor(1, 0);
  moveMotor(2, 1);
  moveMotor(3, 1);
  moveMotor(4, 0);
  }

  void LEFT_BACKWARD(){
  moveMotor(1, 2);
  moveMotor(2, 0);
  moveMotor(3, 0);
  moveMotor(4, 2);
  }
  
void ROTATE_RIGHT(){
  moveMotor(1, 1);
  moveMotor(2, 2);
  moveMotor(3, 1);
  moveMotor(4, 2);
}

void ROTATE_LEFT(){
  moveMotor(1, 2);
  moveMotor(2, 1);
  moveMotor(3, 2);
  moveMotor(4, 1);
}

int getMotorPin(int i, bool fwd){
  if(i==1){
    if(fwd){ 
    return motor1fwd;
       }else{
        return motor1bwd;
       }
  }

   else if(i==2){
    if(fwd){ 
    return motor2fwd;
       }else{
        return motor2bwd;
       }
  }

  else if(i==3){
    if(fwd){ 
    return motor3fwd;
       }else{
        return motor3bwd;
       }
  }

   else if(i==4){
    if(fwd){ 
    return motor4fwd;
       }else{
        return motor4bwd;
       }
  }
}

void moveMotor(int motor, int cmd){

    if(cmd==1){
       digitalWrite(getMotorPin(motor,true),   HIGH);
  digitalWrite(getMotorPin(motor,false), LOW);
    }else if(cmd==2){
       digitalWrite(getMotorPin(motor,true),    LOW);
    digitalWrite(getMotorPin(motor,false), HIGH);
    }else{
       digitalWrite(getMotorPin(motor,true),   LOW);
    digitalWrite(getMotorPin(motor,false), LOW);
    
  }
}
// Led blinking at start and when GOAL is reached 
void Hurra()
{
//  digitalWrite(ledPinUp, HIGH);
//  digitalWrite(ledPinRight, HIGH);
//  digitalWrite(ledPinDown, HIGH);
//  digitalWrite(ledPinLeft, HIGH);
//  delay(2000);
//  digitalWrite(ledPinUp, LOW);
//  digitalWrite(ledPinRight, LOW);
//  digitalWrite(ledPinDown, LOW);
//  digitalWrite(ledPinLeft, LOW);
//  delay(1000);
}


void SLEEP()
{
 // Goodnight Explorino, either you finished your job or you are out of the map
// LeftServo.write(96);  
// RightServo.write(95);
 delay(20000000);
}

//char ACTIVATE_SONAR(char which_sonar)
//{
//  // The size of an unit cell of the matrix, in centimeter. It should be bigger than robot size.
//  int maximumRange = 60; //43.5 cm: size of one unit plus half unit
//
//  // Read temperature and determine sound speed (not strictly necessary for reading sonar value, just to have fun :))
//  const float soundSpeed_0 = 33145;                 
//  float temperature = ((analogRead(tPin) * 5.0 / 1024.0) - 0.5) * 100 ;
//  float soundSpeed = (soundSpeed_0 + 62 * temperature);
//  
//  // Our sonars are in paralell, so each one waits 50ms for the previous sonar to end its acquisition to start a new one
//  delay(50); 
//  
//  // Send trigger impulse to sonar to make them do the measurement
//  digitalWrite(trigPin, LOW); 
//  delayMicroseconds(5);
//  digitalWrite(trigPin, HIGH); 
//  delayMicroseconds(10); 
//  digitalWrite(trigPin, LOW);
//  
//  // Now wait for response from the sonar
//  
//  if (which_sonar == 'f')
//  {
//    float durationUp = pulseIn(echoPinUp, HIGH)*soundSpeed/2000000;
//    
//    if ( durationUp > maximumRange ) //case of path, which meaans that the obsvered unit of surface is free
//    {
//      return 'p'; // p: "path"
//    }
//    
//    else //case of obstacle, robot cannot go there
//    {
////      digitalWrite(ledPinUp, HIGH); // corresponing led turns on to allert: in this direction there is an obstacle
////      delay(150);
////      digitalWrite(ledPinUp, LOW);
//      return 'o'; // o: "obstacle"
//    }
//  }
//  
//  else if (which_sonar == 'r')
//  {
//    float durationRight = pulseIn(echoPinRight, HIGH)*soundSpeed/2000000;
//    
//    if ( durationRight  > maximumRange ) return 'p';
//    
//    else
//    {
//      digitalWrite(ledPinRight, HIGH);
//      delay(150);
//      digitalWrite(ledPinRight, LOW);
//      return 'o';
//    }
//  }
//  
//  else if (which_sonar == 'd')
//  {
//    float durationDown = pulseIn(echoPinDown, HIGH)*soundSpeed/2000000;
//    
//    if ( durationDown > maximumRange ) return 'p';
//    
//    else
//    {
//      digitalWrite(ledPinDown, HIGH);
//      delay(150);
//      digitalWrite(ledPinDown, LOW);
//      return 'o';
//    }
//  }
//  
//  else if (which_sonar == 'l')
//  {
//    float durationLeft = pulseIn(echoPinLeft, HIGH)*soundSpeed/2000000;
//    
//    if ( durationLeft > maximumRange ) return 'p'; 
//    
//    else
//    {
//      digitalWrite(ledPinLeft, HIGH);
//      delay(150);
//      digitalWrite(ledPinLeft, LOW);
//      return 'o';
//    }
//  }
//}
