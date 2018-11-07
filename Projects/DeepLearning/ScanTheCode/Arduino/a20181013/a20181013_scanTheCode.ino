  /* Code developed by Yong Li */
  /* The code produces color sequences for the light buoy. The color sequence will randomize after four consecutive patterns*/
  
  /*Set Pins*/
  const int red = 4; 
  const int blue = 5;
  const int green = 6;
  
  //Button
  int button = 1;
  const int buttonPin = 1;
  
  //RGB for color pattern
  const int r=1;
  const int g=2;
  const int b=3;
  
  //Color Variables for Color Picker
  int color1;
  int color2;
  int color3;

  //All the High/Low Outputs
  int out1;
  int out2;
  int out3;
  
  int out4;
  int out5;
  int out6;
  
  int out7;
  int out8;
  int out9;
  
  //counter
  int counter;
  
void setup(){ //Set up pins
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
  
  pinMode(buttonPin, INPUT);
  //digitalWrite(buttonPin, HIGH);
}
void loop(){
  
  while(counter==4){ //counter checker to change randomize color sequence after 4 sequences
    while(button == 1){
      color1=random(1,4); //Chooses random color 
      color2=random(1,4); //Chooses random color
      color3=random(1,4); //Chooses random color
    
      if(color2==color1 || color3==color2){ //the same color CANNOT reappear.
        button=1;
        }
      else{
        button=0;
          }
    counter = 0;
    }
  }
  
  
  /*COLOR 1 */
  if(color1==r){    // if the color variable is chosen to be a certain color. The outputs will 
    out1=HIGH;
    out2=LOW;
    out3=LOW;
  }
  if(color1==b){
    out1=LOW;
    out2=HIGH;
    out3=LOW;
  }
  if(color1==g){
    out1=LOW;
    out2=LOW;
    out3=HIGH;
  }

  /*COLOR 2 */
  if(color2==r){
    out4=HIGH;
    out5=LOW;
    out6=LOW;
  }
  if(color2==b){
    out4=LOW;
    out5=HIGH;
    out6=LOW;
  }
  if(color2==g){
    out4=LOW;
    out5=LOW;
    out6=HIGH;
  }

  /*COLOR 3 */
  if(color3==r){
    out7=HIGH;
    out8=LOW;
    out9=LOW;
  }
  if(color3==b){
    out7=LOW;
    out8=HIGH;
    out9=LOW;
  }
  if(color3==g){
    out7=LOW;
    out8=LOW;
    out9=HIGH;
  }

  delay(1000);
  digitalWrite(red, out1);
  digitalWrite(blue, out2);
  digitalWrite(green, out3);
  delay(1000);
  digitalWrite(red, out4);
  digitalWrite(blue, out5);
  digitalWrite(green, out6);
  delay(1000);
  digitalWrite(red, out7);
  digitalWrite(blue, out8);
  digitalWrite(green, out9);
  delay(1000);
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  delay(1000);
  
  counter += 1; 
  if(counter == 4){ //When the counter reaches 4 turn button back to 1 to reset randomizer.
    button=1;
  }
}
