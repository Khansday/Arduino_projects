int ch[] = {'a','b','c'}; //this is an array of int that contain the ASCII value of letters

char *c = "any string"; //this is the use of a pointer that adresses a string

char string[] = "I am happy"; //this is an array of character

void print_ch(){  //function to print the int array
    for (int i=0;i<3;i++){  //iterate over the array
    Serial.println(char(ch[i])); //print the element by casting into a char
  }

  for (int i=0;i<3;i++){  //iterate over the array
    Serial.println((ch[i])); //print the ASCII of the element
  }
}

void print_c(){ //function to print the pointer
   Serial.println(c); //print the string
   }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //initialise serial monitor
  delay(30);    //giving time to complete the initialistaion
  
  print_ch();   //print the int array
  
  print_c();    //print the pointer
  
  Serial.println(string); //print the array of characters
}

void loop() {
  // put your main code here, to run repeatedly:

}
