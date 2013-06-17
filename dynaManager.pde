/**
* DynaManager
*
*http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_18
*http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm
*http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
*http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
*/

import controlP5.*; //Import the P5 Library for GUI interface elements (drop list, button)
import processing.serial.*; //import serial library to communicate with the ArbotiX

Serial sPort;  //serial object 
ControlP5 cp5;// p5 control object
DropdownList comList, curIdList, baudList, setIdList, setBaudList;//inintiate drop down boxes
Group g1, g2, workArea;
Accordion errorMessage;

int cnt = 0;//count for listbox items
int selectedPort;//currently selected port from comList drop down

//model numbers for 10-bit goal position dynamixels, in int (AX-12A, AX-18A, AX-12W, RX-24, RX-28, RX-64)
int[] tenBitDyna = {12, 18, 300, 24, 28, 64};
//model numbers for 12-bit goal position dynamixels, in int(MX-28, MX-64, MX-106, EX-106)
int[] twelveBitDyna = {29, 310, 320, 107};

int debug = 1;
int running = 0;

void setup() 
{
  size(700, 400);//size of application working area
  
  cp5 = new ControlP5(this);//intiaite controlp5 object
    
  ControlFont cf1 = new ControlFont(createFont("Arial",15));//intitiate new font 
 
   workArea = cp5.addGroup("workArea")
                .setPosition(100,100)
                .setBackgroundColor(color(0, 255))
                .setWidth(500)
                .setBackgroundHeight(200)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Connected on Port")
                .setVisible(false)
                ;  
  
  
  

/*********************THIRD ROW******************/
  
    //servo test button  
   cp5.addButton("testServoButton")
     .setValue(1)
     .setPosition(100,100)
     .setSize(100,50)
     .setCaptionLabel("Test Servo") 
     .moveTo(workArea)   
     ;
     
    //set ID button 
    cp5.addButton("setDynaButton")
     .setValue(1)
     .setPosition(210,100)
     .setSize(100,50)
     .setCaptionLabel("Set ID/Baud") 
     .moveTo(workArea)   
     ;
     
     //set and test button
    cp5.addButton("setDynaAndTest")
     .setValue(1)
     .setPosition(320,100)
     .setSize(100,50)
     .setCaptionLabel("Set ID/Baud & Test") 
     .moveTo(workArea)   
     ;
 
   //set font for test button
   cp5.getController("testServoButton")
     .getCaptionLabel()
     .setFont(cf1)     
     ;  
     
     
 /*********************SECOND ROW******************/ 
  //initialize set id dropdown properties
  setIdList = cp5.addDropdownList("setServoId")
          .setPosition(110, 60)
          .setSize(100,200)
          .setCaptionLabel("New Servo ID") 
     .moveTo(workArea)   
          ;
  customize(setIdList); // customize the id list
    
  //iterate through 255 integers, add each to the list (DYNAMXIEL IDs can be from 0-255
  for (int i=0;i<256;i++) 
  {
    ListBoxItem lbi = setIdList.addItem(str(i),i);
    lbi.setColorBackground(0xffff0000);
  }   
   


 
  //initialize baud dropdown properties and customize it
 setBaudList = cp5.addDropdownList("setServoBaud")
          .setPosition(220, 60)
          .setSize(100,200)
          .setCaptionLabel("New Servo Baud") 
     .moveTo(workArea)   
          ;
  customize(setBaudList); // customize the baud list
    
  //iterate through 255 integers, add each to the list (DYNAMXIEL IDs can be from 0-255


    ListBoxItem lbi3 = setBaudList.addItem("1000000",1);
    lbi3.setColorBackground(0xffff0000);
    ListBoxItem lbi4 = setBaudList.addItem("57600",34);
    lbi4.setColorBackground(0xffff0000);
 
 
 /*****************SECOND ROW**************/
   //initialize currentid list dropdown properties 
  curIdList = cp5.addDropdownList("currentServoId")
          .setPosition(110, 40)
          .setSize(100,200)
          .setCaptionLabel("Current Servo ID") 
     .moveTo(workArea)   
          ;
  customize(curIdList); // customize the id list
    
  //iterate through 255 integers, add each to the list (DYNAMXIEL IDs can be from 0-255
  for (int i=0;i<256;i++) 
  {
    ListBoxItem lbi = curIdList.addItem(str(i),i);
    lbi.setColorBackground(0xffff0000);
  }
 
 
  //initialize baud dropdown properties and customize it
  baudList = cp5.addDropdownList("currentServoBaud")
          .setPosition(220, 40)
          .setSize(100,200)
          .setCaptionLabel("Current Servo Baud") 
     .moveTo(workArea)   
          ;
  customize(baudList); // customize the baud list
    
  //iterate through 255 integers, add each to the list (DYNAMXIEL IDs can be from 0-255


    ListBoxItem lbi1 = baudList.addItem("1000000",1);
    lbi1.setColorBackground(0xffff0000);
    ListBoxItem lbi2 = baudList.addItem("57600",34);
    lbi2.setColorBackground(0xffff0000);
    
    cp5.addButton("scanDynaButton")
     .setValue(1)
     .setPosition(330,25)
     .setSize(60,15)
     .setCaptionLabel("Scan")  
     .moveTo(workArea)   
     ;
     
 
/*********************FIRST ROW******************/ 


  //initialize comlist dropdown properties
  comList = cp5.addDropdownList("comPort")
          .setPosition(10, 20)
          .setSize(60,200)
          .setCaptionLabel("Com Port")
          ;
  customize(comList); // customize the com port list
    
  //iterate through all the items in the serial list (all available serial ports) and add them to the 'comlist' dropdown
  for (int i=0;i<Serial.list().length;i++) 
  {
    ListBoxItem lbi = comList.addItem(Serial.list()[i], i);
    lbi.setColorBackground(0xffff0000);
  }

  //button for connecting to selected port ()
    cp5.addButton("connectSerial")
     .setValue(1)
     .setPosition(80,5)
     .setSize(60,15)
     .setCaptionLabel("Connect")
     ;
    cp5.addButton("disconnectSerial")
     .setValue(1)
     .setPosition(150,5)
     .setSize(60,15)
     .setCaptionLabel("Disconnect")
     ;
    cp5.addButton("autoSearch")
     .setValue(1)
     .setPosition(220,5)
     .setSize(80,15)
     .setCaptionLabel("Auto Search")
     ;
/*
     

 
 */
 
  
 /******************ALERTS*******************/
 
   g1 = cp5.addGroup("errorWindow")
                .setPosition(100,100)
                .setBackgroundColor(color(0, 255))
                .setWidth(400)
                .setBackgroundHeight(200)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Com Port Error")
                .setVisible(false)
                ;
    cp5.addButton("errorButton")
     .setValue(1)
     .setPosition(175,150)
     .setSize(50,20)
     .setCaptionLabel("      OK")     
     .moveTo(g1)   
     ;
     
  Textarea errorText = cp5.addTextarea("errorText")
                  .setPosition(100,20)
                  .setSize(220,100)
                  .setFont(createFont("arial",18))
                  .setLineHeight(18)
                  .setColor(color(128))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))   
                  .moveTo(g1)   
                  ;
  errorText.setText("Error Connecting to Port - try a different port or try closing other applications using the current port");    
    
    
     
   g2 = cp5.addGroup("dynaFoundGroup")
                .setPosition(100,100)
                .setBackgroundColor(color(0, 255))
                .setWidth(400)
                .setBackgroundHeight(200)
                .disableCollapse()
                .bringToFront()
                .setVisible(false)
                .setCaptionLabel("DYNAMIXEL Found")
                ;
    cp5.addButton("dynaFoundButton")
     .setValue(1)
     .setPosition(175,150)
     .setSize(50,20)
     .setCaptionLabel("      OK")     
     .moveTo(g1)   
     ;
     
  Textarea dynaFoundText = cp5.addTextarea("dynaFoundText")
                  .setPosition(100,20)
                  .setSize(220,100)
                  .setFont(createFont("arial",18))
                  .setLineHeight(18)
                  .setColor(color(128))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))   
                  .moveTo(g1)   
                  ;
  dynaFoundText.setText("DYNAMIXEL ID Found at baud type:");    
    
    
    
    
}//END SETUP


// function testServoButton will receive changes from 
// controller with name testServo
public void testServoButton(int theValue) 
{
  //println();
  int currentServoId = int(curIdList.value());


  testServo(currentServoId);
  //sendDynaPacket(1);
  //test the servo
  println("test!");
}

public void errorButton(int theValue) 
{
  g1.setVisible(false);
  
}  
  
public void setDynaButton(int theValue) 
{
  int newBaud = int(setBaudList.value());
  int newId = int(setIdList.value());
  int currentServoId = int(curIdList.value());

  setServoId(currentServoId, newId);
  setServoBaud(newId, newBaud);

  println("Set Dyna!!");
}
public void setDynaAndTest(int theValue) {
  //test the servo
  println("Set Dyna and Test!!!");
}

public void connectSerial(int theValue) 
{
    int serialPortIndex = (int)comList.value();
    try
    {
      sPort = new Serial(this, Serial.list()[serialPortIndex], 115200);
    }
    catch(Exception e)
    {
      if(debug ==1){println("Error Opening Serial Port");}
      g1.setVisible(true);
    }
    
    if(pingArbotix() == 1)
    {
      workArea.setVisible(true);
    }
    else
    {
      println("No Arbotix Found :(");
      sPort.stop();
    }
    


    
}

public void disconnectSerial(int theValue) 
{
 sPort.stop();   
 workArea.setVisible(false);
}

public void autoSearch(int theValue) 
{
  if(running ==1)
  {
    for (int i=0;i<Serial.list().length;i++) 
    {
      
      try
      {
        sPort = new Serial(this, Serial.list()[i], 115200);
      }
      catch(Exception e)
      {
        //if(debug ==1){println("Error Opening Serial Port");}
        //g1.setVisible(true);
      }
      
      if(pingArbotix() == 1)
      {
        workArea.setVisible(true);
        comList.setValue(i);
  
        break;
      }
      else
      {
        println("No Arbotix Found :(");
        sPort.stop();

      }
    }

  }


}
    
public void scanDynaButton(int theValue) {
  
   // int currentServoId = int(curIdList.value());
   
  
  int servoId =0;
  int i =0;
  
  setArbotixBaud(1000000);
  baudList.setValue(1);
  while(servoId == 0 && i<30)
  {
    if(checkId(i) == i)
    {
      servoId = i; 
      curIdList.setValue(i);
    }
    i++;
  }
  
    i = 0;
  if( servoId ==0)
  {
  
    setArbotixBaud(57600);
    baudList.setValue(34);
  }
  while(servoId == 0 && i<30)
  {
    if(checkId(i) == i)
    {
      servoId = i; 
      curIdList.setValue(i);
    }
    i++;
  }
  
    i = 30;
  if( servoId ==0)
  {
    setArbotixBaud(1000000);
    baudList.setValue(1);
  }
  while(servoId == 0 && i<30)
  {
    if(checkId(i) == i)
    {
      servoId = i; 
      curIdList.setValue(i);
    }
    i++;
  }
  
    i = 30;
  if( servoId ==0)
  {
  
    setArbotixBaud(57600);
    baudList.setValue(34);
  }
  while(servoId == 0 && i<30)
  {
    if(checkId(i) == i)
    {
      servoId = i; 
      curIdList.setValue(i);
    }
    i++;
  }  
  
}  


//set servo to full clockwise, full counter, then center
public void testServo(int servoId)
{
  byte[] fullcw = {0,0};//full clockwise will be 0 for all servos
  byte[] fullccw ={0,0};
  byte[] centered ={0,0};
  int resolution = getDynaResolution(servoId);
  
  if(resolution == 12)
  {
    //convert integer posotions to bytes
    fullccw = intToBytes(4095);//4095 = full counter clockwise for 12 bit
    centered = intToBytes(2048);//2048 = centrered for 12 bit    
  }
  
  //else
  
  //assume 10 bit
  else if(resolution == 10);
  {
    //convert integer posotions to bytes
    fullccw = intToBytes(1023);//1023 = full counter clockwise for 12 bit
    centered = intToBytes(512);//512 = centrered for 12 bit   
  }
  
  if(resolution != 0)
  {
    setGoalPositionBytes(servoId, fullcw);
    delayMs(1500);
    setGoalPositionBytes(servoId, fullccw);
    delayMs(1500);
    setGoalPositionBytes(servoId, centered);
    delayMs(1500);  
  }
  else
  {
    println("Unknown Servo"); 
  }
}





void keyPressed() {
  /*
  if (key=='0') {
    // will activate the listbox item with value 5
    l.setValue(0);
  }
  if (key=='1') {
    // set the height of a listBox should always be a multiple of itemHeight
    l.setHeight(210);
  } 
  else if (key=='2') {
    // set the height of a listBox should always be a multiple of itemHeight
    l.setHeight(120);
  } 
  else if (key=='3') {
    // set the width of a listBox
    l.setWidth(200);
  }
  else if (key=='i') {
    // set the height of a listBoxItem, should always be a fraction of the listBox
    l.setItemHeight(30);
  } 
  else if (key=='u') {
    // set the height of a listBoxItem, should always be a fraction of the listBox
    l.setItemHeight(10);
    l.setBackgroundColor(color(100, 0, 0));
  } 
  else if (key=='a') {
    int n = (int)(random(100000));
    l.addItem("item "+n, n);
  } 
  else if (key=='d') {
    l.removeItem("item "+cnt);
    cnt++;
  } else if (key=='c') {
    l.clear();
  }
  
  */
}

void controlEvent(ControlEvent theEvent) {
  // ListBox is if type ControlGroup.
  // 1 controlEvent will be executed, where the event
  // originates from a ControlGroup. therefore
  // you need to check the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  if (theEvent.isGroup()) 
  {
    // an event from a group e.g. scrollList
    println(theEvent.group().value()+" from "+theEvent.group());
  }
  
  if(theEvent.isGroup() && theEvent.name().equals("currentServoBaud"))
  {
    int baud =(int)theEvent.group().value();
    if(  baud == 1)
    {
      setArbotixBaud(1000000);
    }
    else if(baud == 34)
    {
      setArbotixBaud(57600);
    }
    

  }
    
  if(theEvent.isGroup() && theEvent.name().equals("comPort"))
  {
    /*
    int test = (int)theEvent.group().value();
    selectedPort = test;
    sPort = new Serial(this, Serial.list()[selectedPort], 115200);
    setArbotixBaud(1000000);
    */
    //sPort.clear();//temporary 'clearing' the serial port since the arbotix sends out a ping on startup?
  
    //println("testo "+test);
  }
}

void draw() {
  running = 1;
  /*
  while(Comselected == true && serialSet == false)
  {
    port = new Serial(this, theport[Ss], 9600);
    serialSet = true;

  }
  */
  
   
   
  background(128);
  // scroll the scroll List according to the mouseX position
  // when holding down SPACE.
  if (keyPressed && key==' ') {
    //l.scroll(mouseX/((float)width)); // scroll taks values between 0 and 1
  }
  if (keyPressed && key==' ') {
    //l.setWidth(mouseX);
  }
}

void customize(DropdownList ddl) {
  // a convenience function to customize a DropdownList
  ddl.setBackgroundColor(color(190));
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;


  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));

  



}





/************************************
 * getDynaPacket
 *
 * Parameters
 * int id : id # of the DYNAMIXEL servo being queried 
 * int regstart : the starting register that data is being request from. With a length of '1'
 *   only this register will be returned 
 * int length : number of registers to return - useful for multi-byte properties like position/speed
 *
 * Returns:
 *
 ************************************/

byte[] getDynaPacket(int id, int regStart, int length)
{
  
  int[] responseArray = new int[6+length];// array to hold all members of the serial response. 0->header 1-> header 2->id 3->length 4->error then length * parameters and fiannly the checksum 
  byte[] responseParameter = new byte[length]; // array that holds the returned data from the registers only 
  int inByte = 0;//current byte we're reading
  byte localChecksum = 0; //a byte to hold the localChecksum calculated on the processing side, to compare against the checksum receieved
  byte checksumFromDyna = 0;//checksum returned from the dynamixel
  
  sPort.clear();//clear out any lefover data
  //sendDynaPacket(id, 2, regStart, length); //send a packet with instruction '2'(read) to request register data
  delayMs(100);
  if ( sPort.available() > 0)   // If data is available,
  {
    inByte = sPort.read();
  }

  
  if(inByte == 0xff)
  {
    responseArray[0] = inByte;
    for(int j = 1; j <6+length ; j++)
    {
      if ( sPort.available() > 0)   // If data is available,
      {
        inByte = sPort.read();
      responseArray[j] = inByte;
        print(inByte);
        print("-");
      }
      
    }   
     println(" ");
    if(length == 1)//if we're only requesting 1 packet, checksum correctly
    {
      localChecksum = byte(~((responseArray[2] +responseArray[3]+ responseArray[4] + responseArray[5]  ) % 256)); //compute checksum !(ID+length+instruction+parameter1+parameter2%256)
      checksumFromDyna = byte(responseArray[6]);   
   }
    
    else if(length ==2)//if we have 2 packets, add in the extra parameter to the checksum
    {
      localChecksum = byte(~((responseArray[2] +responseArray[3]+ responseArray[4] + responseArray[5] + responseArray[6] ) % 256)); //compute checksum !(ID+length+instruction+parameter1+parameter2%256)
      checksumFromDyna = byte(responseArray[7]);   

    }
  
   //fill responseParameter array from the response array (i.e. remove the leading bytes, just retrieve the requested data   
    for(int k = 0; k< length; k++)
    {
     //TODO:do a check to make sure casting is ok
     responseParameter[k] = byte(responseArray[5+k]);
     //print(responseParameter[k]);
     //print("-");
    
    }
    //println(" ");
    //print(" Checksum calc =  ");
   // println(hex(localChecksum));
   
  }
  
  else
  {
    //return(null);
    println("No header");
  }
  
   // sPort.clear();
    
    
    
   //check if the packet returned is valid - check that the id, length, and checksum match
   if ((responseArray[2] == id) && (responseArray[3] == (length+2)) && (checksumFromDyna == localChecksum) ) 
   {
    print("SUCCESS");
    print(" - ");
    print(responseParameter[0]);
    print(" - ");
    print(responseArray[2]);
    print(" - ");
    print(responseArray[3]);
    print(" - ");
    print(checksumFromDyna);
    print(" - ");
    print(localChecksum);
    print(" - ");
   // println(responseParameter[1]);
   } 
  
   else
   {
    print("CHECKSUM/ID/LENGTH BAD");
    print(" . ");
    print(id);
    print(" . ");
    print(responseParameter[0]);
    print(" . ");
    print(responseArray[2]);
    print(" . ");
    print(length);
    print(" . ");
    print(responseArray[3]);
    print(" . ");
    print(checksumFromDyna);
    print(" . ");
    print(localChecksum);
    println(" . ");
      responseParameter = null; 
    //  println("Error");
   }  
    //  println("moo");
    
  //  sPort.clear();
   sPort.clear();
   return(responseParameter);
    
}//END getDynaPacket




//pingDyna()
//regWriteDyna()
//actionDyna
//resetDyna
//syncWriteDyna




byte[] intToBytes(int convertInt)
{
  byte[] returnBytes = new byte[2]; // array that holds the returned data from the registers only 
  byte mask = byte(0xff);
  returnBytes[0] =byte(convertInt & mask);
  returnBytes[1] =byte((convertInt>>8) & mask);
  return(returnBytes);
  
}

//0 -> low byte 1 -> high byte
int bytesToInt(byte[] convertBytes)
{
  return((convertBytes[1]<<8)+convertBytes[0]);
}


 void delayMs(int ms){
  int time = millis();
  while(millis()-time < ms);
}




/************************************
 * readDynaPacket
 *
 * Parameters: 
 *            int numberOfRegisters: an interget that corresponds to how many registers/byted you are expecting in the response packet
 *                                  0 = response after a WRITE command, no registers
 *                                  1 = response after a READ command, one register/byte
 *                                  2 = response after a READ command, two registers/bytes
 *                                  etc.
 *
 * Returns: an array of bytes repsenting the dynamixel return packet
 *          If length = 0
 *          0->header 1-> header 2->id 3->length 4->error 6->checksum 
 *          If length = 1
 *          0->header 1-> header 2->id 3->length 4->error 5->parameter 6->checksum 
 *          If length = 2
 *          0->header 1-> header 2->id 3->length 4->error 5->parameter1 6->parameter2 7->checksum 
 *          etc.
 *
 *Description: This function will read an incoming DYNAMIXEL packet and return a byte array with the packet's contents.
 *             This function is usually run after a sendDynaPacket() call, whether to get a response from a READ command,
 *             or to confirm  a WRITE command.
 *             This function adds a small delay at the begining to account for latency from the ArbotiX Robocontroller
 *
 ************************************/

 
byte[] readDynaPacket(int numberOfRegisters)
{
  byte[] responseBytes = new byte[6+numberOfRegisters];// array to hold all members of the serial response.
  //int returnedBytes = 0; //actual number of returned bytes
  delayMs(100);//wait a period to ensure that the controller has responded 
  byte inByte = 0;
  
  if(debug==1){print("    Incoming Raw Packet from readDynaPacket():");}
  for(int i =0; i < 6+numberOfRegisters;i++)    // If data is available in the serial port, continute
  {
    if(sPort.available() > 0)
    {
      inByte = byte(sPort.readChar());
      responseBytes[i] = inByte;
      if(debug==1){ print(hex(inByte) + "-");} //debug 
    }
    else
    {
      if(debug==1){ print("NO-BYTE");} //debug 
    }
  }//end looking for bytes from packet
  if(debug==1){ println(" ");} //debug  

  return(responseBytes);
   
}//END readDynaPacket



/************************************
 * sendDynaPacket(byte, byte, byte[])
 *
 * Parameters: 
 *           byte id: id of the servo to send data to
 *           byte instruction: kind of instruction to send to servo
 *                              1-Ping
 *                              2-READ_DATA
 *                              3-WRITE_DATA
 *                              4-REG_WRITE
 *                              5-ACTION
 *                              6-RESET
 *                              83-SYNC WRITE
 *                              more info at  http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
 *           byte parameters: parameters for the packet.
 *                            Writes : [0] ->starting register
 *                                     [1] -> first value (at register [0])
 *                                     [2] -> first value (at register [0]+1)(optional)
 *                                     etc(optional)
 *                             Reads : [0] ->starting register
 *                                     [1] -> length (how many registers to read)
 *
 *
 * Returns: Nothing
 *
 * Description: This function will send a DYNAMIXEL packet.
 *             This function is usually run in conjunction with readDynaPacket(), which will read the response to both READ and WRITE commands
 *                  However it can be used on its own for commands that do not return packet data (multi broadcast commands, sync write)
 *
 ************************************/
void sendDynaPacket(byte id, byte instruction, byte[] parameters)
{
  byte checksum = 0;
  int numberOfParameters = parameters.length;  //find out how many parameters to send out
  byte length = byte(2+numberOfParameters);    //packet length = 2 + number of parameters 

  //add up the values of any parameters for the checksum
  for(int i = 0; i < numberOfParameters; i++)
  {
    checksum = byte(checksum + parameters[i]);
  }
  //do the rest of the checksum math checksum 
  //!(ID+length+instruction+parameters%256)  
  checksum = byte(~((id + length + instruction + checksum) % 256)); 
 
  //send serial packet to DYNAMIXEL chain  
  sPort.write(0xff);//header byte 2, always 255
  sPort.write(0xff);//header byte 2, always 255
  sPort.write(id);//servo ID
  sPort.write(length);//packet length  
  sPort.write(instruction);//instruction
  for(int i = 0; i < numberOfParameters; i++)
  {
    sPort.write(parameters[i]);//parameters
  }
  sPort.write(checksum); 
  
  
  if(debug ==1)
  {
    print("    Outgoing Raw Packet from sendDynaPacket():");
    print("");
    print(hex(byte(255)));
    print("-");
    print(hex(byte(255)));
    print("-");
    print(hex(id));
    print("-");
    print(hex(length));
    print("-");
    print(hex(instruction));
    print("-");
    for(int i = 0; i < numberOfParameters; i++)
    {
      print(hex(parameters[i]));
    print("-");
    }      
    println(hex(checksum));
  }
}

/************************************
 * sendDynaPacket(int, int, int[])
 * 
 * Description: This function is a simple wrapper for  sendDynaPacket(byte, byte, byte[]). It will cast the incoming ints to bytes
 *              and send them to  sendDynaPacket(byte, byte, byte[]). See that function for more information
 *
 *  
 ************************************/
void sendDynaPacket(int id, int instruction, int parameters)
{
  sendDynaPacket(byte(id), byte(instruction), byte(parameters));
  if(debug ==1){println("sendDynaPacket int version-");}
}



/************************************
 * writeDynaReg(byte, byte, byte, byte[])
 *
 * Parameters: 
 *           byte id: id of the servo to send data to
 *           byte instruction: kind of instruction to send to servo
 *                              1-Ping
 *                              2-READ_DATA
 *                              3-WRITE_DATA
 *                              4-REG_WRITE
 *                              5-ACTION
 *                              6-RESET
 *                              83-SYNC WRITE
 *                              more info at  http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
 *           byte parameters: parameters for the packet.
 *                            Writes : [0] ->starting register
 *                                     [1] -> first value (at register [0])
 *                                     [2] -> first value (at register [0]+1)(optional)
 *                                     etc(optional)
 *                             Reads : [0] ->starting register
 *                                     [1] -> length (how many registers to read)
 *
 *
 * Returns: int - returns a '0' if the write was successful. If the response packet comes back with an error or bad checksum, the function returns the error code
 *          The error code is generated bit-by-bit, with a '1' for each bit 
 *          0- input voltage error
 *          1- angle limit error
 *          2- overheating error
 *          3- range error
 *          4- checksum error
 *          5- overload error
 *          6- instruction error
 *          7- none
 *
 * Description: This function will send a DYNAMIXEL packet.
 *             This function is usually run in conjunction with readDynaPacket(), which will read the response to both READ and WRITE commands
 *                  However it can be used on its own for commands that do not return packet data (multi broadcast commands, sync write)
 *
 ************************************/
 int writeDynaReg(byte servoId, byte[] parameters)
{

  byte[] response = new byte[6];    //the response after a write is fixed to 6 bytes
                                     //0-header, 1-header, 2-id, 3-length, 4-error, 5-checksum
  byte localChecksum = 0;//local checsum calculated by processing
  int numberOfRegisters = 0;//length is 0 because its a return packet for the write
  
  sendDynaPacket(servoId, byte(3), parameters); //send a dynamixel packet to the id# servoid with parameters. '3' is the 'WRITE' instruction
  response = readDynaPacket(numberOfRegisters);//get response of write to check for errors
  
  //check headers
  if(response[0] == byte(255) && response[1] == byte(255))
  {
    //check if id returned is the same as the one sent out
    if(response[2] == servoId)
    {
      localChecksum = byte(~((response[2]+response[3]+response[4]) % 256)); //calculate checksum locally
      //check if calculated checksum matches the one in the packet
      if(localChecksum == response[5])
      {
        //if the error packet is empty return a '1' for a successful packet send
        if(response[4] == 0)
        {
          return(1);
        }
        //todo: add debugging code to print out specific error codes
        else 
        {
          if(debug == 1){println("writeDynaReg: return packet has an error code:"+response[4]);}
          return(response[4]);
        }

      }
      else if(debug == 1){println("writeDynaReg: return packet checksum does not match");}
    }
    else if(debug == 1){println("writeDynaReg: return packet id does not match command issued ");}


  }
  else if(debug == 1){println("writeDynaReg: return packet has no headers");}


  return(1);
  
}//end writeDynaReg



/************************************
 * writeDynaReg(int, int, int, byte[])
 *
 * Description: This function is a simple wrapper for  writeDynaReg(byte,byte, byte, byte[]). It will cast the incoming ints to bytes
 *              and send them to  writeDynaReg(byte,byte, byte, byte[]). See that function for more information
 *
 ************************************/
int writeDynaReg(int servoId, byte[] parameters)
{
  if(debug ==1){println ("writeDynaReg int version-");}
  return(writeDynaReg(byte(servoId),parameters));
}//end writeDynaReg


/************************************
 * readDynaReg(byte, byte[])
 *
 * Parameters: 
 *           byte id: id of the servo to send data 
 *           byte parameters: parameters for the packet.
 *                             Reads : [0] ->starting register
 *                                     [1] -> length (how many registers to read)
 *
 *
 * Returns: byte[] - array with the register data requested
 *
 * Description: This function will send a DYNAMIXEL packet that requests a certain number of register data from a specific servo
 *
 ************************************/
 byte[] readDynaReg(byte servoId, byte[] parameters)
{
  int numberOfRegisters = parameters[1];//the second parameter will tell how many registers we are trying to retrieve

  byte[] response = new byte[6+numberOfRegisters];   //the response after willl vary depending on how many registers we are requesting
                                                     //1 reg, 0-header, 1-header, 2-id, 3-length,4-error, 5-parameter1, 6-checksum
                                                     //2 reg, 0-header, 1-header, 2-id, 3-length,4-error, 5-parameter1, 6-parameter2, 7-checksum
                                                     //etc
  byte localChecksum = 0;//local checsum calculated by Processing
  byte[] registerData = new byte[numberOfRegisters];
  
  sendDynaPacket(servoId, byte(2), parameters); //send a dynamixel packet to the id# servoid with parameters. '2' is the 'READ' instruction
  response = readDynaPacket(numberOfRegisters);//get response of write to check for errors
  
  //check headers
  if(response[0] == byte(255) && response[0] == byte(255))
  {
    //check if id returned is the same as the one sent out
    if(response[2] == servoId)
    {
      //add up the values of the parameters for the checksum
      for(int i = 0;i<numberOfRegisters;i++ )
      {
        localChecksum = byte(localChecksum + response[i+5]); //offset by 5 since the parameters start at 5
        registerData[i] = response[i+5];//make use of this loop to also move register data to the return array
      }      
      localChecksum = byte(~((response[2]+response[3]+response[4] +localChecksum) % 256)); //do the rest of the math to calculate the checksum locally
      
      //check if calculated checksum matches the one in the packet
      if(localChecksum == response[5+numberOfRegisters])
      {
        //if the error packet is empty return a '1' for a successful packet send
        if(response[4] == 0)
        {
          return(registerData);//array formed above
        }
        //todo: add debugging code to print out specific error codes
        else 
        {
          if(debug == 1){println("readDynaReg: return packet has an error code:"+response[4]);}
          return(null);
        }

      }
      else if(debug == 1){println("readDynaReg: return packet checksum does not match");}
    }
    else if(debug == 1){println("readDynaReg: return packet id does not match command issued ");}


  }
  else if(debug == 1){println("readDynaReg: return packet has no headers");}


  return(null);
  
}//end readDynaReg



/************************************
 * readDynaReg(int, byte[])
 *
 * Description: This function is a simple wrapper for  readDynaReg(byte,byte[]). It will cast the incoming ints to bytes
 *              and send them to  readDynaReg(byte,byte[]). See that function for more information
 *
 ************************************/
byte[] readDynaReg(int servoId, byte[] parameters)
{
  if(debug ==1){println ("readDynaReg int version-");}
  return(readDynaReg(byte(servoId),parameters));
}//end readDynaReg


int setServoBaud(int servoId, int servoBaud)
{
    byte[] parameters = {4, byte(servoBaud)};
    
    return(writeDynaReg(servoId, parameters));
}

int setServoId(int currentServoId, int newServoId)
{

   byte[] parameters = {3, byte(newServoId)};
    
    return(writeDynaReg(currentServoId, parameters));
    
}


int setGoalPositionBytes(int servoId, byte[]goalPosition)
{
  byte[] parameters = {byte(30), goalPosition[0], goalPosition[1]};//the goal position is 30, so create a paramater array that starts with 30 and ends with the goal position data 
  writeDynaReg(servoId, parameters);
  return(1);
}
int setGoalPositionDegree(int servoId, int goalPositionDeg)
{
  
  return(1);
}
int setGoalPositionInt(int servoId, int goalPositionInt)
{
  
  return(1);
}


int setTorqueOn(int servoId)
{
  
  byte[] onVal = {byte(1)};
  //writeDynaReg(servoId, 24,onVal);
  return(1);

}
int setTorqueOff(int servoId)
{
  
  byte[] offVal = {byte(0)};
  //writeDynaReg(servoId, 24,offVal);
  return(1);

}

int setLedOn(int servoId)
{
  
  byte[] onVal = {byte(1)};
  //writeDynaReg(servoId, 25, onVal);
  return(1);

}
int setLedOff(int servoId)
{
  
  byte[] offVal = {byte(0)};
  //writeDynaReg(servoId, 25,offVal);
  return(1);

}



int checkId(int servoId)
{
  byte[] returnedId = {0};
  byte[] parameters = {3, 1};
  returnedId=readDynaReg(servoId, parameters);
  //returnedId = getDynaPacket(servoId,3,1);   
  if(returnedId != null)
  {
    return(returnedId[0]);
  }
  else
  {
     return(0); 
  }
}


/************************************
 * getDynaResolution(int)
 *
 * Parameters: int servoId: id of the servo to get resolution info
 *
 * Returns: int - resolution of the goal position, 10 = 10-bit(0-1023), 12 = 12 bit(0-4096) and  0 = unknown servo
 *
 * Description: This function will get the model number of the servo, then compare it against a list of known servos to determine
 *              Whether the servo has a 10 bit or 12 bit positional encoder. In addition we assume that all 12-bit servos have a
 *              300 degree reach in joint mode, while 12 bit servos go to 360 degrees
 *
 ************************************/
int getDynaResolution(int servoId)
{
 
  int[] tenBitDyna = {12, 18, 300, 24, 28, 64}; //model numbers for 10-bit goal position dynamixels, in int (AX-12A, AX-18A, AX-12W, RX-24, RX-28, RX-64)
  int[] twelveBitDyna = {29, 310, 320, 107};    //model numbers for 12-bit goal position dynamixels, in int(MX-28, MX-64, MX-106, EX-106)
  byte[] modelByte = {0,0};                     //bytes that represent the DYNAMIXEL model
  int modelInt = 0;                             //model number as a single int
  int resolution = 0;                           //resolution, 0 = unknown, 10=10bit, 12=12bit
  byte[] parameters = {0, 2};                   //model number starts at register 0 and 2 registers long
    
  modelByte=readDynaReg(servoId, parameters);   //get the model number

  if(modelByte != null)
  {
    modelInt = bytesToInt(modelByte);
  } 
  
  for(int i = 0; i< tenBitDyna.length;i++)
  {
    if(modelInt == tenBitDyna[i])
    {
      resolution = 10;
    }
  }
  
  for(int i = 0; i< twelveBitDyna.length;i++)
  {
    if(modelInt == twelveBitDyna[i])
    {
      resolution = 12;
    }
  }
  
  return(resolution); 
}






int setArbotixBaud(int baud)
{
  byte[] parameters = {byte(4), 0};//baud is at register 4, placeholder 0 for baud
  byte id = byte(0xfd);//253 = id to set Arbotix parameters

  //this function accepts the baud as a 'real' baud number. We must translate this into a baud rate in terms of the DYNAMIXEL register
  //TODO: populate other bauds  
  if(baud == 1000000)
  {
    parameters[1] = 1;
  }
  else if(baud == 57600)
  {
    parameters[1] = 34;  
  }
  else if(baud == 57142)
  {
     parameters[1] = 34;
  }
  
  else//no recognizable baud
  {
     return 0;//error will robinson
  }
  
  return(writeDynaReg(id, parameters));
  
  
   
    
  //TODO: Add code to query Arbotix and confirm that the Arbotix had changed to the requested baud
 
}

int pingArbotix()
{
  byte[] parameters = {byte(4), 1};//baud is at register 4, placeholder 0 for baud
  byte id = byte(0xfd);//253 = id to set Arbotix parameters
  byte[] response = new byte[6];
  sendDynaPacket(id, byte(3), parameters); //send a dynamixel packet to the id# servoid with parameters. '3' is the 'WRITE' instruction
  response = readDynaPacket(0);//get response to see if arbotix is present
  
  //check headers
  if(response[0] == byte(255) && response[1] == byte(255) && response[2] == byte(253) && response[3] == byte(2) && response[4] == byte(0) && response[5] == byte(0))
  {
    return(1);
  }
   
   else 
   {
    return(0);
   }
  
  
  
}
