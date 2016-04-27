/****************************************************
* Trossen Robotics DYNAMIXEL Servo ID Program 
*
*
*
* This progam offers an interface to easily set and test the ID and Baud of various
* DYNAMIXEL Robot Servos using the ArbotiX Robocontroller.
*
* With this program you can
* -open a serial port comunication line with an ArbotiX running the ROS sketch
* -scan for a DYNAMIXEL ID 0-252 on bauds 57600 or 1000000
* -set the ID of a servo
* -baud will always be set to 1000000
* -send a value to the "Goal Position" register of the DYNAMIXEL, moving it
*
* Currently supported DYNAMIXELS include
* AX-12+
* AX-12A
* AX-12W
* AX-18A
* MX-28T
* MX-64T
* MX-106T
*
* The Following servos should work, using an RX bridge, but have not been tested
*  RX-24F
*  RX-28
*  RX-64
*  EX-106
*  MX-28R
*  MX-64R
*  MX-106R
*  
*
*
*
*Protocol and DYNAMIXEL reference
*  http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_18
*  http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm
*  http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
*  http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
*/

import controlP5.*; //Import the P5 Library for GUI interface elements (drop list, button)
import processing.serial.*; //import serial library to communicate with the ArbotiX


Serial sPort;            //serial object 
ControlP5 cp5;           // p5 control object
DropdownList serialList;    //inintiate drop down boxes for com port list
Group scanGroup;         //group for DYNAMIXEL scanning widgets
Group setGroup;          //group for DYNAMIXEL setting widgets
Group testGroup;         //group for DYNAMIXEL testing widgets
Group startupGroup;      //group for startup message
Group errorGroup;        //group for error messages
Textfield curIdField;         //current servo ID from sanner, non user editable


                  
Textfield NameField; //current servo model name, non user editable
Textfield newIdField;         //new ID to set the servo to, user editable
Textfield presentPositionField;//present position of servo, in interger form
Textfield dynaModelNameField;
Canvas knob;                  //canvas to hold custom knob shapes, to be attached to testGroup 
Textarea startupText;         //startup text that tells user to connect to the arbotix - also shows up after a disconnect
Textarea errorText;           //error text for error group
Textarea successSet;          //text that displays info on a successful set after setDyna has been pressed
Button scanDynaButton;
Button connectButton;
Button disconnectButton;
Button autoSearchButton;
int curIdMargin = 45;
int modelMargin = 36;
int newIdMargin = 0;
int positionMargin = 40;
int cnt = 0;                  //count for listbox items
int selectedPort;             //currently selected port from serialList drop down
int baudToSet = 1;            //set to 34 for 57600 baud 
int debug1 = 1;
int debug = 0;                //change to '0' to disable bedbugginf messages from the console, '1' to enable TODO:log debugging to a file, add option to enable debugging
int running = 0;              //enabled on draw(), used to avoid controlp5 from running functions immidealty on startup
int servoScanned = 0;         //0 = no servo attached, 300 = 300 degree/10bit capable servo connected, 360 = 360degree/12 bit capable servo connected
int knobClickState = 0;       //0 = no knob interaction ,1 = knob has been clicked on and is being held 
int knobLimited = 0;          //whether the knob will do a full 360(0, not limited) or just 300(0, limited) degrees. 
float mouseRad = 0;           //angle the mouse is at from center, in radians
int xRel;                     //mouse x position, realtive to the center of the knob
int yRel;                     //mouse y position, realtive to the center of the knob
int time = 0;                 //holds the time of the last time the servo and arbotix connections were checked.
PImage img;                   //image object for TR logo
//model numbers for 10-bit goal position dynamixels, in int (AX-12A, AX-18A, AX-12W, RX-24, RX-28, RX-64)
int[] tenBitDyna = {12, 18, 300, 24, 28, 64};
//model numbers for 12-bit goal position dynamixels, in int(MX-28, MX-64, MX-106, EX-106)
int[] twelveBitDyna = {29, 310, 320, 107};
int packetRepsonseTimeout = 6500;
int lastPositionTime ;
void setup() 
{
  size(220, 443);//size of application working area
  img = loadImage("TRheaderLogo.png");  // Load the TR logo
  cp5 = new ControlP5(this);//intiaite controlp5 object   
  ControlFont cf1 = new ControlFont(createFont("Arial",15));//intitiate new font for buttons
 
//Instantiate groups to hold our various control items. Groups will make it easy to organize and hide/show control items 
  scanGroup = cp5.addGroup("scanGroup")
                .setPosition(10,50)
                .setBackgroundColor(color(0, 255))
                .setWidth(200)
                .setBackgroundHeight(100)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Scan Options")
                .setVisible(false)
                .hideBar()
                ;  
  setGroup = cp5.addGroup("setGroup")
                .setPosition(10,170)
                .setBackgroundColor(color(0, 255))
                .setWidth(200)
                .setBackgroundHeight(100)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Set Servo ID")
                .setVisible(false)
                ;    
  testGroup = cp5.addGroup("testGroup")
                .setPosition(10,290)
                .setBackgroundColor(color(0, 255))
                .setWidth(200)
                .setBackgroundHeight(90)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Test Servo")
                .setVisible(false)               
                ;  

  cp5.getGroup("testGroup").addCanvas(new knobCanvas());//attach the knobCanvas canvas to the test group
  
/*********************SCAN GROUP******************/
  //scan button
  scanDynaButton = cp5.addButton("scanDynaButton")
   .setValue(1)
   .setPosition(10,10)
   .setSize(70,70)
   .setCaptionLabel("  Scan")  
   .moveTo(scanGroup)   
   ;
   
  //set font for the scan button                
  cp5.getController("scanDynaButton")
     .getCaptionLabel()
     .setFont(cf1)     
     ;     
   
  //text field that will hold the current servo's ID (when nothing is connected or during scanning, this field will show a message)
  curIdField = cp5.addTextfield("currentServoId")
                  .setPosition(90,10)
                  .setAutoClear(false)
                  .lock()
                  .setCaptionLabel("Servo ID") 
                  .setWidth(100)
                  .setValue("No Servo Connected")
                  .moveTo(scanGroup)   
                  ;   

  //text field that will hold the current servo's Model (when nothing is connected or during scanning, this field will show nothing)   
  dynaModelNameField = cp5.addTextfield("dynaModelNameField")
                          .setPosition(90,50)
                          .setAutoClear(false)
                          .lock()
                          .setCaptionLabel("Servo Model") 
                          .setWidth(100)
                          .setValue("No Servo Connected")
                          .moveTo(scanGroup)   
                          ;   
   

   
/*********************SET GROUP******************/
   
  //initialize an editable text field to hold the ID that the user would like to set the servo to
  newIdField = cp5.addTextfield("setServoId")
                  .setPosition(10, 10)
                  .setWidth(70)
                  .setCaptionLabel("New Servo ID") 
                  .setValue("1")
                  .moveTo(setGroup)   
                    ;
  newIdField.valueLabel().style().marginLeft = newIdMargin; 

        
    //initialize button that will set the ID and baud of the servo
  cp5.addButton("setDynaButton")
     .setValue(1)
     .setPosition(90,10)
     .setSize(100,20)
     .setCaptionLabel("Set ID/Baud") 
     .moveTo(setGroup)   
       ;
  //set font for set dyna button             
  cp5.getController("setDynaButton")
     .getCaptionLabel()
    .setFont(cf1)     
     ;          

/*********************TEST GROUP******************/
   //initlaize button that will send a positional command to the DYNAMIXEL to center the horn
   cp5.addButton("centerDyna")
     .setValue(1)
     .setPosition(95,55)
     .setSize(100,20)
     .setCaptionLabel("         Center Servo") 
     .moveTo(testGroup)   
     .setVisible(true)
   ;
   
   //initlialize thext field that will report the DYNAMIXEL's current positon
   presentPositionField = cp5.addTextfield("presentPositionField")
                             .setPosition(95,15)
                             .setAutoClear(false)
                             .lock()
                             .setCaptionLabel("Present Position") 
                             .setWidth(100)
                             .setValue("")
                             .moveTo(testGroup)   
                             .setVisible(true)
                             ;  
                             
  presentPositionField.valueLabel().style().marginLeft = positionMargin; 



 /******************ALERTS*******************/
  //initialize group that will be shown on startup with instructions
  startupGroup = cp5.addGroup("startupGroup")
                .setPosition(10,170)
                .setBackgroundColor(color(0, 255))
                .setWidth(200)
                .setBackgroundHeight(100)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Not Connected")
                .setVisible(true)
                ;
  //initialize text area instruction text for startup text              
  startupText = cp5.addTextarea("startupText")
                  .setPosition(10,10)
                  .setSize(180,70)
                  .setFont(createFont("arial",15))
                  .setLineHeight(15)
                  .setColor(color(128))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))   
                  .moveTo(startupGroup)   
                  .setVisible(true)
                  ;
  //set startupText text
  startupText.setText("No ArbotiX Connected - Please choose a COM port and connect");    
    

  //initialize group to hold errors       
  errorGroup = cp5.addGroup("errorWindow")
                .setPosition(10,170)
                .setBackgroundColor(color(0, 255))
                .setWidth(200)
                .setBackgroundHeight(100)
                .disableCollapse()
                .bringToFront()
                .setCaptionLabel("Error")
                .setVisible(false)
                ;
                
  //initialize button to dismiss errors              
  cp5.addButton("errorButton")
     .setValue(1)
     .setPosition(75,75)
     .setSize(50,20)
     .setCaptionLabel("      OK")     
     .moveTo(errorGroup)   
     ;


  //initialize out of bounds error text - show when the ID set by the user is lower than 0 or exceeds 252       
  errorText = cp5.addTextarea("errorText")
                 .setPosition(10,10)
                  .setSize(150,50)
                  .setFont(createFont("arial",12))
                  .setLineHeight(12)
                  .setColor(color(128))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))   
                  .moveTo(errorGroup)    
                  .setVisible(true)
                  .hideScrollbar()
                  ;
  errorText.setText("Error");    


  //initalize text to be shown when servos is sucesfully set  
  successSet = cp5.addTextarea("successSet")
                  .setPosition(10,50)
                  .setSize(180,45)
                  .setFont(createFont("arial",12))
                  .setLineHeight(12)
                  .setColor(color(128))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))   
                  .moveTo(setGroup)    
                  .hideScrollbar()
                  .setVisible(false)
                  ;
    
/*********************SERIAL PORTS/BUTTONS******************/ 

  //initialize button for connecting to selected serial port 
  connectButton = cp5.addButton("connectSerial")
                     .setValue(1)
                     .setPosition(10,25)
                     .setSize(55,15)
                     .setCaptionLabel("Connect")
     ;
     
  //initialize button for disconnecting from current serial port 
  disconnectButton =  cp5.addButton("disconnectSerial")
                         .setValue(1)
                         .setPosition(75,25)
                         .setSize(60,15)
                         .setCaptionLabel("Disconnect")                       
                         .lock()
                         .setColorBackground(color(200))
                         ;
     
  //initialize button to search all available serial ports for an ArbotiX     
  autoSearchButton =  cp5.addButton("autoSearch")
                         .setValue(1)
                         .setPosition(150,25)
                         .setSize(60,15)
                         .setCaptionLabel("Auto Search")
                         ;
     
  //initlaize help button   
  cp5.addButton("helpButton")
     .setValue(1)
     .setPosition(180,5)
     .setSize(30,15)
     .setCaptionLabel("Help")
     ;      
  //initialize serialList dropdown properties
  serialList = cp5.addDropdownList("serialPort")
                  .setPosition(10, 21)
                  .setSize(150,200)
                  .setCaptionLabel("Serial Port")
                  ;
  customize(serialList); // customize the com port list
    
  //iterate through all the items in the serial list (all available serial ports) and add them to the 'serialList' dropdown
  for (int i=0;i<Serial.list().length;i++) 
  {
    //if((Serial.list()[i]).startsWith("/dev/tty.usbserial"))//remove extra UNIX ports to ease confusion
    //{  
    ListBoxItem lbi = serialList.addItem(Serial.list()[i], i);
    lbi.setColorBackground(0xffff0000);
    //}
  }
  
    curIdField.setValue("No Servo Connected");//change current id field to a prompt

}//END SETUP



/*****************************************************END P5 CONTROLLER FUNCTIONS****************************/

/************************************
 * draw
 *
 * the draw()loop runs continously. Most of this program's 
 * functionality is taken care of by P5 control events, but
 * the draw loop will check on the heatbeat signals -
 * persistend connections to the arbotix and connected servos
 *  
 ************************************/  
void draw() 
{
  int curServoId = 0;//id of the current servo
  running = 1;//set to run mode, so that P5 control functions can begein working
  background(128);    //set the background color
  image(img, 0, 400, 220,43);//place the TR logo
   
  //check for hearbeat signal every 100ms. If it's been more than 100ms and the serial port is active, proceed
  if((millis()- time > 100) && sPort != null)
  {
    if(servoScanned != 0)//if a servoscanne is nonzero, then a servo has been connected. 
    {
      try
      {
        curServoId = Integer.parseInt(curIdField.getText());
      }
      catch(Exception e)
      {
        if(debug ==1){println("Error converting string to int");}
      }
     
      //servoHeartbeat(curServoId);   //run a heartbea check to see id the servo is still connected        
    }
   //arbotixHeartbeat();  //run a heartbeat check to see if the arbotix is still connected
   time = millis(); //update the time
  }
  
  //if an outside event or servoHeartbeat has set servoscanned to zero, remove the applicable GUI groups
  if(servoScanned ==0)
  {
     testGroup.setVisible(false);
     setGroup.setVisible(false);
  }
  //otherwise make sure the groups are visible
  else
  {
     testGroup.setVisible(true);
     setGroup.setVisible(true);

  }





  // scroll the scroll List according to the mouseX position
  // when holding down SPACE.
  if (keyPressed && key==' ') {
    //l.scroll(mouseX/((float)width)); // scroll taks values between 0 and 1
  }
  if (keyPressed && key==' ') {
    //l.setWidth(mouseX);
  }
}//end draw()


/*****************************************************START P5 CONTROLLER FUNCTIONS****************************/

/************************************
 * centerDyna
 *
 * centerDyna will receive changes from  controller(button) with name centerDyna
 * centerDyna will send a  positional command to the currently connected servo. 
 ************************************/
 
public void centerDyna(int theValue) 
{
  byte[] centered ={0,0};  //postion, as two bytes, that will be send to the DYNAMIXEL
  int currentServoId =0;   //servo ID to send the positional command to
  
  //get ID from the current ID text field
  try
  {
    currentServoId = Integer.parseInt(curIdField.getText());
  }
  catch(Exception e)
  {
    if(debug ==1){println("Error converting string to int, curIdField to int");}
  }


  if(servoScanned == 300)
  {
    centered = intToBytes(512);//512 = centrered for 12 bit  
    setGoalPositionBytes(currentServoId, centered);  
  }
  else if(servoScanned == 360)
  {
    centered = intToBytes(2048);//2048 = centrered for 12 bit
    setGoalPositionBytes(currentServoId, centered);
  }
  else
  {
    if(debug ==1){println("Servo Not Connected or Unknown Servo");}
    //TODO: Add error text if unknown servo is encountered
  }
}//end centerDyna

/************************************
 * errorButton
 *
 * errorButton will receive changes from  controller(button) with name errorButton
 * errorButton will hide the errortext and group. 
 ************************************/
public void errorButton(int theValue) 
{
  if(running == 1)
  {
    errorGroup.setVisible(false);
  }
}  //end error button

/************************************
 * setDynaButton
 *
 * setDynaButton will receive changes from  controller(button) with name setDynaButton
 * setDynaButton will send two commands to the currently connected DYNAMIXEL
 *  1)set the ID to the ID found in the setid text field
 *  2)set the servo Baud to 1000000
 *
 ************************************/  
 
public void setDynaButton(int theValue) 
{
  if(running ==1 && sPort != null)//run a check to make sure we are in 'running' mode and that we are connected to a serial port
  {
    int newId =256;          //new id that we want to set the servo to
    int currentServoId = 256; //get the current
    String tempId = ""; //string to hold the ID so that we can write it back to the current ID text field
    
    //get ID from the current ID text field
    try
    {
      currentServoId = Integer.parseInt(curIdField.getText());
    }
    catch(Exception e)
    {
      if(debug ==1){println("Error converting string to int, curIdField to int");}
    }
        
    //get ID from the set ID text field        
    try
    {
      newId = Integer.parseInt(newIdField.getText());
    }
    catch(Exception e)
    {
      if(debug ==1){println("Error converting string to int, newIdField to int");}
    }
    
    tempId = ""+newId;  //create an ID string to write back to the currentid field
    
    //check that the ID the user has input is between 0 and 252 (253 is reserved for the arbotix and 254-255 is reserved for DYNAMIXEL operations)
    if(newId <253 && newId >= 0)
    {
      
      //send commmand to the current servo to change its ID. If not sucessful, the function will return a non-1 value
      //in that case, show an error message
      if(setServoId(currentServoId, newId) != 1)
      {
        errorGroup.setVisible(true);
        errorText.setVisible(true);
        errorText.setText("Error setting servo ID. Please re-scan and try again");  
        servoScanned = 0;     
        tempId = "No Servo Connected"; 
      }
      
      //send commmand to the current servo(that has a new ID) to change its baud. If not sucessful, the function will return a non-1 value
      //in that case, show an error message
      if(setServoBaud(newId, 1) != 1)
      {
        errorGroup.setVisible(true);
        errorText.setVisible(true);
        errorText.setText("Error setting servo ID. Please re-scan and try again"); 
        servoScanned = 0;   
        tempId = "No Servo Connected"; 

      }
      
      curIdField.setValue(tempId);   //set currentID to the new id, to reflect the change in ID (or error text)
      curIdField.valueLabel().style().marginLeft = curIdMargin;
      successSet.setVisible(true);   //make visible  success text
      successSet.setText("DYNAMIXEL "+currentServoId+" has been succesfuly been set to ID " + newId + " at 1000000 Baud" );    

    }
    //if ID input by use it out of bounds
    else
    {
      errorGroup.setVisible(true);
      errorText.setVisible(true);
      errorText.setText("Please Enter an ID between 0 and 252");    
    }
    
  }
}//end setDynaButton


/************************************
 * connectSerial
 *
 * connectSerial will receive changes from  controller(button) with name connectSerial
 * connectSerial will take the currently selected serial port and attempt to connect to it
 * connectSerial will also check that serial port to make sure that an arbotix is connected
 *
 ************************************/  
public void connectSerial(int theValue) 
{
  if(running  == 1)//check to make sure we're in run mode
  {
    int serialPortIndex = (int)serialList.value();//get the serial port selected from the serlialList
    
    //try to connect to the port at 115200bps, otherwise show an error message
    try
    {
      sPort = new Serial(this, Serial.list()[serialPortIndex], 115200);
    }
    catch(Exception e)
    {
      if(debug ==1){println("Error Opening Serial Port");}
      errorGroup.setVisible(true);
      errorText.setVisible(true);        
      errorText.setText("Error Connecting to Port - try a different port or try closing other applications using the current port");    
    }
    
    delayMs(100);//add delay for some systems
    
    //send a command to see if the ArbotiX is connected. 
    if(pingArbotix()== 1)
    {
      //show scan and test group
      scanGroup.setVisible(true);
      testGroup.setVisible(true);
      
      //hide error groups
      startupGroup.setVisible(false);
      errorGroup.setVisible(false);
      
      //lock connect button and change apperance, unlock disconnect button and change apperance
      connectButton.lock();
      connectButton.setColorBackground(color(200));
      autoSearchButton.lock();
      autoSearchButton.setColorBackground(color(200));
      disconnectButton.unlock();
      disconnectButton.setColorBackground(color(2,52,77));
    }
    else
    {
      if(debug ==1){println("ArbotiX Not detected");}
      errorGroup.setVisible(true);
      errorText.setVisible(true);
      errorText.setText("No ArbotiX detected on this port. Make sure your ArbotiX is powered on and try again, or try a different port.");    
      sPort.stop();  //disconnect from serial port
      sPort = null;  //set serial port to null, so other functions can easily know we're not connected
    }
  }     
}// end connectSerial


/************************************
 * disconnectSerial
 *
 * disconnectSerial will receive changes from  controller(button) with name disconnectSerial
 * disconnectSerial will disconnect from the current serial port and hide GUI elements that should only
 * be available when connected to an arbotix
 ************************************/  
public void disconnectSerial(int theValue) 
{
  //check to make sure we're in run mode and that the serial port is connected
  if(running ==1 && sPort != null)
  {
    sPort.stop();//stop/disconnect the serial port   
    sPort = null;//set the serial port to null, incase another function checks for connectivity
    curIdField.setValue("No Servo Connected");//change current id field to a prompt
    curIdField.valueLabel().style().marginLeft = 0;
    dynaModelNameField.setValue("");//change model name field to nothing
    servoScanned = 0; //disconnecting the serial port also disconnects any currently connected sercos
    //hide the scan set and test group
    scanGroup.setVisible(false);
    setGroup.setVisible(false);
    testGroup.setVisible(false);
    //make visible the statup prompt
    startupGroup.setVisible(true);

    //unlock connect button and change apperance, lock disconnect button and change apperance
    connectButton.unlock();
    connectButton.setColorBackground(color(2,52,77));
    autoSearchButton.unlock();
    autoSearchButton.setColorBackground(color(2,52,77));
    disconnectButton.lock();
    disconnectButton.setColorBackground(color(200));
  
  }
}//end disconnectSerial



/************************************
 * helpButton
 *
 * helpButton will receive changes from  controller(button) with name helpButton
 * helpButton will link to the servo setter's documentation
 * TODO: make the help a full group panel, to include options
 *  -debug
 *  -full scan
 *  -version info
 ************************************/  
public void helpButton(int theValue) 
{
  if(running ==1)
  {
   link("http://www.trossenrobotics.com/dynamanager");
  }
}//end helpButton



/************************************
 * autoSearch
 *
 * autoSearch will receive changes from  controller(button) with name autoSearch
 * autoSearch will scan/connect to each available serial port and
 *  check if an ArbotiX is connected. If there is not, it will move to the next
 *  port.
 *  
 ************************************/  
public void autoSearch(int theValue) 
{
  //check that we're in run mode  
  if(running ==1)
  {
    //for (int i=0;i<Serial.list().length;i++) //scan from bottom to top
    //scan from the top of the list to the bottom, for most users the ArbotiX will be the most recently added ftdi device
    for (int i=Serial.list().length;i>=0;i--) 
    {
      //try to connect to the current serial port
      try
      {
        sPort = new Serial(this, Serial.list()[i], 115200);
      }
      catch(Exception e)
      {
        if(debug ==1){println("Error Opening Serial Port for Auto Search");}
        //errorGroup.setVisible(true);
        sPort = null;
      }
      delayMs(100);//delay for some systems
        
      if(sPort !=null)
      {
          if(pingArbotix() == 1)
          {
            scanGroup.setVisible(true);
            errorGroup.setVisible(false);
            startupGroup.setVisible(false);
            serialList.setValue(i);
            
            //lock connect button and change apperance, unlock disconnect button and change apperance
            connectButton.lock();
            connectButton.setColorBackground(color(200));
            autoSearchButton.lock();
            autoSearchButton.setColorBackground(color(200));
            disconnectButton.unlock();
            disconnectButton.setColorBackground(color(2,52,77));
      
            break;
          }
          else
          {
            if(debug ==1){println("No Arbotix Found On Port" + Serial.list()[i]);}
            sPort.stop();
            sPort = null;
    
          }
      }
    }
    
    if(sPort == null)
    {
      if(debug ==1){println("AutoSearch Could not detect an ArbotiX ");}
      errorGroup.setVisible(true);
      errorText.setVisible(true);
      errorText.setText("No ArbotiX detected on any port. Check that the ArbotiX is powered on and connected through a serial port.");    
    }
  }
}//end autoSearch



/************************************
 * scanDynaButton
 *
 * scanDynaButton will receive changes from  controller(button) with name scanDynaButton
 * scanDynaButton will scan for a servo by sending out read requests for a servo ID for 
 *   servos 0-252 on bauds 1000000 and 57600. It will stop when it finds the first servo.
 * To save time, this function targets the common cases. It searches the servos in the following sequence
 *     -Search servos 0-29 at 1000000 baud
 *     -Search servos 0-29 at 57600 baud
 *     -Search servos 30-252 at 1000000 baud
 *     -Search servos 30-252 at 57600 baud
 *
 * This function does not scan any other bauds. 
 *  
 ************************************/  
 
public void scanDynaButton(int theValue) 
{
  
  int servoId =0;    //id of the first servo found
  String tempId;     //string ID value, to write back to the current id text field
  int i =0;          //counter
  servoScanned = 0;
  //check that we're connected and in run mode
  if(sPort != null && running == 1)
  {          
    
    
    setArbotixBaud(1000000);  //set the Arbotix's DYNAMIXEL bus to 1000000
    curIdField.setColorBackground(color(2,52,77));
            
            
            if(debug1 ==1){println("Scan ID 1 @ 1MBPS" );}
                  
    while (servoScanned == 0 && i <6)
  {
      if(checkId(1) == 1)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        dynaModelNameField.valueLabel().style().marginLeft = modelMargin;
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 1000000" );}
      }
    
    
    
      i++;
  }  
  
    i = 0;//reset counter
    
            if(debug1 ==1){println("Scan ID 1 @ 57600" );}
    if( servoScanned ==0)//if we still haven't found a servo, change the baud
    {
    setArbotixBaud(57600);  //set the Arbotix's DYNAMIXEL bus to 1000000
    
                    //curIdField.setColor(color(10));
                  curIdField.setColorBackground(color(255,204,0));
    }
    
    while (servoScanned == 0 && i <6)
  {
      if(checkId(1) == 1)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        dynaModelNameField.valueLabel().style().marginLeft = modelMargin;
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 1000000" );}
      }
    
    
    
      i++;
  }  
    
    
    i = 0;//reset counter
    
            if(debug1 ==1){println("Scan ID  @ 1MBPS" );}
    if( servoScanned ==0)//if we still haven't found a servo, change the baud
    {
    setArbotixBaud(1000000);  //set the Arbotix's DYNAMIXEL bus to 1000000
    curIdField.setColorBackground(color(2,52,77));
    }
    
    while(servoScanned == 0 && i<30)  //scan servos 0-29
    {
      if(checkId(i) == i)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        dynaModelNameField.valueLabel().style().marginLeft = modelMargin;
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 1000000" );}
        
       // curIdField.setValue("Scanning..." + i);
    
      }
      i++;
    }

            if(debug1 ==1){println("Scan ID  @ 57600" );}
    i = 0;//reset counter
    
    if( servoScanned ==0)//if we still haven't found a servo, change the baud
    {
      setArbotixBaud(57600);
    }
    
    
    while(servoScanned == 0 && i<30)  //scan servos 0-29
    {
      if(checkId(i) == i)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 57600" );}
      }
      i++;
    }
    
    i = 30;
    if( servoScanned ==0)//if we still haven't found a servo, change baud back to 1000000
    {
    setArbotixBaud(1000000);  //set the Arbotix's DYNAMIXEL bus to 1000000
    curIdField.setColorBackground(color(2,52,77));
    }
    
            if(debug1 ==1){println("Scan ID up @ 1MBPS" );}
    while(servoScanned == 0 && i<253)
    {
      if(checkId(i) == i)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 1000000" );}
      }
      i++;
    }
    
      i = 30;
      
            if(debug1 ==1){println("Scan ID up @ 57600" );}
            
    if( servoScanned ==0)//if we still haven't found a servo, change baud back to 57600
    {
    
      setArbotixBaud(57600);
    }
    while(servoScanned == 0 && i<253)
    {
      if(checkId(i) == i)//send read command for id, if the same id is returned, then we have found a servo
      {        
        dynaModelNameField.setValue(getDynaModelName(i));  //get the connected DYNAMIXEL's model name and set it to the name text field
        tempId = ""+i; //prepare the id as a string
        curIdField.setValue(tempId);//write the id as a string back to the text field
        curIdField.valueLabel().style().marginLeft = curIdMargin;
        setScannedServo(i);         //set global varaibles for the servo being scanned
        if(debug ==1){println("Servo Scan complete, servo "+ i + " found at 57600" );}
      }
      i++;
    }  
  
    //if we've exhausted the scan, set the proper text fields 
    if(servoScanned == 0)
    {
      if(debug ==1){println("Servo Scan complete, no servo found");}
      curIdField.setValue("No Servo Found");
      curIdField.valueLabel().style().marginLeft = 0;
      dynaModelNameField.setValue("");
      dynaModelNameField.valueLabel().style().marginLeft = modelMargin;
      servoScanned = 0; 
    }
  }
} //end scanDynaButton 

void customize(DropdownList ddl) {
  // a convenience function to customize a DropdownList
  ddl.setBackgroundColor(color(190));
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 2;
  ddl.valueLabel().style().marginTop = 3;


  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}

/************************************
 * stop
 * called when ending the program,
 * make sure to close the serial port
 ***********************************/
void stop()
{
 sPort.stop(); 
}




void keyPressed() 
{
}



/************************************
 * mouseClicked
 *
 * open the Trossen Robotics website if the logo is pressed
 ************************************/  
void mouseClicked()
{
  if((mouseX >= 0) && (mouseX <= 220) && (mouseY >= 400) && (mouseY <= 443) == true)
  {
    link("http://www.trossenrobotics.com/");
  }
  
}

/*
  cp5.addButton("scanDynaButton")
   .setValue(1)
   .setPosition(10,10)
   .setSize(70,70)
   .setCaptionLabel("  Scan")  
   .moveTo(scanGroup)   
   ;*/

/************************************
 * mousePressed
 *
 * A bit of a hack here - if you call an update to a controllerp5 object,
 * like a Textfield from a function called by another control object,
 * like a button, the Textfield will not update until the end of the 
 * button's function. 
 * By calling this on a mouse press, we can change the message to 
 * 'Scanning' 
 *
 *We will also use this function to see if the knob are has been clicked
 *
 **************************************/  
void mousePressed()
{
  //if((mouseX >= scanDynaButton.getAbsolutePosition().x) && (mouseX <= scanDynaButton.getWidth()+scanDynaButton.getAbsolutePosition().x) && (mouseY >= scanDynaButton.getAbsolutePosition().x) && (mouseY <= scanDynaButton.getHeight()+scanDynaButton.getAbsolutePosition().x) == true)//position of scan button
  if((mouseX >= 20) && (mouseX <= 91) && (mouseY >= 61) && (mouseY <= 131) &&  scanGroup.isVisible() == true)//position of scan button
  {
    dynaModelNameField.setValue("");
    curIdField.setValue("Scanning...");
    curIdField.valueLabel().style().marginLeft = 0;
    successSet.setVisible(false);
  }
  
  if(mouseClickKob()==true)
  {
    knobClickState=1;
  }
  else
  {
    knobClickState=0;
  }
    
}

boolean mouseClickKob()
{
   return ((mouseX >= 11) && (mouseX <= 85) && (mouseY >= 296) && (mouseY <= 374));
}


void mouseReleased()
{
  knobClickState = 0; //if the mouse is released, we've stopped clicking the kbnob
}

/************************************
 * knobCanvas
 *
 * A canvas item that holds the knob element
 *
 **************************************/  
class knobCanvas extends Canvas 
{
  int curServoId;    //current servo id
  int servoPosition;  //current servo position as an integer
  float servoDegree;  //current servo degree, where full clockwise is 0 degreees
  float servoRadOffset;//servo posoion, offset for home position of rotation of knob vs home position for rotation of the horn
  float goalRad;      //the place we want the  servo to go to in radians
  float goalDegree;    //the place we want the servo to go to in degrees
  int goalPositionInt;  //the goal position as an integer
  byte[] goalPositionBytes = {0,0};  //goal position as two bytes, ready to send to DYNAMIXEL position registers
  int maxPosition = 0;               //maximum integet position, 1023 for 10 bit servos, 4096 got 12 bit
  float maxDegrees =0;               //maximum degress, 300 for 10 bit servos, 360 for 12 bit
  float lowerLimit = 0;              //lower limits in degrees of servo
  float upperLimit = 0;              //upper limits in degrees of servo
  int limitedKnob = 0;               //flago to figure out if the knob neess to be drawn as limited or full
  float posOffset = 0;               //radian offset for positive values
  float negOffset = 0;                //radian offser foir negative values
  public void setup(PApplet p) 
  {

  }
  
  public void draw(PApplet p) 
  {
    //set variables for 10-bit, 300 degree servos
    if(servoScanned == 300)
    {
      maxPosition = 1023;
      lowerLimit = radians(240);  //lower limits for 300 degrees
      upperLimit = radians(300);  //upper limits for 300 degrees
      limitedKnob = 1;
      maxDegrees =300;
      posOffset = PI/3;
      negOffset = 5*PI/3;
    }
    
  
    //set variables for 12-bit, 360 degree servos
    else if(servoScanned == 360)
    {
      maxPosition = 4095;
      lowerLimit = radians(360);
      upperLimit = radians(360); 
      limitedKnob = 0;
      maxDegrees = 360;
      posOffset = PI/2;
      negOffset = 3*PI/2;
      
    }
    
    //if a servo has been connected sucesffuly, proceed to  the knob
    if(servoScanned == 300 || servoScanned == 360)
    {    
      //mouse positions relative to the center of the knob      
      xRel = mouseX-60;
      yRel = (mouseY-340);
      
      //angle of the mouse from the center of the knob
      mouseRad = atan2(yRel, -1*xRel) + (PI);
      
      //get the current servo id
      try
      {
        curServoId = Integer.parseInt(curIdField.getText());
      }
      catch(Exception e)
      {
        if(debug ==1){println("Error converting string to int");}
      }

      if(knobClickState==1)//if the knob is currenly been clicked on, update the goal position indicator
      {         
        //todo: update behavior when indicator jumps from one side of the deadband to the other 
        if(mouseRad >lowerLimit && mouseRad < upperLimit )//handle cases where mouse is in the servo deadzone
        {   
          if(mouseRad < (3*PI/2))//if its in the lower half of the deadzone, keep the indicator on the left half
          {
           mouseRad = lowerLimit; 
          }
          else if(mouseRad > (3*PI/2))//if its in the upper half of the deadzone, keep the indicator on the right  half
          {
           mouseRad = upperLimit;     
          }
          if(debug ==1){println("Mouse in Serial deadband");}
          drawKnob(mouseRad,limitedKnob, color(255,0,0));//draw the knob, with a red positon indicator
        } 
        
        else//oherwise the mouse is in a good area, and we will move the servo to that position.
        {
          drawKnob(mouseRad,limitedKnob, color(0,255,255));//draw the knob with the indicator based on the angle of the mouse vs the knob              
          if(mouseRad > (negOffset))//if the mouse position is greater than the neagtive offset, than we need to move it back
          {
            goalRad = mouseRad - (negOffset);
          }
          else//otherwise add the positive offset
          {
            goalRad = mouseRad + (posOffset);
          }
          
          goalDegree = degrees(goalRad);//convert radians to degrees
          goalPositionInt = floor(maxPosition*goalDegree/maxDegrees);//convert degrees to integer
          goalPositionBytes = intToBytes(goalPositionInt);//  convert position in to two bytes
          
          presentPositionField.setValue(goalPositionInt + ""); //update the position value

          //send  postional data
          try
          {
            setGoalPositionBytes(curServoId, goalPositionBytes); 
          }
          catch(Exception e)
          {
            if(debug ==1){println("SERVO NOT CONNECTED");}
          }
        }
      }  

      //if the mouse hasn't been clicked, update the indicator based on the actual servo position    
      else
      {
        //get postional data
        try
        {
          if(millis() - lastPositionTime > 100)
          {
            lastPositionTime = millis();
            int tempPos = getPosition(curServoId);
            //println(tempPos);
            if(tempPos != -1)
            {
               servoPosition = tempPos;//getPosition(curServoId);
            }
            
          }
        }
        catch(Exception e)
        {
          if(debug ==1){println("getting servo data");}
        }
        servoDegree = servoPosition*(maxDegrees/maxPosition);//convert position int to degrees based on current servo type
        servoRadOffset = radians(servoDegree) - (posOffset) ;//convert degrees to radians, and account for offet to tranlsate into knob rotation
        drawKnob(servoRadOffset,limitedKnob);//draw knob with indicator at correct position
        presentPositionField.setValue(servoPosition + ""); //update the position value

         
         
      }
    }
  }   //end applet draw
  
  


/************************************
 * drawKnob(float, int, color)
 *
 * Parameters: 
 *           float rotation : angle from center to rotate indicator circle
 *           int limits     : 1 = limited(300 degree servo) so display a large arc to represent that
 *           color indicator: used to change the color of the positional indicator
 *
 * Returns: none
 *
 * Description: This function will draw the needed ellipses to represent the DYNAMIXEL
 *               positional knob.
 *
 ************************************/
  public void drawKnob(float rotation, int limits, color indicator)
  {
    pushMatrix();
    ellipseMode(CENTER);  //center based ellipse drawing
    fill(2,52,77);       //dark blue 
    ellipse(45,45,75,75);//draw background circle  
    
    strokeWeight(2);//thicker line
    stroke(255);  //white stroke
    line(45,15,50,25);//draw center line at top of servo
    noStroke();//disable stroke
    
    fill(1,108,158);//ligheter blue
    ellipse(45,45,65,65);//draw middle circle
    fill(2,52,77);//darker blue
    ellipse(45,45,55,55);//draw inner circle
    
    //limited servo, draw large arc
    if(limits ==1)
    {
      arc(45, 45, 75, 75, 1.047, 2.094);//draw arc to hide section of middle circle
    }
    //not limited, draw small arc
    else
    {
      arc(45, 45, 75, 75, HALF_PI-.05, HALF_PI+.05);//draw arc to hide section of middle circle

    }
    //translate reference point 
    translate(45, 45);
    //rotate reference 
    rotate(-rotation);
    fill(indicator);
    //fill(153, 204, 0);
    ellipse(28, 0, 10, 10);//draw indicator color
    popMatrix();
    
  }
  
   /************************************
   * drawKnob(float, int)
   *
   * Parameters: 
   *           float rotation : angle from center to rotate indicator circle
   *           int limits     : 1 = limited(300 degree servo) so display a large arc to represent that
   *
   * Returns: none
   *
   * Description: Conveience wrapper, defualts the indcaitor to a green color
   *
   ************************************/ 
  public void drawKnob(float rotation, int limits)
  {
    drawKnob( rotation, limits, color(153, 204, 0));
  
  }
  
}//end applet canvas












/************************************
 * setScannedServo(int)
 *
 * Parameters
 * int id : id # of the DYNAMIXEL servo being queried 
 *
 *
 * Returns: none
 *
 * Description: sets the global 'servoScanned' based on what
 * resolution the currently connected sero is
 *
 ************************************/
public void setScannedServo(int servoId)
{
  int servoResolution;
  servoResolution =getDynaResolution(servoId);
  if(servoResolution == 12)
  {        
    servoScanned = 360;
  }
  else if(servoResolution == 10)
  {
    servoScanned = 300;
  }
  else
  {
    if(debug==1){println("Unknown Servo"); }
    servoScanned = 0; 
  } 
}

/************************************
 * getDynaPacket(int)
 *
 * Parameters
 * int id : id # of the DYNAMIXEL servo being queried 
 *
 *
 * Returns: none
 *
 * Description: set servo to full clockwise, full counter, then center
 *
 ************************************/
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
  return((int(convertBytes[1]<<8))+int(convertBytes[0]));//cast to int to ensureprper signed/unsigned behavior
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
  sPort.clear();
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


  return(0);
  
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
    return(int(returnedId[0]));
  }
  else
  {
     return(256);//reutrn 256 to represent no ID found, as a servo can never have id=256 
  }
}
int getPosition(int servoId)
{
  byte[] parameters = {36, 2};
  byte[] positionByte = {0,0};
  int positionInt=0;
  positionByte=readDynaReg(servoId, parameters);


  if(positionByte != null)
  {
    positionInt = bytesToInt(positionByte);
    if(debug==1){ println("Current Position:" + positionInt);} //debug 

  }
  else
  {
     //return(256);//reutrn 256 to represent no ID found, as a servo can never have id=256 
     return(-1);
  }
  return(positionInt);
}

void servoHeartbeat(int servoId)
{
  if(checkId(servoId) == servoId)
  {
    //println("HEARTBEAT GOOD");
  } 
  else
  {
    servoScanned =0;  
    dynaModelNameField.setValue("");
    successSet.setVisible(false);

    curIdField.setValue("No Servo Connected");
    curIdField.valueLabel().style().marginLeft = 0;
    
   // println("HEARTBEAT BAD");
  }




  
}

void arbotixHeartbeat()
{
  
  if(pingArbotix() == 1)
  {
   scanGroup.setVisible(true);
    setGroup.setVisible(true);
    testGroup.setVisible(true);
    startupGroup.setVisible(false);  
    //connected = 1;
  }
  else
  {
    scanGroup.setVisible(false);
    setGroup.setVisible(false);
    testGroup.setVisible(false);
    startupGroup.setVisible(true);
    //connected = 0;
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
      return(10);
    }
  }
  
  for(int i = 0; i< twelveBitDyna.length;i++)
  {
    if(modelInt == twelveBitDyna[i])
    {
      return(12);
    }
  }
  
  return(0); //no model info found, return 0
}

String getDynaModelName(int servoId)
{
  int[] modelNumbers = {12, 18, 300, 24, 28, 64, 29, 310, 320, 107};
  String[] modelNames = {"AX-12","AX-18","AX-12W","RX-24","RX-28","RX-64","MX-28","MX-64","MX-106","EX-106",};
  byte[] modelByte = {0,0};                     //bytes that represent the DYNAMIXEL model
  int modelInt = 0;                             //model number as a single int
  byte[] parameters = {0, 2};                   //model number starts at register 0 and 2 registers long
    
  modelByte=readDynaReg(servoId, parameters);   //get the model number

  if(modelByte != null)
  {
    modelInt = bytesToInt(modelByte);
  } 
  
  for(int i = 0; i< modelNumbers.length;i++)
  {
    if(modelInt == modelNumbers[i])
    {
      return(modelNames[i]); 
    }
  }

  return("Unknown Model");
  
  
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

 
  byte[] parameters = {byte(3), 1};//id is at register 4, length 1 
  byte id = byte(0xfd);//253 = id to set Arbotix parameters
  byte[] response = new byte[6];

  long startReadingTime = millis();//time that the program started looking for data

 // long startReadingTime = millis();//time that the program started looking for data

  while(sPort.available() < 5  & millis()-startReadingTime < packetRepsonseTimeout)
  {
      sendDynaPacket(id, byte(2), parameters); //send a dynamixel packet to the id# servoid with parameters. '2' is the 'READ' instruction
      response = readDynaPacket(0);//get response to see if arbotix is present
        //check headers
      if(response[0] == byte(255) && response[1] == byte(255) && response[2] == byte(253) && response[3] == byte(3) && response[4] == byte(0) && response[5] == byte(253))
      {
        return(1);
      }
       
       else 
       {
        
       }
  
  }
return(0);

  
 
  /*
  if(checkId(253) == 253)
  {
    println("heartbeat good");
   return(1);
  }
  else
  {
    println("heartbeat bad");
 
   return(0); 
  }*/
  /*
  
  */
  
  
}

