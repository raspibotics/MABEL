///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>

byte error, MPU_6050_found, nunchuck_found, lowByte, highByte;
int address;
int nDevices;

void setup()
{
  Wire.begin();
  TWBR = 12;
  Serial.begin(9600);
}

void loop()
{
  Serial.println("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
      if(address == 0x68 || address == 0x69){
        Serial.println("This could be a MPU-6050");
        Wire.beginTransmission(address);
        Wire.write(0x75);
        Wire.endTransmission();
        Serial.println("Send Who am I request...");
        Wire.requestFrom(address, 1);
        while(Wire.available() < 1);
        lowByte = Wire.read();
        if(lowByte == 0x68){
          Serial.print("Who Am I responce is ok: 0x");
          Serial.println(lowByte, HEX);
        }
        else{
          Serial.print("Wrong Who Am I responce: 0x");
          if (lowByte<16)Serial.print("0");
          Serial.println(lowByte, HEX);
        }
        if(lowByte == 0x68 && address == 0x68){
          MPU_6050_found = 1;
          Serial.println("Starting Gyro....");
          set_gyro_registers();
        }
      }
      if(address == 0x52){
        Serial.println("This could be a Nunchuck");
        Serial.println("Trying to initialise the device...");
        Wire.beginTransmission(0x52);
        Wire.write(0xF0);
        Wire.write(0x55);
        Wire.endTransmission();
        delay(20);
        Wire.beginTransmission(0x52);
        Wire.write(0xFB);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(20);
        Serial.println("Sending joystick data request...");
        Wire.beginTransmission(0x52);
        Wire.write(0x00);
        Wire.endTransmission();
        Wire.requestFrom(0x52,1);
        while(Wire.available() < 1);
        lowByte = Wire.read();
        if(lowByte > 100 && lowByte < 160){
          Serial.print("Data response normal: ");
          Serial.println(lowByte);
          nunchuck_found = 1;
        }
        else{
          Serial.print("Data response is not normal: ");
          Serial.println(lowByte);
        }
      }
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  if(MPU_6050_found){
    Serial.print("Balance value: ");
    Wire.beginTransmission(0x68);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(0x68,2);
    Serial.println((Wire.read()<<8|Wire.read())*-1);
    delay(20);
    Serial.println("Printing raw gyro values");
    for(address = 0; address < 20; address++ ){
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission();
      Wire.requestFrom(0x68,6);
      while(Wire.available() < 6);
      Serial.print("Gyro X = "); 
      Serial.print(Wire.read()<<8|Wire.read());
      Serial.print(" Gyro Y = "); 
      Serial.print(Wire.read()<<8|Wire.read());
      Serial.print(" Gyro Z = "); 
      Serial.println(Wire.read()<<8|Wire.read());
    }
    Serial.println("");
  }
  else Serial.println("No MPU-6050 device found at address 0x68");

  if(nunchuck_found){
    Serial.println("Printing raw Nunchuck values");
    for(address = 0; address < 20; address++ ){ 
      Wire.beginTransmission(0x52);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(0x52,2);
      while(Wire.available() < 2);
      Serial.print("Joystick X = "); 
      Serial.print(Wire.read());
      Serial.print(" Joystick y = ");
      Serial.println(Wire.read());
      delay(100);
    }
  }
  else Serial.println("No Nunchuck device found at address 0x52");
  while(1);
}

void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}









