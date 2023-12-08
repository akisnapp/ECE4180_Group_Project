#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include "mbed.h"
#include "ultrasonic.h"

//Set the delay constant for waiting for cycles to complete
#define BNO055_SAMPLERATE_DELAY_MS (100)

Serial pc(USBTX, USBRX);
I2C i2c(p9, p10);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &i2c);

PwmOut speaker(p21);

DigitalIn button(p22);
DigitalIn reset(p23);

PwmOut re(p26);
PwmOut gr(p25);
PwmOut bl(p24);

AnalogIn red(p20);
AnalogIn green(p19);
AnalogIn blue(p18);

float location[3] = {0.0, 0.0, 0.0};
int prev_height = 0;
int prev = 1;
float init_height;


void dist(int distance)
{
    //update y coordinate based on height readings and initialization constant calculated below
    if (reset == 0) {
        location[1] = 0;
        prev_height = 0;
    } else {
        if (distance > prev_height + 5) {
            location[1] = init_height - 5;
        } else if (distance < prev_height - 5) {
            location[1] = init_height + 5;
        } else {
            location[1] = init_height - (distance / 10.0);
        }
    }

    if (!init_height) {
        init_height = distance / 10.0;
    }

}


ultrasonic mu(p6, p7, .1, 1, &dist);    //Set the trigger pin to p6 and the echo pin to p7
                                        //have updates every .1 seconds and a timeout after 1
                                        //second, and call dist when the distance changes


void loop();

/*************************************************************************

    setup function for initializing location statistics and starting data sampling

*************************************************************************/
int main()
{
  pc.baud(115200);
  pc.printf("Orientation Sensor Raw Data Test\r\n");

  mu.startUpdates();
  button.mode(PullUp);
  reset.mode(PullUp);
  //mu.startUpdates();//start measuring the distance

  //Initialise sensor
  if(!bno.begin())
  {
    //bno-055 not detected
    pc.printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\r\n");
    while(1);
  }
  else
    pc.printf("BNO055 was detected!\r\n");

  wait(1);
  
  mu.checkDistance();

  //int[] location = {0, 0, 0};

  // print the temperature during setup (if 0 C this is indicator the bno-055 clock needs to be reset)
  int8_t temp = bno.getTemp();
  pc.printf("Current Temperature: %d C\r\n", temp);
  bno.setExtCrystalUse(true);

  pc.printf("Calibration status values: 0=uncalibrated, 3=fully calibrated\r\n");

  while(true)
      loop();
}

/*************************************************************************

    Loop what happens continuously, used to calculate color, update location and send data over serial port
    each time that it is called
    
**************************************************************************/
void loop(void)
{
    //color calculations from potentiometers
    int b = (int) (blue * 255); //blue
    int g = (int) (green * 255); //green
    int r = (int) (red * 255); //red

    //update rgb led with color the user is currently using
    re = red;
    gr = green;
    bl = blue;

    mu.checkDistance();  //the class checks if dist needs to be called.

    //get vector data in euler angles, function uses quanternion data to get euler data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    //location calculations
    if (reset == 0) { //reset to initial starting point
        location[0] = 0;
        location[2] = 0;
    } else {
        if (euler.y() / 3 > 5) { //catch for if movement is too much
            location[2] += 5;
        } else {
            location[2] += euler.y() / 3;
        }
        if (euler.z() / 3 > 5) { //catch for if movement is too much
            location[0] += 5;
        } else {
            location[0] += euler.z() / 3;
        }
    }

    if (button == 0) { //print data to serial so that it is accessible by the python script
        
        //pc.printf("EULER: X: %f Y: %f Z: %f\r\n", euler.x(), euler.y(), euler.z());
        pc.printf("X: %f Y: %f Z: %f\r\n", location[0], location[1], location[2]);
        pc.printf(" @%d,%d,%d\r\n",r, g, b);
        speaker = 0.3; //speaker on to let user know they are drawing

    } else {
        speaker = 0; //speaker off to let user know that they are not drawing
    }

    //check if there is a break statement between drawings so that python code knows to leave a gap for 'pen up'
    if (prev == 0 && button == 1) { 
        pc.printf("b\r\n");
    }
    prev = button;



  //calibration status update for each sensor.
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  //pc.printf("CALIBRATION: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\r\n", (int)(system), (int)(gyro), (int)(accel), (int)(mag));
  wait_ms(BNO055_SAMPLERATE_DELAY_MS * 2);
}
