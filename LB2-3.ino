#include <ros.h>
#include <std_msgs/Int16.h>
#include <SoftwareSerial.h>

class BlueROS : public ArduinoHardware
{
  protected:

  private:
    SoftwareSerial *mySerial;

  public:
  BlueROS(){}

  void init() {
    mySerial = new SoftwareSerial(A1, 11);
    mySerial->begin(9600);
  }

  int read()  {
    return mySerial->read();
  };

  void write(uint8_t* data, int length) {
    for (int i = 0; i<length; i++)  {
      mySerial->write(data[i]);
    }
  }
};

ros::NodeHandle_<BlueROS, 3, 3, 100, 100> nh;
std_msgs::Int16 msg;
ros::Publisher pubFront("sensorFront", &msg);
const int echoPinFront = 5;
const int trigPinFront = A4;

long duration;
int distance;

void setup() {
  Serial.begin(9600);
  
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);

  nh.initNode();
  nh.advertise(pubFront);
}

void loop() {

  msg.data = ultrasonic(echoPinFront, trigPinFront);
  pubFront.publish(&msg);
  
  
  nh.spinOnce();

  delay(100);

}

int ultrasonic(int echoPin, int trigPin)  {
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin,HIGH);
  distance = duration * 0.034/2;

  return distance;
}
