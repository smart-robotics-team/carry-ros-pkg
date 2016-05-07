/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIXELS_PIN 6
#define RELAY_PIN 5
#define VOLTAGE_PIN A0

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIXELS_PIN, NEO_GRB + NEO_KHZ800);

ros::NodeHandle  nh;

volatile int recharge_state;
volatile unsigned long last_time_output = 0;

volatile unsigned long last_time_sensing = 0;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  recharge_state = 1;
  last_time_output = millis();
}
ros::Subscriber<std_msgs::Empty> sub("recharge_battery", messageCb );

void messageTwistCb( const geometry_msgs::Twist& msg){
  double nb_pixels = 0.0;
  if(msg.linear.x > 0.01)
  {
    uint16_t i=0;
    nb_pixels = fmin(msg.linear.x / 0.2 * 9.0, 9.0);
    //strip.setPixelColor(7, strip.Color(0, 255, 0)); 
    for(i=0; i<nb_pixels; i++) {
      strip.setPixelColor(0+i, strip.Color(0, 255, 0)); 
      strip.setPixelColor(59-i, strip.Color(0, 255, 0)); 
    }
    for(; i<9; i++) {
      strip.setPixelColor(0+i, 0);
      strip.setPixelColor(59-i, 0);
    }
    
    //strip.setPixelColor(37, 0);
    for(i=0; i<9; i++) {
      strip.setPixelColor(30+i, 0); // blue
      strip.setPixelColor(29-i, 0); // blue
    }
    
  }
  else if(msg.linear.x < -0.01)
  {
    uint16_t i=0;
    nb_pixels = fmin(-msg.linear.x / 0.2 * 9.0, 9);
    //strip.setPixelColor(37, strip.Color(0, 255, 0)); 
    for(i=0; i<nb_pixels; i++) {
      strip.setPixelColor(30+i, strip.Color(0, 255, 0)); 
      strip.setPixelColor(29-i, strip.Color(0, 255, 0)); 
    }
    for(; i<9; i++) {
      strip.setPixelColor(30+i, 0);
      strip.setPixelColor(29-i, 0);
    }
    
    //strip.setPixelColor(7, 0);
    for(i=0; i<9; i++) {
      strip.setPixelColor(0+i, 0); // blue
      strip.setPixelColor(59-i, 0); // blue
    }
    
  }
  else
  {
    for(uint16_t i=0; i<9; i++) {
      strip.setPixelColor(0+i, 0);
      strip.setPixelColor(59-i, 0);
      strip.setPixelColor(30+i, 0);
      strip.setPixelColor(29-i, 0);
    }
  }
  
  
  
  if(msg.linear.y > 0.01)
  {
    uint16_t i=0;
    nb_pixels = fmin(msg.linear.y / 0.2 * 8.0, 8);
    //strip.setPixelColor(22, strip.Color(0, 255, 0)); 
    for(i=0; i<nb_pixels; i++) {
      strip.setPixelColor(15+i, strip.Color(0, 255, 0)); 
      strip.setPixelColor(14-i, strip.Color(0, 255, 0)); 
    }
    for(; i<8; i++) {
      strip.setPixelColor(15+i, 0);
      strip.setPixelColor(14-i, 0);
    }
    
    //strip.setPixelColor(52, 0);
    for(i=0; i<8; i++) {
      strip.setPixelColor(45+i, 0); // blue
      strip.setPixelColor(44-i, 0); // blue
    }
    
  }
  else if(msg.linear.y < -0.01)
  {
    uint16_t i=0;
    nb_pixels = fmin(-msg.linear.y / 0.2 * 8.0, 8);
    //strip.setPixelColor(52, strip.Color(0, 255, 0)); 
    for(i=0; i<nb_pixels; i++) {
      strip.setPixelColor(45+i, strip.Color(0, 255, 0)); 
      strip.setPixelColor(44-i, strip.Color(0, 255, 0)); 
    }
    for(; i<8; i++) {
      strip.setPixelColor(45+i, 0);
      strip.setPixelColor(44-i, 0);
    }
    
    //strip.setPixelColor(22, 0);
    for(i=0; i<8; i++) {
      strip.setPixelColor(15+i, 0); // blue
      strip.setPixelColor(14-i, 0); // blue
    }
    
  }
  else
  {
    for(uint16_t i=0; i<8; i++) {
      strip.setPixelColor(15+i, 0);
      strip.setPixelColor(14-i, 0);
      strip.setPixelColor(45+i, 0);
      strip.setPixelColor(44-i, 0);
    }
  }
  
  
  
  strip.show();
}
ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", messageTwistCb );

std_msgs::Float32 float_msg;
ros::Publisher battery_pub("battery_level", &float_msg);


void setup()
{
  pinMode(13, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  analogReference(DEFAULT);
  recharge_state = 0;
  
  last_time_sensing= millis();
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  nh.initNode();
  nh.advertise(battery_pub);
  nh.subscribe(sub);
  nh.subscribe(subTwist);

  //colorWipe(strip.Color(255, 0, 0), 50); // Red

}

void loop()
{
  
  nh.spinOnce();
  delay(10);
  
  if( abs(last_time_sensing*1.0 - millis()*1.0) > (30.0*1000.0) )
  {
    last_time_sensing = millis();
    float_msg.data = analogRead(VOLTAGE_PIN) * 5.0 / 1023.0;
    battery_pub.publish( &float_msg );
  }
  
  switch(recharge_state)
  {
    case 0 :
      break;
    case 1 :
      digitalWrite(RELAY_PIN, HIGH);
      if( abs(last_time_output*1.0 - millis()*1.0) > (3.0*1000.0) )
      {
        digitalWrite(RELAY_PIN, LOW);
        last_time_output = millis();
        recharge_state = 2;
      }
      break;
    case 2 :
      if( abs(last_time_output*1.0 - millis()*1.0) > (2.0*1000.0) )
      {
        digitalWrite(RELAY_PIN, HIGH);
        last_time_output = millis();
        recharge_state = 3;
      }
      break;
    case 3 :
      if( abs(last_time_output*1.0 - millis()*1.0) > (0.5*1000.0) )
      {
        digitalWrite(RELAY_PIN, LOW);
        last_time_output = millis();
        recharge_state = 0;
      }
      break;
    default :
      recharge_state = 0;
      break;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
