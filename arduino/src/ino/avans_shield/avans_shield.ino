/* 
 * Avans Arduin shield
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>


ros::NodeHandle nh;

const int number_of_buttons = 4;
const int number_of_leds = 8;
const int number_of_adc = 2;

std_msgs::Bool pushed_msg[number_of_buttons];
std_msgs::Bool state_msg[number_of_buttons];
ros::Publisher *pub_button_pushed[number_of_buttons];
ros::Publisher *pub_button_state[number_of_buttons];

const int button_pin[] = {8, 9, 10, 11};


bool last_reading[number_of_buttons];
long last_debounce_time[number_of_buttons] = {0};
long debounce_delay=50;
bool published[number_of_buttons] = {true};


const int led_pin[] = {12, 13, 2, 3, 4, 5, 6, 7};



std_msgs::Int16 adc_value_msg[number_of_adc];
ros::Publisher *pub_adc_value[number_of_adc];


void ledToggle(int led_pin){
  digitalWrite(led_pin, HIGH-digitalRead(led_pin));   // blink the led
}


#define GENERATE_LED_TOGGLE_FUNC(__index) \
    void ledToggleCb##__index(const std_msgs::Empty& toggle_msg){\
    ledToggle(led_pin[__index]);}  \
    
GENERATE_LED_TOGGLE_FUNC(0)
GENERATE_LED_TOGGLE_FUNC(1)
GENERATE_LED_TOGGLE_FUNC(2)
GENERATE_LED_TOGGLE_FUNC(3)
GENERATE_LED_TOGGLE_FUNC(4)
GENERATE_LED_TOGGLE_FUNC(5)
GENERATE_LED_TOGGLE_FUNC(6)
GENERATE_LED_TOGGLE_FUNC(7)


typedef void (*led_toggle_func_cb)(const std_msgs::Empty&);  

led_toggle_func_cb ledToggleCb[]={ledToggleCb0,
                                  ledToggleCb1,
                                  ledToggleCb2,
                                  ledToggleCb3,
                                  ledToggleCb4,
                                  ledToggleCb5,
                                  ledToggleCb6,
                                  ledToggleCb7};
                                  
                                  
                                  
void ledState(int led_pin, const std_msgs::Bool& state_msg){
  digitalWrite(led_pin, state_msg.data);
}


#define GENERATE_LED_STATE_FUNC(__index) \
    void ledStateCb##__index(const std_msgs::Bool& state_msg){\
    ledState(led_pin[__index], state_msg);}  \
    
GENERATE_LED_STATE_FUNC(0)
GENERATE_LED_STATE_FUNC(1)
GENERATE_LED_STATE_FUNC(2)
GENERATE_LED_STATE_FUNC(3)
GENERATE_LED_STATE_FUNC(4)
GENERATE_LED_STATE_FUNC(5)
GENERATE_LED_STATE_FUNC(6)
GENERATE_LED_STATE_FUNC(7)


typedef void (*led_state_func_cb)(const std_msgs::Bool&); 

led_state_func_cb ledStateCb[]={ledStateCb0,
                                  ledStateCb1,
                                  ledStateCb2,
                                  ledStateCb3,
                                  ledStateCb4,
                                  ledStateCb5,
                                  ledStateCb6,
                                  ledStateCb7};


ros::Subscriber<std_msgs::Empty> *led_topic_sub[number_of_leds];
ros::Subscriber<std_msgs::Bool> *led_state_sub[number_of_leds];


char button_push_topic[number_of_buttons][32];
char button_state_topic[number_of_buttons][32];
char led_toggle_topic[number_of_leds][32];
char led_state_topic[number_of_leds][32];
char adc_value_topic[number_of_adc][32];

void setup()
{

  nh.initNode();

  for(int i = 0; i < number_of_buttons; i++){
     sprintf(button_push_topic[i], "avans/button%i/pushed", i);
     pub_button_pushed[i] = new ros::Publisher(button_push_topic[i], &pushed_msg[i]);
     nh.advertise(*pub_button_pushed[i]);
  }

  for(int i = 0; i < number_of_buttons; i++){
     sprintf(button_state_topic[i], "avans/button%i/state", i);
     pub_button_state[i] = new ros::Publisher(button_state_topic[i], &state_msg[i]);
     nh.advertise(*pub_button_state[i]);
  }
  
  for(int i = 0; i < number_of_buttons; i++){
    //pinMode(led_pin[i], OUTPUT);
    pinMode(button_pin[i], INPUT);
    
    //Enable the pullup resistor on the button
    digitalWrite(button_pin[i], HIGH);
    
    //The button is a normally button
    last_reading[i] = ! digitalRead(button_pin[i]);  
  }

  for(int i = 0; i < number_of_leds; i++){
    sprintf(led_toggle_topic[i], "avans/led%i/toggle", i);
    led_topic_sub[i] = new ros::Subscriber<std_msgs::Empty>(led_toggle_topic[i], ledToggleCb[i]);
    nh.subscribe(*led_topic_sub[i]);
  }


  for(int i = 0; i < number_of_leds; i++){
    sprintf(led_state_topic[i], "avans/led%i/state", i);
    led_state_sub[i] = new ros::Subscriber<std_msgs::Bool>(led_state_topic[i], ledStateCb[i]);
    nh.subscribe(*led_state_sub[i]);
  }


  for(int i = 0; i < number_of_leds; i++){
    pinMode(led_pin[i], OUTPUT);
  }

  for(int i = 0; i < number_of_adc; i++){
    sprintf(adc_value_topic[i], "avans/adc%i/value", i);
    pub_adc_value[i] = new ros::Publisher(adc_value_topic[i], &adc_value_msg[i]);
    
    nh.advertise(*pub_adc_value[i]);   
  }

}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{

  bool reading[number_of_buttons];

  for(int i = 0; i < number_of_buttons; i++){
     reading[i]= ! digitalRead(button_pin[i]);
    
    if (last_reading[i]!= reading[i]){
        last_debounce_time[i] = millis();
        published[i] = false;
    }
    
    //if the button value has not changed for the debounce delay, we know its stable
    if ( !published[i] && (millis() - last_debounce_time[i])  > debounce_delay) {
      pushed_msg[1].data = reading[i];
      pub_button_pushed[i]->publish(&pushed_msg[i]);
      published[i] = true;
    }
  
    state_msg[i].data = reading[i];
    pub_button_state[i]->publish(&state_msg[i]);
  
    last_reading[i] = reading[i];
  }
  
  for(int i = 0; i < number_of_adc; i++){
    int adc_value = averageAnalog(i);
    adc_value_msg[i].data = adc_value;
    pub_adc_value[i]->publish(&adc_value_msg[i]);
  }


  nh.spinOnce();
  //delay(100);
}
