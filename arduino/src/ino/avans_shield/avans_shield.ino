/* 
 * Avans Arduin shield
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>


ros::NodeHandle nh;

//#define UNO
#define MEGA

#ifdef UNO
#undef MEGA
#endif

#ifdef UNO
#define NUMBER_OF_BUTTON    2
#define NUMBER_OF_LED       2
#define NUMBER_OF_ADC       1
#endif

#ifdef MEGA
#define NUMBER_OF_BUTTON    4
#define NUMBER_OF_LED       8
#define NUMBER_OF_ADC       2
#endif


std_msgs::Bool pushed_msg[NUMBER_OF_BUTTON];
std_msgs::Bool state_msg[NUMBER_OF_BUTTON];
ros::Publisher *pub_button_pushed[NUMBER_OF_BUTTON];
ros::Publisher *pub_button_state[NUMBER_OF_BUTTON];


#if NUMBER_OF_BUTTON > 0
const int button_pin[] = {8, 9, 10, 11};

bool last_reading[NUMBER_OF_BUTTON];
long last_debounce_time[NUMBER_OF_BUTTON] = {0};
long debounce_delay=50;
bool published[NUMBER_OF_BUTTON] = {true};
#endif

#if NUMBER_OF_LED > 0
const int led_pin[] = {12, 13, 2, 3, 4, 5, 6, 7};
#endif

#if NUMBER_OF_ADC > 0
std_msgs::Int16 adc_value_msg[NUMBER_OF_ADC];
ros::Publisher *pub_adc_value[NUMBER_OF_ADC];
#endif


#if NUMBER_OF_LED > 0
void ledToggle(int led_pin){
  digitalWrite(led_pin, HIGH-digitalRead(led_pin));   // blink the led
}
#endif

#define GENERATE_LED_TOGGLE_FUNC(__index) \
    void ledToggleCb##__index(const std_msgs::Empty& toggle_msg){\
    ledToggle(led_pin[__index]);}  \


#if NUMBER_OF_LED > 0
GENERATE_LED_TOGGLE_FUNC(0)
#endif
#if NUMBER_OF_LED > 1
GENERATE_LED_TOGGLE_FUNC(1)
#endif
#if NUMBER_OF_LED > 2
GENERATE_LED_TOGGLE_FUNC(2)
#endif
#if NUMBER_OF_LED > 3
GENERATE_LED_TOGGLE_FUNC(3)
#endif
#if NUMBER_OF_LED > 4
GENERATE_LED_TOGGLE_FUNC(4)
#endif
#if NUMBER_OF_LED > 5
GENERATE_LED_TOGGLE_FUNC(5)
#endif
#if NUMBER_OF_LED > 6
GENERATE_LED_TOGGLE_FUNC(6)
#endif
#if NUMBER_OF_LED > 7
GENERATE_LED_TOGGLE_FUNC(7)
#endif


typedef void (*led_toggle_func_cb)(const std_msgs::Empty&);  

#if NUMBER_OF_LED > 0
led_toggle_func_cb ledToggleCb[]={ledToggleCb0,
#if NUMBER_OF_LED > 1
                                  ledToggleCb1,
#endif
#if NUMBER_OF_LED > 2
                                  ledToggleCb2,
#endif
#if NUMBER_OF_LED > 3
                                  ledToggleCb3,
#endif
#if NUMBER_OF_LED > 4
                                  ledToggleCb4,
#endif
#if NUMBER_OF_LED > 5
                                  ledToggleCb5,
#endif
#if NUMBER_OF_LED > 6
                                  ledToggleCb6,
#endif
#if NUMBER_OF_LED > 7
                                  ledToggleCb7
#endif
}
                                   ;
#endif
                                  
                                  
                                  
void ledState(int led_pin, const std_msgs::Bool& state_msg){
  digitalWrite(led_pin, state_msg.data);
}


#define GENERATE_LED_STATE_FUNC(__index) \
    void ledStateCb##__index(const std_msgs::Bool& state_msg){\
    ledState(led_pin[__index], state_msg);}  \
    

#if NUMBER_OF_LED > 0
GENERATE_LED_STATE_FUNC(0)
#endif
#if NUMBER_OF_LED > 1
GENERATE_LED_STATE_FUNC(1)
#endif
#if NUMBER_OF_LED > 2
GENERATE_LED_STATE_FUNC(2)
#endif
#if NUMBER_OF_LED > 3
GENERATE_LED_STATE_FUNC(3)
#endif
#if NUMBER_OF_LED > 4
GENERATE_LED_STATE_FUNC(4)
#endif
#if NUMBER_OF_LED > 5
GENERATE_LED_STATE_FUNC(5)
#endif
#if NUMBER_OF_LED > 6
GENERATE_LED_STATE_FUNC(6)
#endif
#if NUMBER_OF_LED > 7
GENERATE_LED_STATE_FUNC(7)
#endif

typedef void (*led_state_func_cb)(const std_msgs::Bool&); 

#if NUMBER_OF_LED > 0
led_state_func_cb ledStateCb[]={ledStateCb0,
#if NUMBER_OF_LED > 1
                                  ledStateCb1,
#endif
#if NUMBER_OF_LED > 2
                                  ledStateCb2,
#endif
#if NUMBER_OF_LED > 3
                                  ledStateCb3,
#endif
#if NUMBER_OF_LED > 4
                                  ledStateCb4,
#endif
#if NUMBER_OF_LED > 5
                                  ledStateCb5,
#endif
#if NUMBER_OF_LED > 6
                                  ledStateCb6,
#endif
#if NUMBER_OF_LED > 7
                                  ledStateCb7
#endif
                                  }
                                  ;
#endif

#if NUMBER_OF_LED > 0
ros::Subscriber<std_msgs::Empty> *led_topic_sub[NUMBER_OF_LED];
ros::Subscriber<std_msgs::Bool> *led_state_sub[NUMBER_OF_LED];
char led_toggle_topic[NUMBER_OF_LED][32];
char led_state_topic[NUMBER_OF_LED][32];
char adc_value_topic[NUMBER_OF_ADC][32];
#endif

#if NUMBER_OF_BUTTON > 0
char button_push_topic[NUMBER_OF_BUTTON][32];
char button_state_topic[NUMBER_OF_BUTTON][32];
#endif

void setup()
{

  nh.initNode();
#if NUMBER_OF_BUTTON > 0
  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
     sprintf(button_push_topic[i], "avans/button%i/pushed", i);
     pub_button_pushed[i] = new ros::Publisher(button_push_topic[i], &pushed_msg[i]);
     nh.advertise(*pub_button_pushed[i]);
  }

  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
     sprintf(button_state_topic[i], "avans/button%i/state", i);
     pub_button_state[i] = new ros::Publisher(button_state_topic[i], &state_msg[i]);
     nh.advertise(*pub_button_state[i]);
  }
  
  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
    //pinMode(led_pin[i], OUTPUT);
    pinMode(button_pin[i], INPUT);
    
    //Enable the pullup resistor on the button
    digitalWrite(button_pin[i], HIGH);
    
    //The button is a normally button
    last_reading[i] = ! digitalRead(button_pin[i]);  
  }
#endif


#if NUMBER_OF_LED > 0
  for(int i = 0; i < NUMBER_OF_LED; i++){
    sprintf(led_toggle_topic[i], "avans/led%i/toggle", i);
    led_topic_sub[i] = new ros::Subscriber<std_msgs::Empty>(led_toggle_topic[i], ledToggleCb[i]);
    nh.subscribe(*led_topic_sub[i]);
  }


  for(int i = 0; i < NUMBER_OF_LED; i++){
    sprintf(led_state_topic[i], "avans/led%i/state", i);
    led_state_sub[i] = new ros::Subscriber<std_msgs::Bool>(led_state_topic[i], ledStateCb[i]);
    nh.subscribe(*led_state_sub[i]);
  }


  for(int i = 0; i < NUMBER_OF_LED; i++){
    pinMode(led_pin[i], OUTPUT);
  }
#endif

#if NUMBER_OF_ADC > 0
  for(int i = 0; i < NUMBER_OF_ADC; i++){
    sprintf(adc_value_topic[i], "avans/adc%i/value", i);
    pub_adc_value[i] = new ros::Publisher(adc_value_topic[i], &adc_value_msg[i]);
    
    nh.advertise(*pub_adc_value[i]);   
  }
#endif
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{

  bool reading[NUMBER_OF_BUTTON];

#if NUMBER_OF_BUTTON > 0
  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
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
#endif

#if NUMBER_OF_ADC > 0
  for(int i = 0; i < NUMBER_OF_ADC; i++){
    int adc_value = averageAnalog(i);
    adc_value_msg[i].data = adc_value;
    pub_adc_value[i]->publish(&adc_value_msg[i]);
  }
#endif


  nh.spinOnce();
  //delay(100);
}
