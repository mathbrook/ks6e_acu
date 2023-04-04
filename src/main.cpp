/*Testing live rinehart torque control with MCP3204 and teensy 4.1*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <KS2e.h>
//GPIOs
//const int rtdButtonPin=33,TorqueControl=34,LaunchControl=35,InverterRelay=14;
int pixelColor=0;
bool inverter_restart = false;
Metro cantest = Metro(100);
Metro can = Metro(10);
Metro fantest = Metro(50);
#define DEBUG true
//objects
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_2;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;
static CAN_message_t testMsg;
static CAN_message_t dashMsg;
/*****PROTOTYPES*****/
void read_relay_values();
void get_relay_states();
//void readBroadcast();
void setup()
{
    Serial.begin(115200);



    leds.begin();
    leds.setBrightness(BRIGHTNESS);
    CAN_1.begin();
    CAN_1.setBaudRate(500000);
    CAN_2.begin();
    CAN_2.setBaudRate(500000);

    CAN_1.setMaxMB(NUM_TX_MAILBOXES+NUM_RX_MAILBOXES);
    for (int i = 0; i<NUM_RX_MAILBOXES; i++){
        CAN_1.setMB((FLEXCAN_MAILBOX)i,RX,STD);
    }
    for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
        CAN_1.setMB((FLEXCAN_MAILBOX)i,TX,STD);
    }

    CAN_2.setMaxMB(NUM_TX_MAILBOXES+NUM_RX_MAILBOXES);
    for (int i = 0; i<(NUM_RX_MAILBOXES-1); i++){//leave one free for ext ID
         CAN_2.setMB((FLEXCAN_MAILBOX)i,RX,STD);
    }
    for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
        CAN_2.setMB((FLEXCAN_MAILBOX)i,TX,STD);
    }
     CAN_2.setMB((FLEXCAN_MAILBOX)5,RX,EXT);
    CAN_1.mailboxStatus();
    CAN_2.mailboxStatus();
    DashLedscolorWipe(WHITE);
    delay(50);
    DashLedscolorWipe(PINK);
    delay(50);
    DashLedscolorWipe(GREEN);
}

void loop()
{
    read_relay_values();
    get_relay_states();
    // if(cantest.check()){
    //     testMsg.buf[0] = 0x68;
    //     testMsg.id = 0x68;
    //     CAN_1.write(testMsg);
    //     // CAN_2.write(testMsg);
    //     DashLedscolorWipe(pixelColor);
        
    //     pixelColor++;
    //     if(pixelColor>7){
    //         pixelColor=0;
    //     }
    // }

    if(can.check()){
        dashMsg.buf[0] = 0x68;
        dashMsg.buf[1] = imdstate;
        dashMsg.buf[2] = bmsstate;
        dashMsg.id = 0x68;
        CAN_1.write(dashMsg);
    }

    if(fantest.check()){
        // digitalToggle(FAN_CTRL);
        // Serial.printf("imd relay: %f bms relay: %f imd gpio: %f bms gpio: %f\n",imdrelay,bmsrelay,imdgpio,bmsgpio);
        // Serial.printf("RAW imd relay: %f RAW bms relay: %f RAW imd gpio: %f RAW bms gpio: %f\n",ADC.read_adc(IMD_RELAY),ADC.read_adc(BMS_RELAY),ADC.read_adc(IMD_GPIO),ADC.read_adc(BMS_GPIO));
        //Serial.printf("Bodge imd relay: %f RAW Bodge bms relay: %f\n",BODGEimdrelay,BODGEbmsrelay);
        //Serial.printf("Bodge imd relay: %f RAW Bodge bms relay: %f\n",analogRead(ANALOG_IMD),analogRead(ANALOG_BMS));
        //Serial.println("");
        #if DEBUG;
        Serial.print("IMD: ");
        Serial.println(analogRead(BODGEimdrelay));
        Serial.print("BMS: ");
        Serial.println(analogRead(BODGEbmsrelay));

        Serial.print("IMD: ");
        Serial.println(imdstate);
        Serial.print("BMS: ");
        Serial.println(bmsstate);

        Serial.println("");
        #endif


    }

}


inline void read_relay_values() { // Changed to relay
    /* Filter ADC readings */
    // imdrelay = ALPHA * imdrelay + (1 - ALPHA) * ADC.read_adc(IMD_RELAY);
    // bmsrelay = ALPHA * bmsrelay + (1 - ALPHA) * ADC.read_adc(BMS_RELAY);
    // imdgpio = ALPHA * imdgpio + (1 - ALPHA) * ADC.read_adc(IMD_GPIO);
    // bmsgpio = ALPHA * bmsgpio + (1 - ALPHA) * ADC.read_adc(BMS_GPIO);

    BODGEimdrelay = analogRead(ANALOG_IMD);
    BODGEbmsrelay = analogRead(ANALOG_BMS);
    //we dont have 2 brake sensors so commented out
    // filtered_brake2_reading = ALPHA * filtered_brake2_reading + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_2_CHANNEL);


}

void get_relay_states() {
    if(analogRead(ANALOG_IMD)<10) {
        imdstate=true;
    }
    else {
        imdstate=false;
    }

    if(analogRead(ANALOG_BMS)<10) {
        bmsstate=true;
    }
    else {
        bmsstate=false;
    }
}

/* Shared state functinality */