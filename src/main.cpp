/*
    New ACU code for:
    -reading isolated ACC CAN for temp data
    -performing min/max/avg functions on temp data
    -sending temp data to BMS
    -reading IMD & BMS states via relays
    -sending IMD & BMS states to VCU (and thus to dash)
    
    Hopefully:
    -controlling fans
    -controlling dash lights...?
    -using onboard neopixels for showing state
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <KS2e.h>
#include <MDB_labels.h>
#include <math.h>
#include <ADC_SPI.h>
#define DEBUG
// #define cantest

// BIG DEFINES FOR MODULES & SWITCH CASE (absolute cancer code)
#define NUMBER_OF_CELLS 72
#define NUMBER_OF_MODULES 6
#define CELLS_PER_MODULE 12
#define CELLS_1A CELLS_PER_MODULE*0/2
#define CELLS_1B CELLS_PER_MODULE*1/2
#define CELLS_2A CELLS_PER_MODULE*2/2
#define CELLS_2B CELLS_PER_MODULE*3/2
#define CELLS_3A CELLS_PER_MODULE*4/2
#define CELLS_3B CELLS_PER_MODULE*5/2
#define CELLS_4A CELLS_PER_MODULE*6/2
#define CELLS_4B CELLS_PER_MODULE*7/2
#define CELLS_5A CELLS_PER_MODULE*8/2
#define CELLS_5B CELLS_PER_MODULE*9/2
#define CELLS_6A CELLS_PER_MODULE*10/2
#define CELLS_6B CELLS_PER_MODULE*11/2

//objects
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ACC_1;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;
static CAN_message_t testMsg;
static CAN_message_t dashMsg;
static CAN_message_t rxMsg;
static CAN_message_t vi_measurementsMsg;

// CAN IDs
#define BMS_ID 0x7E3
#define ThermistorToBMS_ID 0x9839F380
#define ThermistorAddressClaim_ID 0x98EEFF80 // probably unnecessary
#define BMS_Response_ID 0x7EB

// CAN bytes
int8_t rawBatteryTemps[NUMBER_OF_CELLS];
int8_t batteryTemps[NUMBER_OF_CELLS];
float ratioTemps; // This is just to make the array a temp variable
float floatTemps; // This is to save the math as a float first
float cal5 = -0.000002416676401;
float cal4 = 0.001082617446913;
float cal3 = -0.194488265848684; // First part of the ^3 best fit
float cal2 = 17.519770902801400; // Second
float cal1 = -792.865188960333000; // Third
float calIntercept = 14494.861100594600000;



// floatTemps=((((cal5V*rawBatteryTe`1  qaaaaaaa    1qA1QAz1amps[i])/cal255)*(calM))+calB);

bool goodID=false;


int moduleNo = 0;                                                         // byte0
int enabledTherm;                                                         // byte4
byte getLowestTemp[] = {0x03, 0x22, 0xF0, 0x28, 0x55, 0x55, 0x55, 0x55};  // lowest temp request
byte getHighestTemp[] = {0x03, 0x22, 0xF0, 0x29, 0x55, 0x55, 0x55, 0x55}; // highest temp request

// CAN timings
Metro getACCCanRate = Metro(5,1);
Metro getTempRate = Metro(500,1);
Metro sendTempRate = Metro(100,1);
Metro sendCAN_1 = Metro(50,1);
Metro sendCANTest = Metro(50,1);

// Regular timings
Metro fanTest = Metro(5000);
Metro heartBeat = Metro(500);
Metro getRelay = Metro(100);
Metro printDebug = Metro(1000);

// Globals
int globalHighTherm = 25, globalLowTherm = 25;
int pixelColor=0;
bool inverter_restart = false;

/*****PROTOTYPES*****/
void get_relay_states();
void get_vi_measurements();
int readACC_1(CAN_message_t &msg);
void updateAccumulatorCAN();
void getTempData();
void sendTempData();
//void readBroadcast();
ADC_SPI pedal_ADC;
// Setup -----------------------------------------------------------------------
void setup()
{
    pedal_ADC = ADC_SPI(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
    pinMode(FAN_CTRL,OUTPUT);
    analogReadResolution(8); //12.890625mV per bit at 8bit resolution (3.3v/256)
    Serial.begin(115200);
    delay(400);

    // This is the main raw battery temp array for  
    Serial.println("Raw battery temp array: ");
    for (int i = 0; i < NUMBER_OF_CELLS; i++)
    {
        Serial.print("Cell number: ");
        Serial.print(i);
        Serial.print(" Value: ");
        rawBatteryTemps[i] = 0; // init default temps as a safe value
        Serial.println(rawBatteryTemps[i]);
    }

    // This is the main calibrated battery temp array 
    Serial.println("Battery temp array: ");
    for (int i = 0; i < NUMBER_OF_CELLS; i++)
    {
        Serial.print("Cell number: ");
        Serial.print(i);
        Serial.print(" Value: ");
        batteryTemps[i] = 0; // init default temps as a safe value
        Serial.println(batteryTemps[i]);
    }

    // CANbus setups
    CAN_1.begin();
    CAN_1.setBaudRate(500000);
    ACC_1.begin();
    ACC_1.setBaudRate(500000);
    CAN_1.setMaxMB(NUM_TX_MAILBOXES+NUM_RX_MAILBOXES);
    for (int i = 0; i<NUM_RX_MAILBOXES; i++){
        CAN_1.setMB((FLEXCAN_MAILBOX)i,RX,STD);
    }
    for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
        CAN_1.setMB((FLEXCAN_MAILBOX)i,TX,STD);
    }

    ACC_1.setMaxMB(NUM_TX_MAILBOXES+NUM_RX_MAILBOXES);
    for (int i = 0; i<(NUM_RX_MAILBOXES-1); i++){//leave one free for ext ID
         ACC_1.setMB((FLEXCAN_MAILBOX)i,RX,STD);
    }
    for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
        ACC_1.setMB((FLEXCAN_MAILBOX)i,TX,STD);
    }
     ACC_1.setMB((FLEXCAN_MAILBOX)5,RX,EXT);
    CAN_1.mailboxStatus();
    ACC_1.mailboxStatus();

    // Dash stuff
    leds.begin();
    leds.setBrightness(BRIGHTNESS);
    DashLedscolorWipe(WHITE);
    delay(50);
    DashLedscolorWipe(PINK);
    delay(50);
    DashLedscolorWipe(GREEN);
}
// Setup -----------------------------------------------------------------------

// Main loop -----------------------------------------------------------------------
void loop()
{
    if(heartBeat.check()) {
        // digitalToggle(LED_BUILTIN);
        // Sadly this has to be disabled in order for the SPI comms to work (The SPI peripheral uses pin 13 which is shared with the LED)
    }

    if (getACCCanRate.check()) {
        updateAccumulatorCAN();
    }

    if (sendTempRate.check()){
        sendTempData();
    }

    if(getRelay.check()){
        get_relay_states();
        get_vi_measurements();
    }

    if(sendCAN_1.check()){
        dashMsg.buf[0] = ID_ACU_RELAY;
        dashMsg.buf[1] = imdstate;
        dashMsg.buf[2] = bmsstate;
        dashMsg.buf[3] = imdgpiostate;
        dashMsg.buf[4] = bmsgpiostate;
        dashMsg.id = ID_ACU_RELAY;
        CAN_1.write(dashMsg);

        vi_measurementsMsg.id = ID_ACU_MEASUREMENTS;
        vi_measurementsMsg.buf[0] = vsense12v;
        vi_measurementsMsg.buf[1] = sdcvsense;
        vi_measurementsMsg.buf[2] = vsense5v;
        vi_measurementsMsg.buf[3] = sense12v;
        vi_measurementsMsg.buf[4] = sdcsense;
        vi_measurementsMsg.buf[5] = sensefan;
        vi_measurementsMsg.buf[6] = humidity;
        vi_measurementsMsg.buf[7] = temp;
        CAN_1.write(vi_measurementsMsg);
    }

    if(fanTest.check()){
        // digitalToggle(FAN_CTRL);
        digitalWrite(FAN_CTRL,HIGH); //Keep this HIGH cuz if you turn off the gpio the mosfet gate will get pulled high and right now there is no clamp zener to protect it
    }

#ifdef cantest
    if(sendCANTest.check()){
        testMsg.buf[0] = 0x68;
        testMsg.id = 0x68;
        CAN_1.write(testMsg);
        // CAN_2.write(testMsg);
        }
#endif

#ifdef DEBUG 
    if(printDebug.check()){
        // Serial.printf("imd relay: %f bms relay: %f imd gpio: %f bms gpio: %f\n",imdrelay,bmsrelay,imdgpio,bmsgpio);
        // Serial.printf("RAW imd relay: %f RAW bms relay: %f RAW imd gpio: %f RAW bms gpio: %f\n",ADC.read_adc(IMD_RELAY),ADC.read_adc(BMS_RELAY),ADC.read_adc(IMD_GPIO),ADC.read_adc(BMS_GPIO));
        //Serial.printf("Bodge imd relay: %f RAW Bodge bms relay: %f\n",BODGEimdrelay,BODGEbmsrelay);
        //Serial.printf("Bodge imd relay: %f RAW Bodge bms relay: %f\n",analogRead(ANALOG_IMD),analogRead(ANALOG_BMS));
        //Serial.println("");
        
        // Serial.print("IMD: ");
        // Serial.println(analogRead(BODGEimdrelay));
        // Serial.print("BMS: ");
        // Serial.println(analogRead(BODGEbmsrelay));

        // Serial.print("IMD: ");
        // Serial.println(imdstate);
        // Serial.print("BMS: ");
        // Serial.println(bmsstate);
        
        // Serial.println("");
    }
#endif

}
// Main loop -----------------------------------------------------------------------

// Read the ACC CAN
int readACC_1(CAN_message_t &msg)
{    
  int rxMSG = ACC_1.read(msg);
  return rxMSG;
}

// Updates battery temp array with the values from the isolated ACC CANbus via a switch case
void updateAccumulatorCAN()
{  
    CAN_message_t rxMsg;
    if (readACC_1(rxMsg))
    {
        #ifdef DEBUG
        // Serial.print("MB "); Serial.print(rxMsg.mb);
        // Serial.print("  OVERRUN: "); Serial.print(rxMsg.flags.overrun);
        // Serial.print("  LEN: "); Serial.print(rxMsg.len);
        // Serial.print(" EXT: "); Serial.print(rxMsg.flags.extended);
        // Serial.print(" TS: "); Serial.print(rxMsg.timestamp);
        // Serial.print(" ID: "); Serial.print(rxMsg.id, HEX);
        // Serial.print(" Buffer: ");
        // for ( uint8_t i = 0; i < rxMsg.len; i++ ) {
        // Serial.print(rxMsg.buf[i], HEX); Serial.print(" ");
        // } 
        // Serial.println();
        #endif

        switch (rxMsg.id)  // This is cancer probably and could better be implemented with a loop I imagine
        {
        case (MODULE_1_A): 
        {
            // Serial.println("Module 1 A");
            memcpy(rawBatteryTemps+CELLS_1A,rxMsg.buf,6);
            break;
        }
        case (MODULE_1_B): 
        {
            // Serial.println("Module 1 B");
            memcpy(rawBatteryTemps+CELLS_1B,rxMsg.buf,6);
            break;
        }
        case (MODULE_2_A): 
        {
            // Serial.println("Module 2 A");
            memcpy(rawBatteryTemps+CELLS_2A,rxMsg.buf,6);
            break;
        }
        case (MODULE_2_B): 
        {
            // Serial.println("Module 2 B");
            memcpy(rawBatteryTemps+CELLS_2B,rxMsg.buf,6);
            break;
        }
        case (MODULE_3_A): 
        {
            // Serial.println("Module 3 A");
            memcpy(rawBatteryTemps+CELLS_3A,rxMsg.buf,6);
            break;
        }
        case (MODULE_3_B): 
        {
            // Serial.println("Module 3 B");
            memcpy(rawBatteryTemps+CELLS_3B,rxMsg.buf,6);
            break;
        }
        case (MODULE_4_A): 
        {
            // Serial.println("Module 4 A");
            memcpy(rawBatteryTemps+CELLS_4A,rxMsg.buf,6);
            break;
        }
        case (MODULE_4_B): 
        {
            // Serial.println("Module 4 B");
            memcpy(rawBatteryTemps+CELLS_4B,rxMsg.buf,6);
            break;
        }
        case (MODULE_5_A): 
        {
            // Serial.println("Module 5 A");
            memcpy(rawBatteryTemps+CELLS_5A,rxMsg.buf,6);
            break;
        }
        case (MODULE_5_B): 
        {
            // Serial.println("Module 5 B");
            memcpy(rawBatteryTemps+CELLS_5B,rxMsg.buf,6);
            break;
        }
        case (MODULE_6_A): 
        {
            // Serial.println("Module 6 A");
            memcpy(rawBatteryTemps+CELLS_6A,rxMsg.buf,6);
            break;
        }
        case (MODULE_6_B): 
        {
            // Serial.println("Module 6 B");
            memcpy(rawBatteryTemps+CELLS_6B,rxMsg.buf,6);
            break;
        }
        default:
        {
            break;
        }
        }
    }
}

// Sending highest/lowest temperature to the BMS
void sendTempData()
{
    CAN_message_t sendTempMsg;
    sendTempMsg.flags.extended = 1;      // extended id
    sendTempMsg.len = 8;                 // per protocol
    sendTempMsg.id = ThermistorToBMS_ID; // Temp broadcast ID
    enabledTherm = NUMBER_OF_CELLS - 1;  // number of cells 0 based

    // This is assigning the calibrated battery temp array from the raw recieved one
    for (int i = 0; i < NUMBER_OF_CELLS; i++)
    {

        // Below is some big BS lmao
        ratioTemps = rawBatteryTemps[i]; // Sets the current part of the array to a temp variable so math functions can be done on it

        floatTemps=(cal5*pow(ratioTemps,5))+(cal4*pow(ratioTemps,4))+(cal3*pow(ratioTemps,3))+(cal2*pow(ratioTemps,2))+(cal1*(ratioTemps))+calIntercept; // Performs the calibration curve math
        
        batteryTemps[i]=round(floatTemps); // Rounds up or down according to standard practice before setting it back equal to battery temps
        
        #ifdef DEBUG
        Serial.print("Cell number: ");
        Serial.print(i);
        Serial.print("Raw Value: ");
        Serial.println(rawBatteryTemps[i]);

        Serial.print("Cell number: ");
        Serial.print(i);
        Serial.print("floatTemps Value: ");
        Serial.println(floatTemps);

        Serial.print("Cell number: ");
        Serial.print(i);
        Serial.print(" Value: ");
        Serial.println(batteryTemps[i]);

        Serial.println();
        #endif

    }

    // Initalize the highest and lowest
    int lowTherm = batteryTemps[0];
    int lowestThermId = 0;
    int highTherm = batteryTemps[0];
    int highestThermId = 0;
    for (int i = 0; i < NUMBER_OF_CELLS; i++)
    { // get lowest and highest
        
        if(i>=0 && i<=5){
            goodID=true;
        }
        else if(i>=12 && i<=15){
            goodID=true;
        }
        else if(i>=17 && i<=17){
            goodID=true;
        }
        else if(i>=36 && i<=41){
            goodID=true;
        }
        else if(i>=48 && i<=53){
            goodID=true;
        }
        else{
            goodID=false;
        }
    
        if(goodID){
            if (batteryTemps[i] < lowTherm)
            {
                lowTherm = batteryTemps[i];
                lowestThermId = i;
            }

            if (batteryTemps[i] > highTherm)
            {
                highTherm = batteryTemps[i];
                highestThermId = i;
            }
            
            #ifdef DEBUG
                // Serial.printf("Iter: %d Highest: %d Lowest: %d\n",i,highTherm,lowTherm);
            #endif  
        }
    }

    #ifdef DEBUG
    // Serial.print("Cell number: ");
    // Serial.print(lowestThermId);
    // Serial.print(" Min Value: ");
    // Serial.println(lowTherm);

    // Serial.print("Cell number: ");
    // Serial.print(highestThermId);
    // Serial.print(" Max Value: ");
    // Serial.println(highTherm);
    #endif

    int avgTherm = (lowTherm + highTherm) / 2;                                                                          // yep
    int checksum = moduleNo + lowTherm + highTherm + avgTherm + enabledTherm + highestThermId + lowestThermId + 57 + 8; // 0x39 and 0x08 added to checksum per orion protocol
    byte tempdata[] = {moduleNo, lowTherm, highTherm, avgTherm, enabledTherm, highestThermId, lowestThermId, checksum};
    #ifdef DEBUG
    // Serial.println(tempdata[2]);
    #endif
    memcpy(sendTempMsg.buf, tempdata, sizeof(sendTempMsg.buf));
    CAN_1.write(sendTempMsg);
    // GLobal ints for tracking
    globalHighTherm = highTherm;
    globalLowTherm = lowTherm;
}

// getting one of the max temps from BMS (high or low not sure lol)
void getTempData()
{
  CAN_message_t getTempMsg;
  getTempMsg.flags.extended = 1;
  getTempMsg.len = 8;
  getTempMsg.id = BMS_ID; // OUR BMS
  memcpy(getTempMsg.buf, getLowestTemp, sizeof(getTempMsg.buf));
  CAN_1.write(getTempMsg);
  Serial.println("Requesting Lowest Temp Data...");
  memcpy(getTempMsg.buf, getHighestTemp, sizeof(getTempMsg.buf));
  CAN_1.write(getTempMsg);
  Serial.println("Requesting Highest Temp Data...");
}

// analogRead for the IMD & BMS states
void get_relay_states() { // Changed to relay
    /* Filter ADC readings */
    imdrelay = pedal_ADC.read_adc(IMD_RELAY);
    bmsrelay = pedal_ADC.read_adc(BMS_RELAY);
    imdgpio = pedal_ADC.read_adc(IMD_GPIO);
    bmsgpio = pedal_ADC.read_adc(BMS_GPIO);
    #ifdef DEBUG

    Serial.printf("ADC Channel IMDRELAY: %d BMSRELAY: %d IMDGPIO: %d BMSGPIO: %d\n",imdrelay,bmsrelay,imdgpio,bmsgpio);
    #endif
    // For all of these thingies, true = good state, false = bad state
    /*
    Values at 12.4V input to the board:
    when relays are closed and GPIOs are in OK state:
    ADC Channel IMDRELAY: 2 BMSRELAY: 0 IMDGPIO: 2167 BMSGPIO: 2124
    IMD Relay State: 1 IMD Gpio State: 1 BMS Relay State: 1 BMS GPIO State: 1
    when relays are open and GPIOs are in FAULT state:
    ADC Channel IMDRELAY: 2230 BMSRELAY: 2141 IMDGPIO: 0 BMSGPIO: 0
    IMD Relay State: 0 IMD Gpio State: 0 BMS Relay State: 0 BMS GPIO State: 0

    Values at 14.4v input 
    when relays are closed and GPIOs are in OK state:
    ADC Channel IMDRELAY: 2 BMSRELAY: 0 IMDGPIO: 2293 BMSGPIO: 2250
    IMD Relay State: 1 IMD Gpio State: 1 BMS Relay State: 1 BMS GPIO State: 1
    when relays are open and GPIOs are in FAULT state:
    ADC Channel IMDRELAY: 2313 BMSRELAY: 2265 IMDGPIO: 0 BMSGPIO: 0
    IMD Relay State: 0 IMD Gpio State: 0 BMS Relay State: 0 BMS GPIO State: 0
    */
    if(imdrelay<50) {
        imdstate=true;
    }
    else {
        imdstate=false;
    }
    if(bmsrelay<50) {
        bmsstate=true;
    }
    else {
        bmsstate=false;
    }
    if(imdgpio>500) {
        imdgpiostate=true;
    }
    else {
        imdgpiostate=false;
    }
    if(bmsgpio>500) {
        bmsgpiostate=true;
    }
    else {
        bmsgpiostate=false;
    }
    #ifdef DEBUG
    Serial.printf("\nIMD Relay State: %d IMD Gpio State: %d BMS Relay State: %d BMS GPIO State: %d\n\n",imdstate,imdgpiostate,bmsstate,bmsgpiostate);
    #endif
}
void get_vi_measurements(){
    sdcvsense=analogRead(VSENSE_12V);
    vsense5v=analogRead(VSENSE_5V);
    vsense12v=analogRead(VSENSE_SDC);
    sdcsense=analogRead(SDC_SENSE);
    sense12v=analogRead(SENSE_12V);
    sensefan=analogRead(SENSE_FAN);
    humidity=analogRead(ANALOG_BMS);
    temp=analogRead(ANALOG_IMD);
    #ifdef DEBUG
    Serial.printf("\nSDC voltage: %d current: %d 12v voltage: %d current: %d 5v voltage: %d fan current: %d humidity v: %d temp v: %d\n",sdcvsense,sdcsense,vsense12v,sense12v,vsense5v,sensefan,humidity,temp);
    #endif
}