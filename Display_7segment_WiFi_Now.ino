// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Display_7segment_WiFi_Now.ino
    Created:	17/1/2023 8:14:00 μμ
    Author:     ROUSIS_FACTORY\user
*/

/*--------------------------------------------------------------------------------------
  Includes
--------------------------------------------------------------------------------------*/
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#define EEPROM_SIZE 100
#define INVERT_DISPLAY false 
#include <Rousis7segment.h>

#define RXD2 16
#define TXD2 17
#define RS485_PIN_DIR 4
HardwareSerial rs485(1);
#define RS485_WRITE     1
#define RS485_READ      0
static uint16_t CRC_receive = 0xffff;
#define CRC_divisor	 0xA001
uint8_t RS485_en = 0;
//Fire up the DMD library as dmd
#define DISPLAY_DIGITS 4
#define BUZZER 15
Rousis7segment myLED(4, 33, 26, 27, 14);    // Uncomment if not using OE pin
//--------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint8_t broadcastAddress1[] = { 0xB8, 0xD6, 0x1A, 0x35, 0x53, 0x48 };
String success; //10:97:BD:D4:59:C4
esp_err_t result;
//------------------------------------------------------------------------------
uint8_t buzzer_cnt = 0;
uint8_t flash_cnt = 0;
bool flash_on = false;
bool Scan = false;
uint8_t Address;
uint8_t In_bytes_count = 0;

char  Line1_buf[10] = { 0 };
char  Line2_buf[10] = { 0 };
char  Line3_buf[10] = { 0 };
char  Line4_buf[10] = { 0 };
//-----------------------------------------------------------------------
// Define variables to store Queue to be sent
uint16_t queue = 0;
String out_queue;
uint8_t out_counter;
String out_category = "A";

// Define variables to store Queue readings
String in_queue;
uint8_t in_counter;
char in_category;

typedef struct struct_message {
    String queue;
    String device;
    String instruction;
    uint8_t counter;
    String category;
} struct_message;

struct_message Queue_senting;
struct_message Queue_receive;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == 0) {
        success = "Delivery Success :)";
    }
    else {
        success = "Delivery Fail :(";
    }
}

// Callback when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&Queue_receive, incomingData, sizeof(Queue_receive));
    Serial.print("Bytes received: ");
    Serial.println(len);
    out_queue = Queue_receive.queue;
    out_counter = Queue_receive.counter;
    out_category = Queue_receive.category;

    Serial.print("Queue Number: ");
    Serial.println(out_queue);
    Serial.print("Counter: ");
    Serial.println(out_counter);
    Serial.print("Category: ");
    Serial.println(out_category);
    Serial.print("Device: ");
    Serial.println(Queue_receive.device);
    Serial.print("Instruction: ");
    Serial.println(Queue_receive.instruction);
    Serial.println("----------------------------------");

    
    char C = 0xff; uint8_t i = 0;

    char  Line_rec[11] = { "          " };
    while (C)
    {
        C = Queue_receive.queue[i];
        if (C)
        {
            Line_rec[i++] = C;
        }
    }

    //Line_rec[4] = out_counter | 0x30;

    Serial.println("print q:");
    for (i = 0; i < sizeof(Line_rec); i++)
    {
        Serial.print(Line_rec[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("-----------------------------------");
    if (!RS485_en)
    {
        memccpy(Line1_buf, &Line_rec[0], 2, 4);
        myLED.print(Line1_buf, INVERT_DISPLAY);

        flash_on = true;
        flash_cnt = 24;
        buzzer_cnt = 6;
        digitalWrite(BUZZER, HIGH);
    }
}
//-----------------------------------------------------------------------
//Timer setup
//create a hardware timer  of ESP32
hw_timer_t* timer = NULL;
hw_timer_t* flash_timer = NULL;
portMUX_TYPE falshMux = portMUX_INITIALIZER_UNLOCKED;
/*--------------------------------------------------------------------------------------
  Interrupt handler for Timer1 (TimerOne) driven DMD refresh scanning, this gets
  called at the period set in Timer1.initialize();
--------------------------------------------------------------------------------------*/

void IRAM_ATTR FlashInt()
{
    portENTER_CRITICAL_ISR(&falshMux);

    if (flash_cnt)
    {
        if (flash_on)
        {
            myLED.print("    ", INVERT_DISPLAY);

            flash_on = false;
        }
        else {
            myLED.print(Line1_buf, INVERT_DISPLAY);
            flash_on = true;
        }
        flash_cnt--;
    }

    if (buzzer_cnt) {
        buzzer_cnt--;
    }
    else {
        digitalWrite(BUZZER, LOW);
    }

    portEXIT_CRITICAL_ISR(&falshMux);
}

/*--------------------------------------------------------------------------------------
  setup
  Called by the Arduino architecture before the main loop begins
--------------------------------------------------------------------------------------*/
void setup(void)
{
    Serial.begin(115200);
    delay(100);

    rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Project      :  Arduino_ESP32_RS485");
    Serial.print("Info: Intial gpio...");
    pinMode(RS485_PIN_DIR, OUTPUT);
    digitalWrite(RS485_PIN_DIR, RS485_READ);
    Serial.println("done");

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    EEPROM.begin(EEPROM_SIZE);
    /*EEPROM.write(0, 33);
    EEPROM.commit();*/
    Address = EEPROM.read(0);
    //------------------------------------------------------------------------
      // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());
    // Init ESP-NOW
//#if !RS485_ENABLE
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    else {
        Serial.println("initialized ESP - NOW");
    }
//#endif // !RS485_ENABLE  
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer (WiFi NOW)...");
        return;
    }
    else {
        Serial.println("Succesfuly added peer (WiFi NOW)!");
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    //------------------------------------------------------------------------
    // return the clock speed of the CPU
    uint8_t cpuClock = ESP.getCpuFreqMHz();

    flash_timer = timerBegin(1, cpuClock, true);
    timerAttachInterrupt(flash_timer, &FlashInt, true);
    timerAlarmWrite(flash_timer, 100000, true);
    timerAlarmEnable(flash_timer);


    delay(100);


    //EEPROM.end();

   // Serial.begin(115200);
    /*Serial1.begin(9600, SERIAL_8N1, 16, 17);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);*/

    delay(100);
    Serial.println("Start initialize...");

    myLED.displayEnable();     // This command has no effect if you aren't using OE pin
    myLED.normalMode();

    uint8_t n[4];
    int i;
    uint8_t val = 0b00000100;
    //for (i = 0; i < 8; i++) {        

        //val = val >> 1;
    val = 0b00000010;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b00001000;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b01000000;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b00000100;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b00000001;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b00100000;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b00010000;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);
    val = 0b10000000;
    n[0] = val; n[1] = val; n[2] = val; n[3] = val; n[4] = val; n[5] = val;
    myLED.printDirect(n);
    delay(500);

    myLED.print("----", INVERT_DISPLAY);

    buzzer_cnt = 4;
    delay(100);

}

/*--------------------------------------------------------------------------------------
  loop
  Arduino architecture main loop
--------------------------------------------------------------------------------------*/
void loop(void)
{
    if (rs485.available())
    {
        byte get_byte = rs485.read();
        uint8_t instruction = 0;

        if (get_byte == 01)
        {
            char received_pckt[20];
            uint8_t met = 0;
            uint8_t CRC_met = 0;
            CRC_receive = 0xffff;
            instruction = 0;
            received_pckt[met++] = get_byte;
            Put_CRC(get_byte);
            unsigned long startedWaiting = millis();
            while (millis() - startedWaiting <= 200)
            {
                if (rs485.available())
                {
                    get_byte = rs485.read();                  
                    if (!CRC_met) { Put_CRC(get_byte); }
                    received_pckt[met++] = get_byte;
                    if (get_byte == 4 || get_byte == 0xA3)
                    {
                        instruction = get_byte;
                        CRC_met = 2;
                    }
                    else if (CRC_met == 2) {
                        CRC_met = 1;
                    }
                    else if (CRC_met == 1) {
                        //Check CRC here
                        /*Serial.println("Received packed:");
                        for (size_t i = 0; i < sizeof(received_pckt); i++)
                        {
                            Serial.print(received_pckt[i], HEX); Serial.print(' ');
                        }
                        Serial.println(); Serial.println("--------------------------------------");*/
                        uint16_t total_RCR = received_pckt[--met] << 8;
                        total_RCR = total_RCR | received_pckt[--met];
                        if (total_RCR == CRC_receive)
                        {
                            if (instruction == 0xA3)
                            {
                                Replay_OK();
                            } else {
                                met = 0; uint8_t i = 0;
                                while (met < 5) {
                                    get_byte = received_pckt[met + 5];
                                    Line1_buf[met++] = get_byte;
                                }
                                myLED.print(Line1_buf, INVERT_DISPLAY);
                                flash_on = true;
                                flash_cnt = 24;
                                buzzer_cnt = 6;
                                digitalWrite(BUZZER, HIGH);
                                Replay_OK();
                            }                            
                        }
                        exit;
                    }
                }
            }
        }
    }
}

void Replay_OK(void) {
    RS485_en = 1;
    char reply_pckt[] = { 0xAA, 0x55, 'O', 'K', '!' };
    CRC_receive = 0xffff;
    digitalWrite(RS485_PIN_DIR, RS485_WRITE);

    for (size_t i = 0; i < sizeof(reply_pckt); i++)
    {
        rs485.write(reply_pckt[i]);
        Put_CRC(reply_pckt[i]);
    }
    rs485.write(CRC_receive & 0xff);
    rs485.write((CRC_receive >> 8) & 0xff);
    rs485.flush();
    digitalWrite(RS485_PIN_DIR, RS485_READ);
}

void Put_CRC(uint8_t Byte) {
    CRC_receive ^= Byte;    // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
        if ((CRC_receive & 0x0001) != 0) {      // If the LSB is set
            CRC_receive >>= 1;                    // Shift right and XOR 0xA001
            CRC_receive ^= CRC_divisor; //0xA001;
        }
        else                            // Else LSB is not set
            CRC_receive >>= 1;                    // Just shift right
    }
}

