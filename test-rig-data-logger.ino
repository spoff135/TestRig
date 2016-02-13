#include "application.h"
#include "sd-card-library-photon-compat/sd-card-library-photon-compat.h"

//UBIDOTS CODE
#include "HttpClient/HttpClient.h"
#define TOKEN "8dTjkwOhMUN7PzQZG3glUB3v84Nn21"
#define WEBCOUNT "56a445867625423915ed0f0a"
#define WEBDEFLECTION "56a445d376254238f60809eb"
#define PING "56b00e9c76254248641afa00"

HttpClient http;
// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Content-Type", "application/json" },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};

http_request_t request;
http_response_t response;
////UBIDOTS CODE

const uint8_t mosiPin = A5;
const uint8_t misoPin = A4;
const uint8_t clockPin = A3;
const int chipSelect = D4;
char dataString[100];
char sdata[10];

String fileName = "datalog.txt";
int ledPin = D7;

long lastI2C = 0;// time of last I2C communication (ms)
long lastPing = 0;// time of last status update (ms)
long pingRate = 10000;// time between pings (ms)
long pingCount = 0;// number of pings sent

void setup() {
    pinMode(ledPin, OUTPUT);
    Wire.begin(8);                // join i2c bus with address #8
    Wire.onReceive(receiveEvent); // register event
    Serial.begin(9600);           // start serial for output

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        return;
    }
    Serial.println("SD card initialized.");

    //UBIDOTS CODE
    request.hostname = "things.ubidots.com";
    request.port = 80;

    lastPing = millis();
    lastI2C = millis();
}

void loop() {

    // send ping to Ubidots every so often
    if(millis()-lastPing > pingRate){
        pingCount++;
        String msg = String(pingCount);
        request.body = "{\"value\":" + msg + "}";
        request.path = "/api/v1.6/variables/"PING"/values?token="TOKEN;
        http.post(request, response, headers);
        lastPing = millis();
    }

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
    char header;
    String msg = "";

    if(howMany > 0){
        String msg;

        header = Wire.read();

        if(header == '*'){//skip SD card, send to web
            header = Wire.read();

            while (0 < Wire.available()) { // loop through all available data
                char c = Wire.read(); // receive byte as a character
                if(Wire.available() > 0) msg = msg + c; // only write to msg if it's not the last character
            }

            if(header == '1'){ // update cycle count
                digitalWrite(ledPin,HIGH);
                request.body = "{\"value\":" + msg + "}";
                request.path = "/api/v1.6/variables/"WEBCOUNT"/values?token="TOKEN;
                http.post(request, response, headers);

            }
            else if(header == '2'){ // update deflection
                digitalWrite(ledPin,HIGH);
                request.body = "{\"value\":" + msg + "}";
                request.path = "/api/v1.6/variables/"WEBDEFLECTION"/values?token="TOKEN;
                http.post(request, response, headers);

            }
            else{
                //do nothing?
            }

            digitalWrite(ledPin,LOW);
        }
        else{

            digitalWrite(ledPin,HIGH);
            File dataFile = SD.open(fileName, FILE_WRITE);

            // write header to serial & file
            Serial.print(header);
            msg = msg + header;
            if (dataFile) dataFile.print(header);

            // do the same for the rest of the data
            while (0 < Wire.available()) { // loop through all available data
                char c = Wire.read(); // receive byte as a character
                Serial.print(c);         // print the character
                msg = msg + c;
                if (dataFile) dataFile.print(c); // print the character to file
            }
            if(!dataFile) Serial.println("error opening " + fileName); // if the file isn't open, pop up an error
            else Serial.println(" " + String(howMany) + " byte msg printed to file.");

            dataFile.close();
            digitalWrite(ledPin,LOW);
        }
/*
        else{
            while(0 < Wire.available()){
                char c = Wire.read();
                msg = msg + c;
            }
            if(header == '1'){
            }
            else if(header == '2'){
                // send deflection
            }
            else if(header == '3'){
                // send other?
            }
        }
*/

    }
}
