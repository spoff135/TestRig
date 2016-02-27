//UBIDOTS CODE
#include "HttpClient.h"
#define TOKEN "Me72mxyaIELmctKM81Y0DVY3MTUJ4z"
#define WEBDEFLECTION "56b671fb76254228866ee416"
#define WEBCYCLECOUNT "56b672cb7625422dd8dbbf52"

HttpClient http;
// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Content-Type", "application/json" },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};

http_request_t request;
http_response_t response;
//UBIDOTS CODE

int cycleCount = 0;
float deflectionAvg = 1;
bool webUpdate = false;

void setup()
{
  //UBIDOTS CODE
  request.hostname = "things.ubidots.com";
  request.port = 80;

  Spark.function("run",WebRunFunction);

}

void loop()
{
  if(webUpdate){
    String valuesMsg = "";

    request.body = "[{\"variable\":"WEBCYCLECOUNT", \"value\": "+String(cycleCount)+" }, { \"variable\":"WEBDEFLECTION", \"value\": "+String(deflectionAvg,3)+" }]";
    request.path = "/api/v1.6/collections/values/";

    //  request.body = "{\"value\":" + String(deflectionAvg,3) + "}";
    //  request.path = "/api/v1.6/variables/"WEBDEFLECTION"/values?token="TOKEN;
    http.post(request, response, headers);

    webUpdate = false;
  }

  cycleCount++;
  delay(1000);

}

//===================================================================================================================
// Spark.functions (registered in Setup() above)
// Spark.functions always take a string as an argument and return an integer.
int WebRunFunction(String command) {

    if(command=="update") {
        webUpdate = true;
        return 1;
    }

    return -1;
}
