#include "restAPI.h"


AsyncWebServer server(80);

const char* PARAM_MESSAGE = "message";

void notFound(AsyncWebServerRequest *request) {
      Serial.println("not found");
    request->send(404, "text/plain", "Not found");
}

// "smoker/temp";     // used for publishing temp and setpoint
// "smoker/setpoint";  //set goal temp
// "smoker/pid";       // set PID coefficients
// "smoker/servo";   // direct control of servo command 0-255
// "smoker/valve";   // direct control of valve pos off/min/max
// "smoker/inj";       // inject temp value for testing
// "smoker/control";  // control loop on/off

void setupServerEvents() {

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      Serial.println("On");

        request->send(200, "text/plain", "Hello, world");
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        Serial.println("get");
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        Serial.println("post");
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });





    server.onNotFound(notFound);

    Serial.println("server before begin");
    server.begin();
    Serial.println("server begins");
}


