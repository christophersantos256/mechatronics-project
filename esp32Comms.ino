#include <esp_wifi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

const char *ssid = "MAE361 Group 11";
const char *password = "12345678";
const char *msg_up_button = "upButton";
const char *msg_get_upButton = "getUpButtonState";
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 4444;

// Motor control pins for ESP32
const int IN1 = 32, IN2 = 33, ENA = 25;
const int IN3 = 26, IN4 = 27, ENB = 14;
const int IN5 = 12, IN6 = 13, ENC = 23;
const int IN7 = 22, IN8 = 21, END = 18;

bool nwPressed = false;
bool northPressed = false;
bool nePressed = false;
bool westPressed = false;
bool eastPressed = false;
bool swPressed = false;
bool southPressed = false;
bool sePressed = false;

bool steerLeftPressed = false;
bool steerRightPressed = false;

AsyncWebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(4444);
char msg_buf[10];
int upButtonState = 0;

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length){
  switch(type){
    case WStype_DISCONNECTED:
    Serial.printf("Disconnected", client_num);
    break;

    case WStype_CONNECTED:
    {
      IPAddress ip = webSocket.remoteIP(client_num);
      Serial.printf("Connected ", client_num);
      Serial.println(ip.toString());
    }
    break;

    case WStype_TEXT:
    //Serial.printf("[%u] Received text: %s\n", client_num, payload);

//---------------------------START OF CONTROLS-------------------

  //NORTH CONTROLS

    if(strcmp((char *)payload, "upButtonPressed")==0){
      northPressed = true;
      //Serial.println(northPressed);
    }
    else if(strcmp((char *)payload, "upButtonReleased")==0){
      northPressed = false;
      //Serial.println(northPressed);
    }

  //NORTH WEST CONTROLS

    else if (strcmp((char *)payload, "leftButtonPressed")==0){
      nwPressed = true;
    }
    else if(strcmp((char *)payload, "leftButtonReleased")==0){
      nwPressed = false;
    }

  //NORTH EAST CONTROLS

    else if(strcmp((char *)payload, "rightButtonPressed")==0){
      nePressed = true;
    }
    else if(strcmp((char *)payload, "rightButtonReleased")==0){
      nePressed = false;
    }

  //WEST CONTROLS

    else if(strcmp((char *)payload, "leftButton2Pressed")==0){
      westPressed = true;
    }
    else if(strcmp((char *)payload, "leftButton2Released")==0){
      westPressed = false;
    }

  //CENTER BUTTON CONTROLS (redundant)

    else if(strcmp((char *)payload, "centerButtonPressed")==0){
      Serial.write(1);
    }
    else if(strcmp((char *)payload, "centerButtonReleased")==0){
      Serial.write(0);
    }

  //EAST CONTROLS

    else if(strcmp((char *)payload, "rightButton2Pressed")==0){
      eastPressed = true;
    }
    else if(strcmp((char *)payload, "rightButton2Released")==0){
      eastPressed = false;
    }

  //SOUTHWEST

    else if(strcmp((char *)payload, "leftButton3Pressed")==0){
      swPressed = true;
    }
    else if(strcmp((char *)payload, "leftButton3Released")==0){
      swPressed = false;
    }

  //SOUTH

    else if(strcmp((char *)payload, "backButtonPressed")==0){
      southPressed = true;
    }
    else if(strcmp((char *)payload, "backButtonReleased")==0){
      southPressed = false;
    }

  //SOUTHEAST

    else if(strcmp((char *)payload, "rightButton3Pressed")==0){
      sePressed = true;
    }
    else if(strcmp((char *)payload, "rightButton3Released")==0){
      sePressed = false;
    }

  //STEER LEFT

    else if(strcmp((char *)payload, "steerLeftButtonPressed")==0){
      steerLeftPressed = true;
    }
    else if(strcmp((char *)payload, "steerLeftButtonReleased")==0){
      steerLeftPressed = false;
    }

  //STEER RIGHT

    else if(strcmp((char *)payload, "steerRightButtonPressed")==0){
      steerRightPressed = true;
    }
    else if(strcmp((char *)payload, "steerRightButtonReleased")==0){
      steerRightPressed = false;
    }

    //----------------------END OF CONTROLS--------------------------------

    else{
      Serial.println("Message not recognized");
    }
    break;

    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
    break;
  }
}

void onIndexRequest(AsyncWebServerRequest *request){
  IPAddress remote_ip = request ->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

void onCSSRequest(AsyncWebServerRequest *request){ //Callback
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/style.css", "test/css");
}

void onPageNotFound(AsyncWebServerRequest *request){
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void setup(){
  Serial.begin(115200);
  delay(100);
  if (!SPIFFS.begin()){
   Serial.println("Error mounting webfile");
    while(1);
  }

  WiFi.softAP(ssid,password);

  Serial.println();
  Serial.println("AP running");
  Serial.println("My IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, onIndexRequest);
  server.on("/style.css", HTTP_GET, onCSSRequest);
  server.onNotFound(onPageNotFound);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Initialize motor control pins as OUTPUT
  int pins[] = {IN1, IN2, ENA, IN3, IN4, ENB, IN5, IN6, ENC, IN7, IN8, END};
  for (int i = 0; i < 12; i++) {
  pinMode(pins[i], OUTPUT);  //Set as Output

  }

}

void loop(){
webSocket.loop(); //RUN WEBPAGE

//MOVEMENT LOGIC

if(northPressed){ //FWD
   movement(1, 1, 1, 1, 250);
}
else if(nwPressed){ //NORTHWEST
  movement(0, 1, 1, 0, 250);
}
else if(nePressed){ //NORTHEAST
  movement(1, 0, 0, 1, 250); 
}
else if(westPressed){
  movement(-1, 1, 1, -1, 250);
}
else if(eastPressed){
  movement(1, -1, -1, 1, 250);
}
else if(swPressed){
  movement(-1, 0, 0, -1, 250);
}
else if(southPressed){
  movement(-1, -1, -1, -1, 250);
}
else if(sePressed){
  movement(0, -1, -1, 0, 250);
}
else if(steerLeftPressed){
  movement(-1, -1, 1, 1, 125);
}
else if(steerRightPressed){
  movement(1, 1 , -1, -1, 125);
}
else{
  movement(0,0,0,0,0);
}

}

void movement(int m1, int m2, int m3, int m4, int SPEED) {
  MOTOR(IN1, IN2, ENA, m1, SPEED);  // Control Motor 1
  MOTOR(IN3, IN4, ENB, m2, SPEED);  // Control Motor 2
  MOTOR(IN5, IN6, ENC, m3, SPEED);  // Control Motor 3
  MOTOR(IN7, IN8, END, m4, SPEED+50);  // Control Motor 4
}

void MOTOR(int INA, int INB, int EN, int direction, int SPEED) {
  if (direction == 1) {  // Forward
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (direction == -1) {  // Backward
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
   } else if (direction==0){
   digitalWrite(INA, LOW);
   digitalWrite(INB, LOW);
  } else {  // Stop
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
  analogWrite(EN, SPEED);  // Control speed using PWM
}

