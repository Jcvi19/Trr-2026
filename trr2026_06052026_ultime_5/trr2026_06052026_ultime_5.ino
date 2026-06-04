#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "LD06forArduino.h"
#include <ArduinoJson.h>
#include <ESP32Servo.h>

/*--------------------------------------------------
        CONFIGURATION GENERALE
--------------------------------------------------*/

// Angle du cône que le robot analyse devant lui
#define NAV_SCAN_MIN 280
#define NAV_SCAN_MAX 80

// Distance de sécurité en millimètres
#define SAFE_DIST 600
#define EMERGENCY_STOP 300
#define SLOW_DIST 1000

// Nombre d'angles du LIDAR
#define ANGLES 360

// Nombre de mesures utilisées pour le filtrage
#define HISTORY 2

#define LED 5
#define HP 4

// Durée de fonctionnement en millisecondes (ici 30 secondes)
#define RUN_DURATION_MS 60000

int chosenDirection = 90;
int prevChosenDirection = 90;

/*--------------------------------------------------
        HARDWARE
--------------------------------------------------*/

// Servo direction
#define SERVO_PIN 25
Servo zoneServo;
float servoAngle = 90;
float servoFiltered = 90;

// Moteur IBT-2
int RPWM_Output = 18;
int LPWM_Output = 19;

const int pwmChannelR = 2;
const int pwmChannelL = 3;

int motorSpeed = 512; // 512 = arrêt

unsigned long startTime = 0;
bool robotStopped = false;

/*--------------------------------------------------
        LIDAR
--------------------------------------------------*/

LD06forArduino lidar;

/* Structure stockant les mesures pour chaque angle */
struct AngleData {
  int distances[HISTORY];
  int filtered;
  uint8_t writeIdx;
  bool isValid;
  unsigned long lastUpdate;  // ← AJOUTER
};

AngleData angleBuffer[ANGLES];


/*--------------------------------------------------
        WIFI
--------------------------------------------------*/

const char* ssid = "Robot_LIDAR";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);


/*--------------------------------------------------
        PAGE WEB (Radar + Carte)
--------------------------------------------------*/

const char PAGE_HTML[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>

<meta charset="utf-8">

<title>Radar LIDAR</title>

<style>

body{
background:#0a0a0a;
color:#00d4ff;
font-family:sans-serif;
text-align:center;
}

canvas{
background:#111;
border:2px solid #00d4ff;
border-radius:10px;
}

button{
margin-top:10px;
padding:8px;
}

</style>

</head>

<body>

<h2>Radar LIDAR + Carte</h2>

<button onclick="clearMap()">Effacer carte</button>

<br><br>

<canvas id="radar" width="600" height="600"></canvas>

<script>

/*----------------------------------
   CANVAS
----------------------------------*/

const canvas = document.getElementById("radar");
const ctx = canvas.getContext("2d");

let width = canvas.width;
let height = canvas.height;

let CX = width/2;
let CY = height/2;

let chosenDir = 90;


/*----------------------------------
   ZOOM + DRAG
----------------------------------*/

let scale = 0.06;

let offsetX = 0;
let offsetY = 0;

let dragging = false;
let lastX = 0;
let lastY = 0;

canvas.addEventListener("wheel",(e)=>{

 e.preventDefault();

 if(e.deltaY < 0)
  scale *= 1.1;
 else
  scale *= 0.9;

});

canvas.addEventListener("mousedown",(e)=>{

 dragging = true;

 lastX = e.clientX;
 lastY = e.clientY;

});

canvas.addEventListener("mouseup",()=>{

 dragging = false;

});

canvas.addEventListener("mousemove",(e)=>{

 if(!dragging) return;

 offsetX += e.clientX - lastX;
 offsetY += e.clientY - lastY;

 lastX = e.clientX;
 lastY = e.clientY;

});


/*----------------------------------
   DONNEES LIDAR
----------------------------------*/

let currentPoints = [];

let mapPoints = new Array(360).fill(0);


/*----------------------------------
   TRAJECTOIRE ROBOT
----------------------------------*/

let robotPath = [];

let robotX = 0;
let robotY = 0;


/*----------------------------------
   WEBSOCKET
----------------------------------*/

let ws = new WebSocket(`ws://${location.hostname}:81/`);

ws.onmessage = (e)=>{

 const data = JSON.parse(e.data);

 if(data.d){

   currentPoints = data.d;

   chosenDir = data.dir;   // <-- AJOUT

   for(let a=0;a<360;a++){

    let d = currentPoints[a];

    if(d > 0){

      mapPoints[a] = mapPoints[a]*0.1 + d*0.9;

    }

   }
    /* TRAJECTOIRE */
   robotPath.push({x:robotX,y:robotY});
   
   if(robotPath.length > 2000)
     robotPath.shift();
 }

};


/*----------------------------------
   RESET MAP
----------------------------------*/

function clearMap(){

 mapPoints.fill(0);
 robotPath = [];

}


/*----------------------------------
   COORDONNEES
----------------------------------*/

function worldToScreen(x,y){

 return {

   x: CX + offsetX + x * scale,
   y: CY + offsetY + y * scale

 };

}


/*----------------------------------
   DESSIN
----------------------------------*/

function draw(){

 ctx.fillStyle="rgba(10,10,10,0.25)";
 ctx.fillRect(0,0,width,height);


 /*-----------------------------
    GRILLE
 -----------------------------*/

 ctx.strokeStyle="#333";
 ctx.fillStyle="#00d4ff";
 ctx.font="10px Arial";

 for(let r=1000;r<=12000;r+=1000){

   let radius = r*scale;

   ctx.beginPath();
   ctx.arc(CX+offsetX,CY+offsetY,radius,0,Math.PI*2);
   ctx.stroke();

   ctx.fillText(r+" mm",CX+offsetX+5,CY+offsetY-radius);

 }


 /*-----------------------------
   ANGLES
 -----------------------------*/

 ctx.strokeStyle="#222";

 for(let a=0;a<360;a+=30){

   let rad=(a-90)*Math.PI/180;

   let x = CX + offsetX + Math.cos(rad)*800;
   let y = CY + offsetY + Math.sin(rad)*800;

   ctx.beginPath();
   ctx.moveTo(CX+offsetX,CY+offsetY);
   ctx.lineTo(x,y);
   ctx.stroke();

   ctx.fillText(a+"°",x,y);

 }

 /*-----------------------------
   ZONE DANGER
 -----------------------------*/
 ctx.strokeStyle="#ff4444";

 for(let r=100;r<=300;r+=100){

   let radius = r*scale;

   ctx.beginPath();
   ctx.arc(CX+offsetX,CY+offsetY,radius,0,Math.PI*2);
   ctx.stroke();

 }

 /*-----------------------------
   CARTE RELIEE
 -----------------------------*/

 ctx.strokeStyle="orange";
 ctx.lineWidth=1;

 ctx.beginPath();

 let first=true;

 for(let a=0;a<360;a++){

   let dist = mapPoints[a];

   if(dist>0){

     let rad=(a-90)*Math.PI/180;

     let x = dist*Math.cos(rad);
     let y = dist*Math.sin(rad);

     let p = worldToScreen(x,y);

     if(first){

       ctx.moveTo(p.x,p.y);
       first=false;

     }
     else{

       ctx.lineTo(p.x,p.y);

     }

   }

 }

 ctx.stroke();


 /*-----------------------------
   SCAN ACTUEL
 -----------------------------*/

 ctx.fillStyle="#00ff88";

 for(let a=0;a<360;a++){

   let dist=currentPoints[a];

   if(dist>0){

     let rad=(a-90)*Math.PI/180;

     let x=dist*Math.cos(rad);
     let y=dist*Math.sin(rad);

     let p = worldToScreen(x,y);

     ctx.beginPath();
     ctx.arc(p.x,p.y,2,0,Math.PI*2);
     ctx.fill();

   }

 }

/*-----------------------------
   DIRECTION CHOISIE
-----------------------------*/

ctx.strokeStyle="#00ff00";
ctx.lineWidth=3;

let rad = (chosenDir-90)*Math.PI/180;

let x = Math.cos(rad)*12000;
let y = Math.sin(rad)*12000;

let p = worldToScreen(x,y);

ctx.beginPath();
ctx.moveTo(CX+offsetX,CY+offsetY);
ctx.lineTo(p.x,p.y);
ctx.stroke();

 /*-----------------------------
   ROBOT
 -----------------------------*/

 let r = worldToScreen(robotX,robotY);
 ctx.fillStyle="red";
 ctx.fillRect(r.x-6,r.y-6,12,12);

 requestAnimationFrame(draw);

}

draw();

</script>

</body>
</html>

)rawliteral";


/*--------------------------------------------------
        FONCTION VERIFICATION CONE AVANT
--------------------------------------------------*/

bool inFrontCone(int a){

  if(NAV_SCAN_MIN < NAV_SCAN_MAX)
    return (a >= NAV_SCAN_MIN && a <= NAV_SCAN_MAX);

  else
    return (a >= NAV_SCAN_MIN || a <= NAV_SCAN_MAX);

}


/*--------------------------------------------------
        CONTROLE MOTEUR
--------------------------------------------------*/

void updateMotor(){

 if (motorSpeed < 500) {

   ledcWrite(pwmChannelR,0);
   ledcWrite(pwmChannelL,map(motorSpeed,500,0,0,1023));

 }
 else if (motorSpeed > 524) {

   ledcWrite(pwmChannelL,0);
   ledcWrite(pwmChannelR,map(motorSpeed,524,1023,0,1023));

 }
 else {

   ledcWrite(pwmChannelR,0);
   ledcWrite(pwmChannelL,0);

 }

}


/*--------------------------------------------------
        ALGORITHME NAVIGATION
--------------------------------------------------*/

void findBestPath(){

  int bestScore = 0;
  int bestAngle = 90;

  for(int a = 0; a < ANGLES; a++){

    if(!inFrontCone(a)) continue;
    if(!angleBuffer[a].isValid) continue;
    if(millis() - angleBuffer[a].lastUpdate > 500) continue;

    /* ── Moyenne des distances sur ±10° ── */
    int sum = 0;
    int count = 0;
    for(int k = -15; k <= 15; k++){
      int aa = (a + k + 360) % 360;
      if(angleBuffer[aa].isValid){
        sum += angleBuffer[aa].filtered;
        count++;
      }
    }
    if(count == 0) continue;
    int avgDist = sum / count;

    if(avgDist < SAFE_DIST) continue;
/*--------------------------------------------*/
    /* ── Vérification gabarit robot (demi-largeur 150mm + 6° marge) ── */
    int halfAngle = (int)(asin(150.0f / max(avgDist, 151))) + 6;

    bool passageClear = true;
    for(int k = -halfAngle; k <= halfAngle; k++){
      int aa = (a + k + 360) % 360;
      if(!angleBuffer[aa].isValid || angleBuffer[aa].filtered < SAFE_DIST){
        passageClear = false;
        break;
      }
    }
    if(!passageClear) continue;   // couloir trop étroit → on saute

    /* ── Score ── */
    int width = halfAngle * 2;
/*--------------------------------------------*/
int angleDiff = abs(a - chosenDirection);
if(angleDiff > 180) angleDiff = 360 - angleDiff;
int score = avgDist + width * 200 - angleDiff * 8;

if(score > bestScore + 150){
  bestScore = score;
  bestAngle = a;
}
  }

  chosenDirection = bestAngle;

  /* conversion angle -> servo (inchangée) */
  if(bestAngle >= NAV_SCAN_MIN){
    servoAngle = map(bestAngle, 330, 359, 65, 90);}
  else if (bestAngle <= NAV_SCAN_MAX){
    servoAngle = map(bestAngle, 0, 30, 90, 115);}
  else {servoAngle = 90;}
}

/*--------------------------------------------------
        VERIFICATION LIDAR PRET
--------------------------------------------------*/

bool lidarReady(){
  int validCount = 0;
  for(int i = 0; i < ANGLES; i++){
    if(angleBuffer[i].isValid) validCount++;
  }
  return validCount >= 360;
}

/*--------------------------------------------------
        TASK NAVIGATION
--------------------------------------------------*/

void navigationTask(void * p){

  static int lastServo = 90;

  // ── ATTENTE LIDAR ──
  motorSpeed = 512;
  updateMotor();
  while(!lidarReady()){
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  // Petit délai de sécurité supplémentaire
  vTaskDelay(500 / portTICK_PERIOD_MS);


  // Bip de démarrage : 3 bips courts
  for(int i = 0; i < 2; i++){
    digitalWrite(HP, HIGH);
    digitalWrite(LED, LOW);
    delay(200);
    digitalWrite(HP, LOW);
    digitalWrite(LED, HIGH);
    delay(200);
  }

  startTime = millis();

  for(;;){

    // ── ARRÊT TEMPORISÉ ──
    if(!robotStopped && millis() - startTime >= RUN_DURATION_MS){
      robotStopped = true;
      motorSpeed = 512;
      updateMotor();
      zoneServo.write(90);
      digitalWrite(LED, LOW);
    // La tâche continue de tourner mais ne fait plus rien
    }

    if(robotStopped){
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

findBestPath();

  // ── DELTA DIRECTION ──
    int delta = chosenDirection - prevChosenDirection;
    if(delta > 180)  delta -= 360;
    if(delta < -180) delta += 360;
    prevChosenDirection = chosenDirection;

    int frontDist = 9999;
    for(int k = -19; k <= 19; k++){ //19 ok, 17 touche le bord
      int aa = (chosenDirection + k + 360) % 360;
      if(angleBuffer[aa].isValid && angleBuffer[aa].filtered < frontDist)
        frontDist = angleBuffer[aa].filtered;
    }

    if(frontDist < EMERGENCY_STOP){
      motorSpeed = 512;
    }
    else if(frontDist < SLOW_DIST){
      motorSpeed = 780 - (abs(servoAngle - 90)/2);
    }
    else if(frontDist < 1.6*SLOW_DIST ){
      motorSpeed = 840 - abs(servoAngle - 90);
    }
    else {
      motorSpeed = 850 - 2*abs(servoAngle - 90);
    }
    // ── FREINAGE ANTICIPATOIRE ── ← ICI, juste après
    int steeringMagnitude = abs((int)servoFiltered - 90);
    int turnRate = abs(delta);

    if(turnRate > 10){
      motorSpeed -= turnRate * 1;
      motorSpeed = max(motorSpeed, 512);
    }


    /* filtrage servo adaptatif à la vitesse */
    float alpha = (motorSpeed > 600) ? 0.15f : 0.35f;
    servoFiltered = servoFiltered * (1.0f - alpha) + servoAngle * alpha;

    /* zone morte : ne bouger que si écart > 3° */
    if(abs((int)servoFiltered - lastServo) > 2){
      zoneServo.write((int)servoFiltered);
      lastServo = (int)servoFiltered;
    }

    updateMotor();

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


/*--------------------------------------------------
        SETUP
--------------------------------------------------*/

void setup(){
  
 /* moteur */

 ledcSetup(pwmChannelR,20000,10);
 ledcSetup(pwmChannelL,20000,10);

 ledcAttachPin(RPWM_Output,pwmChannelR);
 ledcAttachPin(LPWM_Output,pwmChannelL);

 Serial.begin(115200);

 pinMode(LED, OUTPUT);
 pinMode(HP, OUTPUT);

 WiFi.softAP(ssid,password);

 /* servo */

 ESP32PWM::allocateTimer(0);
 zoneServo.setPeriodHertz(50);
 zoneServo.attach(SERVO_PIN,500,2400);

 /* lidar */

 lidar.Init(16);

 /* serveur web */

 server.on("/",[](){
   server.send_P(200,"text/html",PAGE_HTML);
 });

 server.begin();

 webSocket.begin();

  digitalWrite(LED, HIGH);

 // Bip de démarrage : 10 bips courts
for(int i = 0; i < 10; i++){
  digitalWrite(HP, HIGH);
  digitalWrite(LED, LOW);
  delay(50);
  digitalWrite(HP, LOW);
  digitalWrite(LED, HIGH);
  delay(100);
}

  //startTime = millis();

 /* navigation sur core 1 */

 xTaskCreatePinnedToCore(
   navigationTask,
   "nav",
   8192,
   NULL,
   1,
   NULL,
   1
 );

}


/*--------------------------------------------------
        LOOP PRINCIPAL
--------------------------------------------------*/

void loop(){

 lidar.read_lidar_data();

 server.handleClient();
 webSocket.loop();

 /* stockage mesures lidar */

 for(size_t i=0;i<lidar.angles.size();i++){

   if(lidar.confidences[i] < 150) continue;

   int a=((int)(lidar.angles[i]+0.5f)+90)%360;

   AngleData &d = angleBuffer[a];

   d.distances[d.writeIdx] = lidar.distances[i];
   d.writeIdx = (d.writeIdx + 1) % HISTORY;

   int sum = 0;
   for(int j=0; j<HISTORY; j++) sum += d.distances[j];
   d.filtered = sum / HISTORY;   // ← stocké dans son propre champ
   d.isValid = true;
   d.lastUpdate = millis();  
 }
 
 /* envoi radar vers navigateur */

 static unsigned long lastWS = 0;

 if(millis() - lastWS > 100){

   lastWS = millis();

   StaticJsonDocument<4096> doc;

   JsonArray arr = doc.createNestedArray("d");

   for(int i=0;i<360;i++)
     arr.add(angleBuffer[i].filtered);

    doc["dir"] = chosenDirection;   // <-- AJOUT

   String out;
   serializeJson(doc,out);

   if(webSocket.connectedClients() > 0){
    webSocket.broadcastTXT(out);
   }

 }

}