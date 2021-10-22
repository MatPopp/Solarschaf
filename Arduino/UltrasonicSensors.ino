

int triggerU1=12;                               // Der Trigger Pin
int echoU1=10;                                  // Der Echo Pin

int triggerU2=9;                               // Der Trigger Pin
int echoU2=7;                                  // Der Echo Pin

int triggerU3=6;                               // Der Trigger Pin
int echoU3=4;                                  // Der Echo Pin

int triggerU4=3;                               // Der Trigger Pin
int echoU4=2;                                  // Der Echo Pin

long dauer=0;                                // Hier wird die Zeitdauer abgespeichert
// die die Ultraschallwelle braucht

                                             // um zum Sensor zurückzukommen
long entfernungU1=0;                           // Hier wird die Entfernung vom 
                                             // Hindernis abgespeichert
long entfernungU2=0;                           // Hier wird die Entfernung vom 
                                             // Hindernis abgespeichert
long entfernungU3=0;                           // Hier wird die Entfernung vom 
                                             // Hindernis abgespeichert
long entfernungU4=0;                           // Hier wird die Entfernung vom 
                                             // Hindernis abgespeichert

 

void setup()
{
    Serial.begin(9600);                      // Die serielle Kommunikation starten
    pinMode(triggerU1, OUTPUT);                // Trigger Pin als Ausgang definieren
    pinMode(echoU1, INPUT);                    // Echo Pin als Eingang defnieren
    pinMode(triggerU2, OUTPUT);                // Trigger Pin als Ausgang definieren
    pinMode(echoU2, INPUT);                    // Echo Pin als Eingang defnieren
    pinMode(triggerU3, OUTPUT);                // Trigger Pin als Ausgang definieren
    pinMode(echoU3, INPUT);                    // Echo Pin als Eingang defnieren
    pinMode(triggerU4, OUTPUT);                // Trigger Pin als Ausgang definieren
    pinMode(echoU4, INPUT);                    // Echo Pin als Eingang defnieren
}

long detectDistance(int trigger,int echo) {
  digitalWrite(trigger, LOW);              // Den Trigger auf LOW setzen um
                                             // ein rauschfreies Signal
                                             // senden zu können
  delay(5);                                // 5 Millisekunden warten
  digitalWrite(trigger, HIGH);             // Den Trigger auf HIGH setzen um eine 
                                           // Ultraschallwelle zu senden
  delay(10);                               // 10 Millisekunden warten
  digitalWrite(trigger, LOW);              // Trigger auf LOW setzen um das 
                                           // Senden abzuschließen
  dauer = pulseIn(echo, HIGH);             // Die Zeit messen bis die 
                                           // Ultraschallwelle zurückkommt
  long entfernung = (dauer/2) / 29.1;           // Die Zeit in den Weg in Zentimeter umrechnen 
  if(entfernung>=800){
    entfernung=-1;
  }
  return(entfernung);
}

void loop()
{
    entfernungU1 = detectDistance(triggerU1,echoU1);
    entfernungU2 = detectDistance(triggerU2,echoU2);
    entfernungU3 = detectDistance(triggerU3,echoU3);
    entfernungU4 = detectDistance(triggerU4,echoU4);

    Serial.print("U1:");            // Den Weg in Zentimeter ausgeben
    Serial.print(entfernungU1);
    Serial.print(" U2:");
    Serial.print(entfernungU2);
    Serial.print(" U3:");
    Serial.print(entfernungU3);
    Serial.print(" U4:");
    Serial.println(entfernungU4);

    delay(1000);                             // Nach einer Sekunde wiederholen
}
