#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

const String PHONE = "+381677262976"; //string, broj telefona

#define rxPin 3	//RX pin GSM modula ka Arduinu
#define txPin 4	//TX pin GSM modula ka Arduinu
SoftwareSerial SIM900(rxPin,txPin);

AltSoftSerial neogps;		//RX i TX pinovi za GPS modul su predefinisani na 9 i 8
TinyGPSPlus gps;
#define BUZZER 4

// Promenljive kojim pratimo vreme nakon slanja poruka
// Pratimo vreme nakon poslednjeg pokretanja
boolean multiple_sms = false;
unsigned long previousMillis=0;
unsigned long int previoussecs = 0; 
unsigned long int currentsecs = 0; 
 unsigned long currentMillis = 0; 
 int secs = 0; 
 int pmints = 0; 
 int mints = 0; // current mints
 int interval= 1 ; //azurira se na svaku sekundu

// Promenljiva koja definise distancu(precnik) od ref. tacke u metrima
const float maxDistance = 20;

//Definisemo parametre referentne tacke
float initialLatitude = 45.24611596416754;
float initialLongitude = 19.85169687573599;

//Definisemo promenjljive za GPS koordinate u koje ce biti smesteni podaci sa GPS modula
float latitude, longitude;

//Funkcija za dobijanje GPS podataka sa GPS modula
void getGps(float& latitude, float& longitude);



void setup()
{
  Serial.begin(9600);		//Uspostava serijske komunikacije Arduina
  SIM900.begin(9600);	//Uspostava serijske komunikacije sa GPRS/GSM modulom
  neogps.begin(9600);		//Uspostava serijske komunikacije sa GPS modulom

//Podesavamo nacin rada GSM modula
  SIM900.println("AT"); 	//Provera GSM modula
  delay(1000);		//Kasnjenje kako bi GSM modul mogao efikasno raditi
  SIM900.println("ATE1"); 	//Paljenje eho moda
  delay(1000);
  SIM900.println("AT+CPIN?"); 	//Provera da li je SIM spreman
  delay(1000);
  SIM900.println("AT+CMGF=1"); 	//Mode SMS poruka
  delay(1000);
  SIM900.println("AT+CNMI=1,1,0,0,0"); //Odluka kako ce se rukovati pristiglim SMS porukama
  delay(1000);
  //AT +CNMI = 2,1,0,0,0 - AT +CNMI = 2,2,0,0,0 (both are same)
  //--------------------------------------------------------------
  delay(20000);
  buzzer_timer = millis();
}


void loop()
{
  //--------------------------------------------------------------
  getGps(latitude, longitude);		//Trazimo GPS podatke

//Trazimo distancu 
  float distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);

//Stampamo na serijsko monitoru podatke
  Serial.print("Geo. sirina= "); Serial.println(latitude, 6);
  Serial.print("Geo. duzina= "); Serial.println(longitude, 6);
  Serial.print("Referentna geo. sirina= "); Serial.println(initialLatitude, 6);
  Serial.print("Referentna geo. duzina= "); Serial.println(initialLongitude, 6);
  Serial.print("Trenutna udaljenost= "); Serial.println(distance);

//Provera da li je trenutna udaljenost veca od maksimalne
  if(distance > maxDistance) {
//Ako da: saljemo obavestenje
    multiple_sms = true;
    if(send_alert_once == true){
      sendAlert();
      alarm = true;
      send_alert_once = false;
    }
  }
  else{
    send_alert_once = true;
    multiple_sms = false;
  }

//Ako se poruka posalje, pratimo vreme od slanja poruke
  if ( multiple_sms = true)
  {
       currentMillis = millis();
       currentsecs = currentMillis / 1000; 
       if ((unsigned long)(currentsecs - previoussecs) >= interval) {
       secs = secs + 1; 

//Ako prodje 20 sekundi, ponavljamo slanje obavestenja
       if ( secs >= 20)
       {
        sendAlert();
        multiple_sms = false;
        secs = 0;
       }
    }
  }

  while(SIM900.available()){
    Serial.println(SIM900.readString());
  }

  while(Serial.available())  {
    SIM900.println(Serial.readString());
  }
}

//Racunanje udaljenosti izmedju dve tacke
float getDistance(float flat1, float flon1, float flat2, float flon2) {

  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
  //Formula
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}

//Funkcija za dobijanje koordinata od GPS modula
void getGps(float& latitude, float& longitude)
{
  //OVO MOZE TRAJATI I DO 60 SEKUNDI
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;){
    while (neogps.available()){
      if (gps.encode(neogps.read())){
        newData = true;
        break;
      }
    }
  }
  
  if (newData)		//Ako smo dobili nove podatke
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    newData = false;
  }
  else {
    Serial.println("Nisu dostupni novi GPS podaci");
    latitude = 0;
    longitude = 0;
  }
}

//Slanje SMS obavestenja korisniku putem GPRS/GSM modula
void sendAlert()
{
  String sms_data;
  sms_data = "UPOZORENJE! Automobil je van zone pokrivenosti.\r";
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += String(latitude) + "," + String(longitude);

  SIM900.print("AT+CMGF=1\r");
  delay(1000);
  SIM900.print("AT+CMGS=\""+PHONE+"\"\r");
  delay(1000);
  SIM900.print(sms_data);
  delay(100);
  SIM900.write(0x1A);
  delay(1000);
  Serial.println("SMS obavestenje uspesno poslato.");
}
