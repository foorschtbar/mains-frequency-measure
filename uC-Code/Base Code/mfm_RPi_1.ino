/*
  lauffähiges Programm Netzfrequenzmessung Serial-Output RPi-Board oder USB-TTL-Konverter
  OpenSource Basisversion,  03'2020 J. Müller
  Software unter GNU GPL3-Lizenz
  freie Verwendung für kommerzielle und private Projekte unter gleichen Bedingungen
  Programming languange: Arduino IDE (1.8.12)
  www.netzfrequenzanzeige.de  
  www.pc-projekte.de
  Version mit Differenz-Filter
*/

/*
 * Wichtig ! Die µControler-Fuses müssen korrekt eingestellt sein!
   Low lfuse = 0xFF; (B11111111)
   High fuse = 0xDA; (B11011110)
   Extended fuse = 0xFF; (B00000101)
 */

// Settings / Einstellungen*****************************************************************************
#define F_CPU 10000000
const word          Prescaler = 64;        // Prescaler (0,8,64,256,1024)    Prescaler-Auswahl (0,8,64,256,1024)
const byte          Messperiode = 18;      // Period                         Periodendurchläufe
const unsigned long Dividend = 2812500;    // DIVIDEND Calculation constant  Dividend Berechnungskonstante
const word          Sperr = 2400;          // Lock-Counts                    Vorgabe Sperrcounter
const int           UBRR = 0x40;           // Correction serial baud/clock   Anpassung Baud an Systemtakt
const int           DiffMAX = 60;          // DifferenzMAX                   maximal zulässiger Differenzwert zwischen den Messperioden
byte                FA = 0;                // Selection 0 = Auto, 50=50Hz, 60=60Hz     
const short         Calibration = 0;       // Korrekturwert                  Kalibrierung,Angaben in mHz.

// *****************************************************************************************************


word Frequenz;                              // mains frequency as WORD (mHz)  Netzfrequenz-Messwert als WORD in mHz
word fMess;                                 // preliminary measurement        vorläufiger Messwert
//double f;
//int Diff;                                 // Difference measurement         Differenzmesswert
word Triggersperr = Sperr;                  // Lockout-Counter (presetting)   Trigger-Sperrcounter mit Vorgabe


bool Auswertung = false;                    // Evaluation                     starte Auswertung aus Rohdaten
bool Check = false;                         // Measure Evaluation Check      Ergebniss 1.Prüfung

byte Periode = 0;                           // Period Counter                 Perioden-Durchlaufzähler
word Count = 0;                             // Raw Data Buffer                Zwischenspeicher Messwerte (Rohdaten)

const byte LED = 2;                         // Output LED RUN                 Ausgabe LED RUN
bool Toggle;                                // toggle auxiliary bit           Toggle-Hilfsbit

byte Ueberlauf = 0;                         // Overflow Counter               Überlaufs-Zähler

#include<avr/wdt.h>                         // Watchdog Vorbereitung...


// // '********** Einstellungen ****************

int Cachewert ;//As Long                                     'Ungefilterter Wert vom Sensor

//******************************************************

// // '********** Einstellungen ****************

int Sensor_wert ;//As Long                                     'Ungefilterter Wert vom Sensor 


//******************************************************




void setup()
{
  noInterrupts();                           // Alle Interrupts temporär abschalten

  // serial init
  Serial.begin(19200, SERIAL_8N1);
  UBRR0 = UBRR;                             // Korrekturwert/Anpassung an den 10MHz-Quarz

  // RUN-LED init
  pinMode(LED, OUTPUT);                     // LED Ausgabe vorbereiten

  //init Interrupt input 
  const byte interruptPin = 3;
  attachInterrupt(digitalPinToInterrupt(interruptPin), Mess, FALLING );

  //it inTimer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;                // Timer nach obiger Rechnung vorbelegen
  if (Prescaler == 1024) TCCR1B |= (1 << CS12) | (1 << CS10);
  if (Prescaler == 256)  TCCR1B |= (1 << CS12);
  if (Prescaler == 64)   TCCR1B |= (0 << CS12) | (1 << CS11) | (1 >> CS10);
  if (Prescaler == 8)    TCCR1B |= (1 << CS11);
  if (Prescaler == 0)    TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << TOIE1);   // Timer Overflow Interrupt aktivieren

  interrupts();             // alle Interrupts scharf schalten

  wdt_enable(WDTO_4S);      //watchdog

  Serial.println("NFA-RPi 1.0 Basisprogramm (@Arduino)");
  Serial.println("05'2020 J.Müller www.pc-projekte.de");
  Auswertung = false;
  

  if ( !(FA == 50) && !(FA == 60 ))
  {    
  Serial.print("Auto-Detect...");
  delay(1000);

  while (Auswertung == false)
  {
  };
  
  if (Auswertung == true)
  {
    Auswertung = false;    
    //Check1 Messdifferrenzüberprüfung      
    fMess = (Dividend * 1000) / Count;   // Berechne vorläufige Netzfrequenz
    
    if ((fMess > 45000) && (fMess < 55000))  // Messwertauswahl 50Hz
    {
      Cachewert = fMess;
      FA = 50;
      Serial.println("  50Hz-Netz    ");
      delay(1000); 
    };
    
     if ((fMess > 55000) && (fMess < 65000))     // Messwertauswahl 60Hz
     {
      FA = 60;
      Serial.println("  60Hz-Netz    ");
      delay(1000);
     } 
    }
  } 
  //FA = 50;
  Serial.println("Start Netzfrequenzmessung...");
  Serial.write(7);
}


void loop()
{

  if ( Auswertung == true)
  {
    Auswertung = false;
    fMess = (Dividend * 1000) / Count;           // Berechne vorläufige Netzfrequenz   ... X 1000 = mHz
  //  f =   (double)fMess / 1000;

    // Prüfung: Messwert im Gültigkeitsbereich
    Check = false;
    if ( (fMess > 45000) && (fMess < 55000) && (FA == 50)) Check = true;
    if ( (fMess > 55000) && (fMess < 65000) && (FA == 60)) Check = true;

    // Wenn der Check ok, dann weiter zum Differenzfilter / Ausgabe
    if ( Check == true)
    {
      Frequenz = Diff_filter(fMess);
      Toggle = !Toggle;                       // toggle ausführen  LED-Ausgabe
      if (Toggle == true) digitalWrite(LED, HIGH); else digitalWrite(LED, LOW);
      wdt_reset();
    }
    
    Serial.println(Frequenz + Calibration ); // Ausgabe des gemessenen Netzfrequenzwertes + Kalibrierungswert

  };
  delay(50);
}

word Diff_filter(word filter_wert)
{

  int Diff = filter_wert - Cachewert;

  // bei Überschreitung DiffMAX    
  if ( abs(Diff) > DiffMAX )
  {
    if ( Diff > 0) filter_wert = Cachewert + DiffMAX;  // Änderung (positive Steigung) max 
    if ( Diff < 0) filter_wert = Cachewert - DiffMAX;  // Änderung (negative Steigung) max 
  }
  
  Cachewert = (int)filter_wert;
  return filter_wert;
}



ISR(TIMER1_OVF_vect) 
{
  Ueberlauf ++;
  if (Ueberlauf >= 10)
  {
    Serial.println("XXXXX");
    Ueberlauf = 10;
  }
}



void Mess()
{
  // Messroutine
  if (Triggersperr < TCNT1)              // 1. Überprüfung: Sperrzeit eingehalten ?
  {
    Triggersperr = TCNT1 + Sperr;        // neue Sperrzeit
    Periode++;                           // Periodenzähler

    // Auswertung nach letzer Periode
    if (Periode >= Messperiode)
    {
      Count =  TCNT1;                   // Übernehme Counterstand Timer1
      TCNT1 = 0;                        // lösche Wert Timer1
      Periode = 0;                      // setze Periodenzähler zurück
      Auswertung = true;                // Auswertung aktivieren
      Triggersperr = Sperr;             // Rücksetzen der Sperrzeit (1.Periode)
    }
  }
}
