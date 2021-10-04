

/*
  lauffähiges Programm Netzfrequenzmessung Serial-Output RPi-Board oder USB-TTL-Konverter
  OpenSource Basisversion,  10'2020 J. Müller
  Software unter GNU GPL3-Lizenz
  freie Verwendung für kommerzielle und private Projekte unter gleichen Bedingungen
  Programming languange: Arduino IDE (1.8.12)
  www.netzfrequenzanzeige.de  
  www.pc-projekte.de
  Version mit Differenz-Filter
  erweiterte Version A- und B-Messung
  DeltaFilter
  Version mit Protokollauswahl
  Lastdifferenzanzeige
  bereinigte autom. Frequenzerkennung Start
  Protokoll 1,2,3,4
  Binärausgabe Low/HighAktiv
  Blinkfunktion
  erste finale Version für Dauertest mit Einstellungen
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
const word          Sperr = 2000;          // Lock-CountAs                    Vorgabe SperrCount
const int           UBRR = 0x40;           // Correction serial baud/clock   Anpassung Baud an Systemtakt
const int           DiffMAX = 100;          // DifferenzMAX                   maximal zulässiger Differenzwert zwischen den Messperioden
byte                FA = 0;                // Selection 0 = Auto, 50=50Hz, 60=60Hz     
const short         Calibration = 0;       // Korrekturwert                  Kalibrierung,Angaben in mHz.

// *****************************************************************************************************

// ********** Settings /Einstellungen ****************

//mains load differenz / Netzlastdifferenz
byte  Typ = 1;
float Lamda = 15.33333333;
float Offset = 10;
bool Neg = false;

// Alert mains frequency / Frequenzalarme
word f1g = 50070;       // fmax Alert
word f1k = 49930;       // fmin Alter
word tf1 = 10;          // Alert RelapseTime in s

word f2g = 50100;       // fmax Alert
word f2k = 49900;       // fmin Alter
word tf2 = 10;          // Alert RelapseTime in s

word f3g = 50200;       // fmax Alert
word f3k = 49800;       // fmin Alter
word tf3 = 10;          // Alert RelapseTime in s

word f4g = 51000;       // fmax Alert
word f4k = 49000;       // fmin Alter
word tf4 = 10;          // Alert RelapseTime in s


//Alert Output / Alarmausgaben
bool LowAktiv = true;   // Output Alarmausgabe LowAktiv
bool BlinkON = true;    // Blinkende Ausgabe z. B. LEDs


// ********** Settings /Einstellungen ****************


// *****************************************************************************************************


word FrequenzA;                             // mains frequency as WORD (mHz)  Netzfrequenz-Messwert als WORD in mHz A-Messung
word FrequenzB;                             // mains frequency as WORD (mHz)  Netzfrequenz-Messwert als WORD in mHz B-Messung
word Frequenz = 0;                      // mains frequency as WORD (mHz)  Netzfrequenz-Messwert als WORD in mHz
word MessA;                                 // preliminary measurement        vorläufiger Messwert
word MessB;                                 // preliminary measurement        vorläufiger Messwert
//double f;
//int Diff;                                 // Difference measurement         Differenzmesswert
word TriggersperrA = Sperr;                 // Lockout-Counter (presetting)   Trigger-SperrCount mit Vorgabe
word TriggersperrB = Sperr;                 // Lockout-Counter (presetting)   Trigger-SperrCount mit Vorgabe

bool AuswertungA = false;                   // Evaluation                     starte Auswertung aus Rohdaten AMessung
bool AuswertungB = false;                   // Evaluation                     starte Auswertung aus Rohdaten BMessung
bool CheckA = false;                        // Measure Evaluation CheckA      Ergebniss 1.Prüfung AMessung
bool CheckB = false;                        // Measure Evaluation CheckA      Ergebniss 1.Prüfung BMessung

byte PeriodeA = 0;                          // Period Counter                 Perioden-Durchlaufzähler
byte PeriodeB = 0;                          // Period Counter                 Perioden-Durchlaufzähler
word CountA = 0;                            // Raw Data Buffer                Zwischenspeicher Messwerte AMessung (Rohdaten)
word CountB = 0;                            // Raw Data Buffer                Zwischenspeicher Messwerte BMessung (Rohdaten)
word T2Counter = 0;
word BTriggerCounter = 0;

const byte LED = 4;                         // Output LED RUN                 Ausgabe LED RUN
bool Toggle;                                // toggle auxiliary bit           Toggle-Hilfsbit

byte Ueberlauf = 0;                         // Overflow Counter               Überlaufs-Zähler
byte ResetCounter = 0;
bool Reset_OK = false;


#include<avr/wdt.h>                         // Watchdog Vorbereitung...


// '********** Hilfsvariablen  ****************// 

//Hilfsvariablen int Cachewert ;//As Long                                     ' 
int CacheA = 50000;                                //  Hilfsspeicher für Differenzfilter A-Messung
int CacheB = 50000;                                //  Hilfsspeicher für Differenzfilter B-Messung
bool DeltaA = false;                                //  Hilfsvariable Delta A-Messung erkannt
bool DeltaB = false;                                //  Hilfsvariable Delta B-Messung erkannt

//Binäreingänge Protokolleinstellungen
const byte Jumper51 =  A3;                  // Pin26 A3
const byte Jumper52 =  A2;                  // Pin25 A2
const byte Jumper53 =  A1;                  // Pin24 A1
byte Protokoll;

// Binärausgaben
const byte Relay1 = 8;                      // Pin 14
const byte Relay2 = 7;                      // Pin 13
const byte Relay3 = A0;                     // Pin 23
const byte Relay4 = 9;                      // Pin 15 // prepared for mains frequency fast change alarm
const byte Relay5 = 10;                     // Pin 16 // prepared for mains frequency oscillation alarm 

// Hilfsvariblen Alarmausgaben
int AlarmCounter1 = 0;
int AlarmCounter2 = 0;
int AlarmCounter3 = 0;
int AlarmCounter4 = 0;
byte Alarm = 0;

// Hilfsvariable Blink
bool Blink = true;                          // Hilfsvariable

//Hilfsvariablen Sekundenmesstakt
int sc;                                     // Hilfsvariable

// ****************************************************************************************************



void setup()
{

  noInterrupts();                           // Alle Interrupts temporär abschalten

  // serial init
  Serial.begin(19200, SERIAL_8N1);
  UBRR0 = UBRR;                             // Korrekturwert/Anpassung an den 10MHz-Quarz

  // RUN-LED init
  pinMode(LED, OUTPUT);                     // LED Ausgabe vorbereiten

  //init Interrupt input    A-MESSUNG
  const byte interruptPinA = 2;
  attachInterrupt(digitalPinToInterrupt(interruptPinA), AMess, FALLING ); // neg- Flanke
  
  //init Interrupt input    B-MESSUNG
  const byte interruptPinB = 3;
  attachInterrupt(digitalPinToInterrupt(interruptPinB), BMess, RISING );  // pos- Flanke

  //Init Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;                // Timer nach obiger Rechnung vorbelegen
  if (Prescaler == 1024) TCCR1B |= (1 << CS12) | (1 << CS10);
  if (Prescaler == 256)  TCCR1B |= (1 << CS12);
  if (Prescaler == 64)   TCCR1B |= (0 << CS12) | (1 << CS11) | (1 >> CS10);
  if (Prescaler == 8)    TCCR1B |= (1 << CS11);
  if (Prescaler == 0)    TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << TOIE1);   // Timer Overflow Interrupt aktivieren

  //Init Timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;     // Reset Interrupts
  TCNT2 = 0;
  //  TCCR0 |= (1<<CS02) | (1<<CS00); // PRESCALER 1024
  if (Prescaler == 1024) TCCR2B  |= (1 << CS22) | (1 << CS20);
  if (Prescaler == 256)  TCCR2B  |= (1 << CS22);
  if (Prescaler == 64)   TCCR2B  |= (0 << CS22) | (1 << CS21) | (1 >> CS20);
  if (Prescaler == 64)   TCCR2B  = (1 << CS22);  
  if (Prescaler == 8)    TCCR2B  |= (1 << CS21);
  if (Prescaler == 0)    TCCR2B  |= (1 << CS20);
  TIMSK2 = (1 << TOIE2);   // Timer Overflow Interrupt aktivieren

  // Jumper-Input => Auswahl Protokolle
  pinMode(Jumper51,   INPUT_PULLUP);     // PIN = Eingang
  pinMode(Jumper52,   INPUT_PULLUP);     // PIN = Eingang
  pinMode(Jumper53,   INPUT_PULLUP);     // PIN = Eingang

  // Ausgang/Output
  pinMode(Relay1, OUTPUT);               // Alarm-Ausgabe vorbereiten
  pinMode(Relay2, OUTPUT);               // Alarm-Ausgabe vorbereiten
  pinMode(Relay3, OUTPUT);               // Alarm-Ausgabe vorbereiten
  pinMode(Relay4, OUTPUT);               // Alarm-Ausgabe vorbereiten
  pinMode(Relay5, OUTPUT);               // Alarm-Ausgabe vorbereiten

  interrupts();             // alle Interrupts scharf schalten

  wdt_enable(WDTO_8S);      //watchdog 8-Sekunden => Start-Watchdog


  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H"); 

  Serial.println("NFA-RPi 2.0 extendend mfm (@Arduino)");
  Serial.println("02'2021 J.Müller www.pc-projekte.de");

  if ( !(FA == 50) && !(FA == 60 ))
  {    
  Serial.print("Auto-Detect");

  int diff;  
  int d = 0;
  bool fOK = false;
  do 
  {
    for (d; d < 5; d++)
    {
      delay(500);
      Serial.print('.');  
    }
    MessA = (Dividend * 1000) / CountA;   // Berechne vorläufige Netzfrequenz
    MessB = (Dividend * 1000) / CountB;   // Berechne vorläufige Netzfrequenz
    diff = (int)abs(MessA-MessB);
    d = 4;
    fOK = (diff < 10) && (MessA > 45000) && (MessA < 65000);
    if (!fOK) Serial.print("*");  
  }  
  while ( !fOK );
  
  delay(500);
  
  if ((MessA > 45000) && (MessA < 65000))
  {
   
    if ((MessA > 45000) && (MessA < 55000))  // Messwertauswahl 50Hz
    {
      FA = 50;
      Serial.println("  => detect 50Hz    ");
      delay(1000); 
    };
    
     if ((MessA > 55000) && (MessA < 65000))     // Messwertauswahl 60Hz
     {
      FA = 60;
      Serial.println("  => detect 60Hz    ");
      delay(1000);
     } 
     CacheA = MessA;
     CacheB = MessB;   
    }     
  } 
  else // Voreinstellung Netzfrequenz
  {
    if (FA == 50) Serial.println("User setting : 50Hz mains frequency");
    if (FA == 60) Serial.println("User setting : 60Hz mains frequency");
  }

  Protokoll = Protokollauswahl();
  if (FA == 60) Settings_60Hz();  
  
  Serial.print("Output protocol: "); Serial.println(Protokoll);
  delay(500);
  Serial.println("Start mains frequency measurement...");
  Serial.write(7);
  delay(500);
 

  wdt_enable(WDTO_1S);      //watchdog 1-Sekunde => Programm-Watchdog nach Start-Phase
  ResetCounter = 0;
 
}



void loop()
{

    Reset_OK = false;
    
  // A-MESSUNG
  if ( AuswertungA == true)
  {

    MessA = (Dividend * 1000) / CountA;           // Berechne vorläufige Netzfrequenz   ... X 1000 = mHz

    // Prüfung: Messwert im Gültigkeitsbereich
    CheckA = false;
    if ( (MessA >= 45000) && (MessA <= 55000) && (FA == 50)) CheckA = true;
    if ( (MessA >= 55000) && (MessA <= 65000) && (FA == 60)) CheckA = true;

    // Wenn der CheckA ok, dann weiter zum Differenzfilter / Ausgabe
    if ( CheckA == true)
    {
      FrequenzA = Diff_filterA(MessA);
      Reset_OK = true;
    } 
  }

  // B-MESSUNG
  if ( AuswertungB == true)
  {

    MessB = (Dividend * 1000) / CountB;           // Berechne vorläufige Netzfrequenz   ... X 1000 = mHz
 
    // Prüfung: Messwert im Gültigkeitsbereich
    CheckB = false;
    if ( (MessB >= 45000) && (MessB <= 55000) && (FA == 50)) CheckB = true;
    if ( (MessB >= 55000) && (MessB <= 65000) && (FA == 60)) CheckB = true;

    // Wenn der CheckB ok, dann weiter zum Differenzfilter / Ausgabe
    if ( CheckB == true)
    {
      FrequenzB = Diff_filterB(MessB);  
      Reset_OK = true;   
    }
  }


  if (AuswertungA)
  {
    if ( Delta_filter(FrequenzA,FrequenzB) || (DeltaA && DeltaB) )
    {
      Frequenz = FrequenzA + Calibration;
    }
    //else ;//Serial.println("Delta-Fehler A-Messung");
  }
  
  if (AuswertungB)
  {    
    if ( Delta_filter(FrequenzA,FrequenzB)  || (DeltaA && DeltaB)  )
    {
      Frequenz = FrequenzB + Calibration;
    }
    //else ;// Serial.println("Delta-Fehler B-Messung");
  }

  // Hier kommt die Frequenzauswertung---
  if (AuswertungA || AuswertungB)
  {
    AuswertungA = false;
    AuswertungB = false;
//      DeltaA = false;
//      DeltaB = false;

    // Frequenzalarm
    Frequenzalarm(Frequenz);

    //Protokollauswahl // Hier kommt die Protokollauswahl...
    Protokoll = Protokollauswahl();
  
    switch (Protokoll) 
    {
    case 1:    //Protokoll1  // 7-Byte
    {
          Serial.print(Frequenz);// Ausgabe des gemessenen Netzfrequenzwertes Messung + Kalibrierungswer 
          Serial.print('\r');
          Serial.print('\n');
          //Serial.println();
          break;
    } // case1
    
    case 2:    //Protokoll2  17-Byte    
    //                      0         1       
        {                 //012345678901234567
          char Str17[18] = "50000;00000;000";   
          word Freq2 = Frequenz;
          int NLD = MWAusgabe(Frequenz);
          byte Alb = Alarm;
          byte Parity = 0;
          Str17[15] = '\r'; // Carrige Return für SerialOutput
          Str17[16] = '\n'; // Carrige Return für SerialOutput          
          Str17[17] = '\0'; // String Ende         
          byte b;
          
          // Ausgabe Frequenz       
          for (int d = 5; d > 0; d--)
          {
            b = Freq2 % 10;
            Str17[d-1] = (b+48);
            Freq2 = Freq2 / 10;
          }

          // Ausgabe Netzlastdifferenz
          if (NLD > 9999) NLD = 9999;   // maxmimale Grenze Netzlastdifferenz Ausgabe
          if (NLD < -9999) NLD = -9999; // minimale Grenze  Netzlastdifferenz Ausgabe
          if (FA == 50)
          {
            if (Frequenz < 50000) Str17[6] = '-'; // negatives Vorzeichen 
            if (Frequenz > 50000) Str17[6] = '+'; // positives Vorzeichen   
          }
          if (FA == 60)
          {
            if (Frequenz < 60000) Str17[6] = '-'; // negatives Vorzeichen 
            if (Frequenz > 60000) Str17[6] = '+'; // positives Vorzeichen   
          }
          
          NLD = abs(NLD);
          for (int d = 10; d > 6; d--)
          {
            b = NLD % 10;
            Str17[d] = (b+48);
            NLD = NLD / 10;
          }

          // Ausgabe Alarme          
          Alb = Alarm;     
          for (int d = 14; d > 11; d--)
          {
            b = Alb % 10;
            Str17[d] = (b+48);
            Alb = Alb / 10;
          }

          // Sende 17-Bit-String
          Serial.print(Str17);           
      
          break;
        }// case2
         
      
    case 3:    //Protokoll3 24-Byte
    //                      0         1         2
        {                 //01234567890123456789012
          char Str22[24] = "NFA50000f00000P000M00C";   
          word Freq3 = Frequenz;
          int NLD = MWAusgabe(Frequenz);
          byte Alb = Alarm;
          byte Parity = 0;    
          byte b;
          
          // Ausgabe Frequenz       
          for (int d = 7; d > 2; d--)
          {
            b = Freq3 % 10;
            Str22[d] = (b+48);
            Freq3 = Freq3 / 10;
          }

          // Ausgabe Netzlastdifferenz
          if (NLD > 9999) NLD = 9999;   // maxmimale Grenze Netzlastdifferenz Ausgabe
          if (NLD < -9999) NLD = -9999; // minimale Grenze  Netzlastdifferenz Ausgabe
          if (FA == 50)
          {
            if (Frequenz < 50000) Str22[9] = '-'; // negatives Vorzeichen 
            if (Frequenz > 50000) Str22[9] = '+'; // positives Vorzeichen   
          }
          if (FA == 60)
          {
            if (Frequenz < 60000) Str22[9] = '-'; // negatives Vorzeichen 
            if (Frequenz > 60000) Str22[9] = '+'; // positives Vorzeichen   
          }
          NLD = abs(NLD);
          for (int d = 13; d > 9; d--)
          {
            b = NLD % 10;
            Str22[d] = (b+48);
            NLD = NLD / 10;
          }

          // Ausgabe Alarme          
          Alb = Alarm;     
          for (int d = 17; d > 14; d--)
          {
            b = Alb % 10;
            Str22[d] = (b+48);
            Alb = Alb / 10;
          }

          // Berechnung Prüfsumme
          for (int d = 3; d < 19; d++)
          {
            Parity = Parity ^ (byte)Str22[d];
          }

          byte k = Parity;
          
          // Ausgabe Prüfsumme
          for (int d = 20; d > 18; d--)
          {
            b = Parity % 10;
           // if (b <= 9) Str22[d] = (b+48); else Str22[d] = (b+55);
           Str22[d] = (b+48);
           Parity = Parity / 10;
          }

          Str22[0] = 'N';
          Str22[1] = 'F';
          Str22[2] = 'A';
          Str22[22] = '\r'; // Carrige Return für SerialOutput
          Str22[23] = '\n'; // Carrige Return für SerialOutput          
          Str22[24] = '\0'; // String Ende   

          // Sende 22-Bit-String
          Serial.print(Str22);            
         
          break;
        }// case4

    case 4:    //Protokoll4 29Bit Teilstring CSV-Datei
    //                      0         1         2         3 
        {                 //0123456789012345678901234567890
          char Str28[29] = "50000;00000;0;0;0;0;0;0;0;0";   
          word Freq4 = Frequenz;
          int NLD = MWAusgabe(Frequenz);
          byte Alc = Alarm;
          byte Parity = 0;
          Str28[27] = '\r'; // Carrige Return für SerialOutput
          Str28[28] = '\n'; // Carrige Return für SerialOutput         
          Str28[29] = '\0'; // String Ende         
          byte b;
          
          // Ausgabe Frequenz       
          for (int d = 5; d > 0; d--)
          {
            b = Freq4 % 10;
            Str28[d-1] = (b+48);
            Freq4 = Freq4 / 10;
          }

          // Ausgabe Netzlastdifferenz
          if (NLD > 9999) NLD = 9999;   // maxmimale Grenze Netzlastdifferenz Ausgabe
          if (NLD < -9999) NLD = -9999; // minimale Grenze  Netzlastdifferenz Ausgabe
          if (FA == 50)
          {
            if (Frequenz < 50000) Str28[6] = '-'; // negatives Vorzeichen 
            if (Frequenz > 50000) Str28[6] = '+'; // positives Vorzeichen
          }
          if (FA == 60)
          {
            if (Frequenz < 60000) Str28[6] = '-'; // negatives Vorzeichen 
            if (Frequenz > 60000) Str28[6] = '+'; // positives Vorzeichen
          }
             
          NLD = abs(NLD);
          for (int d = 10; d > 6; d--)
          {
            b = NLD % 10;
            Str28[d] = (b+48);
            NLD = NLD / 10;
          }

          // Ausgabe Alarme         
           if ( (Alarm & 1) == 1) Str28[12] = '1' ; else Str28[12] = '0' ; 
           if ( (Alarm & 2) == 2) Str28[14] = '1' ; else Str28[14] = '0' ; 
           if ( (Alarm & 4) == 4) Str28[16] = '1' ; else Str28[16] = '0' ; 
           if ( (Alarm & 8) == 8) Str28[18] = '1' ; else Str28[18] = '0' ; 
           if ( (Alarm & 16) == 16) Str28[20] = '1' ; else Str28[20] = '0' ; 
           if ( (Alarm & 32) == 32) Str28[22] = '1' ; else Str28[22] = '0' ; 
           if ( (Alarm & 64) == 64) Str28[24] = '1' ; else Str28[24] = '0' ; 
           if ( (Alarm & 128) == 128) Str28[26] = '1' ; else Str28[26] = '0' ; 
           
          Serial.print(Str28);
        
          break;
        }// case4      
    } // Switch

    if (sc > 50) // Sekundendurchlauf...
    {
      sc = 0;
            
      Alarm = Alarmausgabe();      
    }
        
    // LED 1
    Toggle = !Toggle;                       // toggle ausführen  LED-Ausgabe
    if (Toggle == true) digitalWrite(LED, HIGH); else digitalWrite(LED, LOW);

    // Watchdog- Reset
    if ( (FA == 50) && ((Frequenz > 44999) && (Frequenz < 55001))) wdt_reset();  
    if ( (FA == 60) && ((Frequenz > 54999) && (Frequenz < 65001))) wdt_reset();  
  } // if AuswertungA || AuswertungB

  // Überlauf Fehlererkennung...
  if (Reset_OK) ResetCounter = 0;
  else
  {
    ResetCounter++;
    if (ResetCounter >= 200)
    {
      Serial.println("failt / Error measurement (Trigger ResetCounter)");
      while (true) { delay(100); } // provoke watchdog reset
    }
  } 
  
  delay(5); // 5ms warten... bis Rücksprung Loop Anfang
 }


// Diff-Filter

word Diff_filterA(word filter_wert)
{

  int Diff = (int)filter_wert - CacheA;
  DeltaA = false;
  // bei Überschreitung DiffMAX    
  if ( abs(Diff) > DiffMAX )
  {
    if ( Diff > 0) filter_wert = CacheA + DiffMAX;  // Änderung (positive Steigung) max 
    if ( Diff < 0) filter_wert = CacheA - DiffMAX;  // Änderung (negative Steigung) max 
    DeltaA = true;
  } 
  
  CacheA = (int)filter_wert;
  return filter_wert;
}

word Diff_filterB(word filter_wert)
{

  int Diff = (int)filter_wert - CacheB;
  DeltaB = false;
  // bei Überschreitung DiffMAX    
  if ( abs(Diff) > DiffMAX )
  {
    if ( Diff > 0) filter_wert = CacheB + DiffMAX;  // Änderung (positive Steigung) max 
    if ( Diff < 0) filter_wert = CacheB - DiffMAX;  // Änderung (negative Steigung) max 
    DeltaB = true;
  } 
  
  CacheB = (int)filter_wert;
  return filter_wert;
}

// Delta Filter
bool Delta_filter(word FreqA, word FreqB)
{
  int xDiff = (int)FreqA - (int)FreqB;
  bool OK;

  // bei Überschreitung DeltaMAX    
  if ( abs(xDiff) >= 10 ) 
  {
    OK = false;
  }
  else OK = true;
  return OK;
}


// Netzlastdifferenz Berechnung / Ausgabe
int MWAusgabe(float Hz)
{
  float Hz_Diff ;
  if (FA == 50) Hz_Diff = ( Hz - 50000 );
  if (FA == 60) Hz_Diff = ( Hz - 60000 );
  bool Vorzeichen;// = false;           // true = neg. Vorzeichen
  if ( abs(Hz_Diff) != Hz_Diff) Vorzeichen = true; else Vorzeichen = false;
  Hz_Diff = ( abs(Hz_Diff ) - Offset ) ;
  if (Hz_Diff < 0) Hz_Diff = 0;
  int Last = (int)(Hz_Diff * 15.333333333);
  if (Last > 9999) Last = 9999;
  if (Vorzeichen == true) Last *= -1;   
  return Last;
}


byte Protokollauswahl()
{
  // Jumper-Input-Abfrage LOW-Aktiv!
  byte Auswahl; 
  if ( (digitalRead(Jumper51)== LOW )  && (digitalRead(Jumper52)== LOW)  ) { Auswahl = 4;};  // 29-Byte-Protokoll
  if ( (digitalRead(Jumper51)== HIGH ) && (digitalRead(Jumper52)== LOW) )  { Auswahl = 3;};  // 24-Byte-Protokoll
  if ( (digitalRead(Jumper51)== LOW )  && (digitalRead(Jumper52)== HIGH) ) { Auswahl = 2;};  // 17-Byte-Protokoll
  if ( (digitalRead(Jumper51)== HIGH ) && (digitalRead(Jumper52)== HIGH))  { Auswahl = 1;};  //  7-Byte-Protokoll
  return Auswahl;
}


void Settings_60Hz()
{
  //Frequency trigger settings for 60Hz
  f1g = 60070;       // fmax Alert
  f1k = 59930;       // fmin Alter
  f2g = 60100;       // fmax Alert
  f2k = 59900;       // fmin Alter
  f3g = 60200;       // fmax Alert
  f3k = 59800;       // fmin Alter
  f4g = 61000;       // fmax Alert
  f4k = 59000;       // fmin Alter 
}


// Frequenzalarme, Ausgabe auf Port xx, JXX
void Frequenzalarm(word f)
{
  if ( (f < f1k) || (f > f1g) ) AlarmCounter1 = tf1;
  if ( (f < f2k) || (f > f2g) ) AlarmCounter2 = tf2;
  if ( (f < f3k) || (f > f3g) ) AlarmCounter3 = tf3;
  if ( (f < f4k) || (f > f4g) ) AlarmCounter4 = tf4;
}

// Alarmausgabe Binär und Protokoll
byte Alarmausgabe()
{
  // AUswertung der ALarme...

  byte Al = 0;
  if ( AlarmCounter1 > 0)
  {
    AlarmCounter1 --;
    Al |= (1+128);
    if (LowAktiv & Blink) digitalWrite(Relay1, LOW); else digitalWrite(Relay1, HIGH);
  } else if (!LowAktiv & Blink) digitalWrite(Relay1, HIGH); else digitalWrite(Relay1, LOW);
  
  if ( AlarmCounter2 > 0)
  {
    AlarmCounter2 --;
    Al |= (2+128);
    if (LowAktiv & Blink) digitalWrite(Relay2, LOW); else digitalWrite(Relay2, HIGH);
  } else if (!LowAktiv & Blink) digitalWrite(Relay2, HIGH); else digitalWrite(Relay2, LOW);
  
  if ( AlarmCounter3 > 0)
  {
    AlarmCounter3 --;
    Al |= (4+128);
    if (LowAktiv & Blink) digitalWrite(Relay3, LOW); else digitalWrite(Relay3, HIGH);
  } else if (!LowAktiv & Blink) digitalWrite(Relay3, HIGH); else digitalWrite(Relay3, LOW);

  if ( AlarmCounter4 > 0)
  {
    AlarmCounter4 --;
    Al |= (8+128);
    if (LowAktiv & Blink) digitalWrite(Relay4, LOW); else digitalWrite(Relay4, HIGH);
  } else if (!LowAktiv & Blink) digitalWrite(Relay4, HIGH); else digitalWrite(Relay4, LOW);
  
  if ( Al > 0) // common alarm flashing signal   Sammelalarm mit Blibnkausgabe
  {
    if (LowAktiv & Blink) digitalWrite(Relay5, LOW); else digitalWrite(Relay5, HIGH);
  } else if (!LowAktiv & Blink) digitalWrite(Relay5, HIGH); else digitalWrite(Relay5, LOW);

//  if ( Al > 0) // common alarm without flashing    Sammelalarm ohne Blinkausgabe
//  {
//    if (LowAktiv) digitalWrite(Relay5, LOW); else digitalWrite(Relay5, HIGH);
//  } else if (!LowAktiv) digitalWrite(Relay5, HIGH); else digitalWrite(Relay5, LOW);

  if (BlinkON) Blink = !Blink;
 
  return Al;
}


// Überlauf Timer1
ISR(TIMER1_OVF_vect) 
{
  Ueberlauf ++;
  Reset_AMess();
  if (Ueberlauf >= 5)
  {
    Serial.println("ERROR measurement");
    while (true)
    { delay(100); } // provoke watchdog reset
  }
}

// Überlauf Timer2 für virtuellen 16-Bit-Counter
ISR(TIMER2_OVF_vect)  
{ // called by timer2
    T2Counter ++;
    BTriggerCounter++;  
}


void Reset_AMess()
{
      TCNT1 = 0;                         // lösche Wert Timer1
      PeriodeA = 0;                      // setze Periodenzähler zurück
      AuswertungA = false;               // Auswertung aktivieren
      TriggersperrA = Sperr;   
}


void AMess()
{
  // Messroutine

  if (TriggersperrA < TCNT1)              // 1. Überprüfung: Sperrzeit eingehalten ?
  {
    TriggersperrA = TCNT1 + Sperr;        // neue Sperrzeit
    PeriodeA++;                           // Periodenzähler

    // Auswertung nach letzer Periode
    if (PeriodeA >= Messperiode)
    {
      CountA =  TCNT1;                   // Übernehme Countstand Timer1
      TCNT1 = 0;                        // lösche Wert Timer1
      PeriodeA = 0;                      // setze Periodenzähler zurück
      AuswertungA = true;                // Auswertung aktivieren
      TriggersperrA = Sperr;             // Rücksetzen der Sperrzeit (1.Periode)    
    }
  }
  sc++; // Sekundenmesstaktzähler
}

void BMess()
{
  // Messroutine
  if (TriggersperrB < (BTriggerCounter *256) + TCNT2)              // 1. Überprüfung: Sperrzeit eingehalten ?
  {
    TriggersperrB = (BTriggerCounter *256) + TCNT2 + Sperr;        // neue Sperrzeit
    PeriodeB++;                           // Periodenzähler

    // Auswertung nach letzer Periode
    if ((PeriodeB >= Messperiode) && ( PeriodeA = 9 ))
    {
      CountB = (T2Counter *256) + TCNT2 ;         
      TCNT2  = 0;       
      T2Counter = 0;
      PeriodeB  = 0;                      // setze Periodenzähler zurück
      TriggersperrB = Sperr;
      BTriggerCounter = 0;
      AuswertungB = true;                // Auswertung aktivieren
    }
  }
  
}
