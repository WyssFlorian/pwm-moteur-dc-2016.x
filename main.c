#include <xc.h>
#include "test.h"

/**
 * Bits de configuration:
 */
#pragma config FOSC = INTIO67   // Osc. interne, A6 et A7 comme IO.
#pragma config IESO = OFF       // Pas d'osc. au démarrage.
#pragma config FCMEN = OFF      // Pas de monitorage de l'oscillateur.

// Nécessaires pour ICSP / ICD:
#pragma config MCLRE = EXTMCLR  // RE3 est actif comme master reset.
#pragma config WDTEN = OFF      // Watchdog inactif.
#pragma config LVP = OFF        // Single Supply Enable bits off.

typedef enum {
    AVANT = 0b01,
    ARRIERE = 0b10
} Direction;

/**
 * Indique la direction correspondante à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return AVANT ou ARRIERE.
 */
Direction conversionDirection(unsigned char v) {
    if (v<128){
    return ARRIERE; 
    }
    return AVANT;
}

/**
 * Indique le cycle de travail PWM correspondant à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return Cycle de travail du PWM.
 */
unsigned char conversionMagnitude(unsigned char v) {
    if (v < 128){
    return 254-2*v;
    }
    return 2*v-256;
}

#ifndef TEST

/**
 * Initialise le hardware.
 */
static void hardwareInitialise() {
    
    ANSELA = 0X00;
    ANSELB = 0X01;
    ANSELC = 0X00;
   
    TRISC = 0x00; //Configure le port C comme sortie digitale
    
    /*Configuration du Fosc à 1 MHz*/
    
    OSCCONbits.IRCF = 3; //inTosc à 1 MHz (facultatif val par défault = 1 MHz)
    OSCCONbits.SCS = 2; //Choix de l'scillateur interne (facultatif)
    
    // Prépare Temporisateur 2 pour PWM (compte jusqu'à 255 en 1ms):
    
    T2CONbits.TMR2ON = 1; //active le timer 2.
    T2CONbits.T2CKPS = 0; //Pas de diviseur de fréquence de FOSC/4
    T2CONbits.T2OUTPS = 9;// Postscaler d'interruptions à 1/10 = 10 ms (p.177))
    
    // PR2=255; (Période en "pas" ou en définissant la valeur du compteur ?)
    
    //Etablit le compteur du timer pour un débordement à 255 
    TMR0H = 0xFC; //TMR0 =655036-[(255*4)/1E6] = 64556 = FC2C [exadécimal] 
    TMR0L = 0x2C;
    
    // Configure PWM 1 pour émettre un signal de 1KHz:
    
    /*Active le PWM sur CCP1*/
    CCPTMRS0bits.C1TSEL = 0; // Raccorde le CCP1 sur TM2 (p202)
    CCP1CONbits.P1M = 0; // PWM simple, sortie uniquement sur P1A.
    CCP1CONbits.CCP1M = 0; // P1A est actif à niveau haut.
    CCP1CONbits.DC1B = 0; // 2 bits les - sign. pour préc. le cyc. de travail

    // Configure RC0 et RC1 pour gérer la direction du moteur:
    
    TRISCbits.RC0=0; //RC0 mise en sortie
    TRISCbits.RC1=0; //RC1 mise en sortie
    
    // Active le module de conversion A/D:
    
    TRISBbits.RB3=1; //RB3 en entrée
    ADCON0bits.ADON = 1; //allume le module A/D
    ADCON0bits.CHS = 9; //branche le convertisseur sur AN9
    ADCON2bits.ADFM = 1; //les 8 bits les moins signifiants sont sur ADRESL
    ADCON1bits.PVCFG=0; // Vref+ en interne sur VDD (p297)
    ADCON1bits.NVCFG=0; // Vref- sur Vss (0V)
    ADCON2bits.ACQT = 2; //temps d'acquisiton à 4 TAD
    ADCON2bits.ADCS = 0; // À 1 MHz, le TAD est à 2 us

    // Active les interruptions générales:
    
    RCONbits.IPEN = 1; //Gère les alarmes en mode Basse priorité.
    INTCONbits.GIE = 1; //Active les interruptions.
    INTCONbits.PEIE = 1; //Active les interruptions périphériques.
    
    PIE1bits.TMR2IE = 1; //Active les interruptions timer 2.
    PIE1bits.ADIE = 1; //Active les interruptions A/D.
}

/**
 * Point d'entrée des interruptions.
 */
void low_priority interrupt interruptionsBassePriorite() {
    if (PIR1bits.TMR2IF) { //lance une conversion
        PIR1bits.TMR2IF = 0;
        ADCON0bits.GO = 1; 
    }
    
    if (PIR1bits.ADIF) { //Commande le moteur
        PIR1bits.ADIF = 0;
        PORTC = conversionDirection(ADRESH);
        CCPR1L = conversionMagnitude(ADRESH);
    }
}

/**
 * Point d'entrée pour l'émetteur de radio contrôle.
 */
void main(void) {
    hardwareInitialise();

    while(1);
}
#endif

#ifdef TEST
void testConversionMagnitude() {
    testeEgaliteEntiers("CM01", conversionMagnitude(0),   254);
    testeEgaliteEntiers("CM02", conversionMagnitude(1),   252);
    testeEgaliteEntiers("CM03", conversionMagnitude(2),   250);
    
    testeEgaliteEntiers("CM04", conversionMagnitude(125),   4);
    testeEgaliteEntiers("CM05", conversionMagnitude(126),   2);
    
    testeEgaliteEntiers("CM06", conversionMagnitude(127),   0);
    testeEgaliteEntiers("CM07", conversionMagnitude(128),   0);

    testeEgaliteEntiers("CM08", conversionMagnitude(129),   2);
    testeEgaliteEntiers("CM09", conversionMagnitude(130),   4);
    
    testeEgaliteEntiers("CM10", conversionMagnitude(253), 250);
    testeEgaliteEntiers("CM11", conversionMagnitude(254), 252);
    testeEgaliteEntiers("CM12", conversionMagnitude(255), 254);
}
void testConversionDirection() {
    testeEgaliteEntiers("CD01", conversionDirection(  0), ARRIERE);    
    testeEgaliteEntiers("CD02", conversionDirection(  1), ARRIERE);    
    testeEgaliteEntiers("CD03", conversionDirection(127), ARRIERE);    
    testeEgaliteEntiers("CD04", conversionDirection(128), AVANT);
    testeEgaliteEntiers("CD05", conversionDirection(129), AVANT);
    testeEgaliteEntiers("CD06", conversionDirection(255), AVANT);    
}
void main() {
    initialiseTests();
    testConversionMagnitude();
    testConversionDirection();
    finaliseTests();
    while(1);
}
#endif
