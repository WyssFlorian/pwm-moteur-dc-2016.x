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
    // À implémenter....
    return AVANT;
}

/**
 * Indique le cycle de travail PWM correspondant à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return Cycle de travail du PWM.
 */
unsigned char conversionMagnitude(unsigned char v) {
    // À implémenter...
    return 0;
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
    
    // Prépare Temporisateur 2 pour PWM (compte jusqu'à 255 en 1ms):
    
    T2CONbits.TMR0ON = 1; //active le timer 0.
    T2CONbits.T0CS = 0; //On utilise Fosc/4 comme source
    T2CONbits.PSA = 1; //Pas de diviseur de fréquence
    
    //Etablit le compteur du timer 0 pour un débordement 
    
    TMR0H = 0xFC; //TMR0 =655036-[(255*4)/1E6] = 64556 = FC2C [exadécimal] 
    TMR0L = 0x2C; 

    /* Compte les interruptions du temporisateur pour mesurer les secondes */

   
    
    /*Routine des interruptions: Détecte de quelle interruption il s'agit puis 
     appelle la fonction correspondante*/
    
    
    
    // Configure PWM 1 pour émettre un signal de 1KHz:
    // À faire...

    // Configure RC0 et RC1 pour gérer la direction du moteur:
    // À faire...
    
    // Active le module de conversion A/D:
    
    ADCON0bits.ADON = 1; //allume le module A/D
    ADCON0bits.CHS = 9; //branche le convertisseur sur AN9
    ADCON2bits.ADFM = 1; //les 8 bits les moins signifiants sont sur ADRESL
    ADCON2bits.ACQT = 2; //temps d'acquisiton à 4 TAD
    ADCON2bits.ADCS = 0; // À 1 MHz, le TAD est à 2 us

    // Active les interruptions générales:
    
     RCONbits.IPEN = 0; //Désactive le mode Haute / Basse priorité.
    INTCONbits.GIEH = 0; //Désactive les interruptions hautes priorité.
    INTCONbits.GIEL = 1; //Active les interruptions basses priorité.
    
    INTCONbits.TMR0IE = 1; //Active les interruptions timer 0.
    PIE1bits.ADIE = 1; //Active les interruptions A/D.
    IPR1bits.ADIP = 1; //Interr. A/D sont de haute priorité.
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
