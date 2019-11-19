/******************************************************************************/
/*                                                                            */
/* Ce code a ete modifie par Matthew Khouzam et Jerome Collin                 */
/* Pour le cours de INF1995                                                   */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                               memoire_24.cpp                               */
/*                                                                            */
/* Exemple de lecture sequentielle et aleatoire et d'eriture aleatoire et en  */
/* mode page en utilisant une memoire I2C 24LC256 avec l'interface materiel   */
/* d'un microcontroleur Atmel atmega16. Les broches d'adresses A0, A1 et A2   */
/* de la memoire sont reliees a la masse. SDA est relie a la broche PC1 du    */
/* microcontroleur, SCL a PC0 et WP n'est pas relie.                          */
/*                                                                            */
/* Le code de l'eeprom est : bits 7, 6, 5 et 4 = 1 0 1 0                      */
/* L'adresse physique est  : bits 3, 2, 1 = 0 0 0                             */
/* Le code d'ecriture est   : bit 0 a 1 = lecture et bit 0 a 0 = ecriture     */
/* Le code de controle est 0xa1 pour une lecture et 0xa0 pour une ecriture    */
/*                                                                            */
/*  Pour les microcontroleurs Atmel, l'interface I2C est appelee TWI.         */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

#ifndef F_CPU
#define F_CPU 8'000'000
#endif

#include "ExternalMemory.h"

namespace lib
{
   uint8_t ExternalMemory::m_peripheralAddress = 0xA0;

   /******************************************************************************/
   /* void Memoire24CXXX::Memoire24CXXX()                                        */
   /*                                                                            */
   /*      Constructeur: ajuste la taille de la page et procede a                */
   /*                    l'initialisation                                        */
   /*                                                                            */
   /* Parametre d'entree  : aucun                                                */
   /* Parametre de sortie : aucun                                                */
   /******************************************************************************/
   ExternalMemory::ExternalMemory()
   : PAGE_SIZE(128)
   {
      init();
   }


   /******************************************************************************/
   /* void Memoire24CXXX::init(void)                                             */
   /*                                                                            */
   /*      Initialisation du port serie et de l'horloge de l'interface I2C       */
   /*                                                                            */
   /* Parametre d'entree  : aucun                                                */
   /* Parametre de sortie : aucun                                                */
   /******************************************************************************/
   void ExternalMemory::init()
   {
      selectBank(0);
      // Initialisation de l'horloge de l'interface I2C
      TWSR = 0;
      // prediviseur
      TWBR =  (F_CPU / 100000UL - 16) / 2;

   }

   /******************************************************************************/
   /* uint8_t Memoire24CXXX::choisir_banc(const uint8_t banc)                    */
   /*                                                                            */
   /*      Choisir un banc de memoire                                            */
   /*                                                                            */
   /* Parametre d'entree  : uint8_t bank - le banc de memoire a choisir          */
   /* Parametre de sortie : uint8_t      - rv si c'est un succes, 255 si echec   */
   /******************************************************************************/
   uint8_t ExternalMemory::selectBank(const uint8_t bank)
   {
      uint8_t temp = bank & 0x03;
      uint8_t rv = 255;
      if (bank == temp)
      {
         m_peripheralAddress = (0xA0 | ( bank << 1 ));
         rv = m_peripheralAddress;
      }
      return rv;
   }


   /******************************************************************************/
   /*                Lecture sequentielle de l'eeprom I2C                        */
   /*                                                                            */
   /* Le microcontroleur est en mode maitre. Il realise un acquittement (ACK) de */
   /* la memoire a la fin de chaque lecture a l'exception de la derniere ou il   */
   /* n'acquitte pas la reception (NACK) pour que la memoire place la ligne SDA  */
   /* a un niveau haut et libere le bus et il transmet ensuite la condition      */
   /* d'arret.                                                                   */
   /*                                                                            */
   /* a la premiere etape on verifie si le cycle d'ecriture precedent est        */
   /* termine. On transmet :                                                     */
   /*   - La condition de depart                                                 */
   /*   - Une donnee de controle : code de la memoire + adresse physique         */
   /*                              + commande d'ecriture                         */
   /*   Si le cycle d'ecriture precedent est en cours on recommence la sequence  */
   /*                                                                            */
   /* A la deuxieme etape on transmet :                                          */
   /*   - La condition de depart                                                 */
   /*   - Une donnee de controle : code de la memoire + adresses physique        */
   /*                              + commande d'ecriture                         */
   /*   - L'adresse : poids fort suivi du poids faible                           */
   /*                                                                            */
   /* A la troisieme etape on transmet :                                         */
   /*   - La condition de depart                                                 */
   /*   - Une donnee de controle : code de la memoire + adresse physique         */
   /*                              + commande de lecture                         */
   /*                                                                            */
   /* A la quatrieme etape on realise une lecture sequentielle :                 */
   /*   - Lecture en boucle - Commande de depart de transmission avec ACK,       */
   /*                         attente de reception et lecture de la donnee       */
   /*   - Derniere lecture  - Commande de depart de transmission avec NACK,      */
   /*                         attente de reception et lecture de la donnee       */
   /*                         Le NACK indique a la memoire la fin du transfert   */
   /*   - Transmission de la condition d'arret                                   */
   /*                                                                            */
   /* Parametres d'entree  : uint16_t address - adresse de debut de lecture      */
   /*                        int length       - nombre de donnees a retourner    */
   /*                                           (variante de la 2eme procedure)  */
   /* Parametres de sortie : uint8_t *data    - donnees lues                     */
   /*                                                                            */
   /******************************************************************************/
   uint8_t ExternalMemory::read(const uint16_t address, uint8_t *data)
   {
   uint8_t rv = 0;

   //______________ Attente de la fin d'un cycle d'ecriture ______________
   for (;;)
   {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    // Condition de depart
      while ((TWCR & _BV(TWINT)) == 0)   // Attente de fin de transmission
         ;

      TWDR = m_peripheralAddress;    //controle - bit 0 a 0, ecriture
      TWCR = _BV(TWINT) | _BV(TWEN);     // R. a Z., interrupt. - Depart de transm.
      while ((TWCR & _BV(TWINT)) == 0)   // Attente de fin de transmission
         ;
      if (TWSR==0x18)         // 0x18 = cycle d'ecriture termine
         break;
   }

   //_______________ Transmission de la condition de depart ________________
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     // Condition de depart
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR = m_peripheralAddress;       // Controle - bit 0 a 0, ecriture
   TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
      ;

   //______________ Transmission du poids fort de l'adresse ________________
   TWDR =  ( address >> 8 );            // 8 bits de poids fort de l'addresse
   TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
      ;

   //_____________ Transmission du poids faible de l'adresse _______________
   TWDR = address;                      // 8 bits de poids faible de l'addresse
   TWCR = _BV(TWINT) | _BV(TWEN);       // R.�Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
      ;

   //_______________ Transmission de la condition de depart ________________
   //  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);    // Condition de fin
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR =  m_peripheralAddress + 1;   // Controle - bit 0 a 1 lecture 
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //________________________ Lecture de l'eeprom __________________________
   TWCR = _BV(TWINT) | _BV(TWEN);     // R.�Z., interrupt. - Depart de transm.+NACK
   while ((TWCR & _BV(TWINT)) == 0)   // Attente de fin de transmission
      ;
   *data = TWDR;

   //________________ Transmission de la condition d'arret _________________
   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
   return rv;
   }


   uint8_t ExternalMemory::read(const uint16_t address, uint8_t *data,
                              uint8_t length)
   {
   uint8_t twcr;

   //______________ Attente de la fin d'un cycle d'ecriture ______________
   for (;;)
   {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    // Condition de depart
      while ((TWCR & _BV(TWINT)) == 0) ;   // Attente de fin de transmission

      TWDR = m_peripheralAddress;       // Controle - bit 0 a 0, ecriture
      TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
      while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
         ;
      if (TWSR==0x18)                      // 0x18 = cycle d'ecriture termine
         break;
   }

   //_______________ Transmission de la condition de depart ________________
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     // Condition de depart
   while ((TWCR & _BV(TWINT)) == 0)    // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR = m_peripheralAddress;        // Controle - bit 0 a 0, ecriture
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //______________ Transmission du poids fort de l'adresse ________________
   TWDR = address >> 8;                  // 8 bits de poids fort de l'addresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //_____________ Transmission du poids faible de l'adresse _______________
   TWDR = address;                       // 8 bits de poids faible de l'addresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //_______________ Transmission de la condition de depart ________________
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    // Condition de depart
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR =  m_peripheralAddress + 1;  // Controle - bit 0 a 1, lecture
   TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
      ;

   //________________________ Lecture de l'eeprom __________________________
   // La memoire transmet 8 bits de donnee et le recepteur transmet un
   // acquittement (ACK). Si c'est la derniere donnee le recepteur n'acquitte
   // pas la reception (NACK) et il transmet ensuite la condition de stop.
   // Le ACK est realisee par le recepteur en placant TWEA a 1 au lieu de le
   // laisser a 0.
   for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) ; length > 0; length--)
   {
         if (length == 1)
            twcr = _BV(TWINT) | _BV(TWEN);  // Derniere donnee, NACK
         TWCR = twcr;                       // R. a Z., interrupt. - Depart de transm.
         while ((TWCR & _BV(TWINT)) == 0)   // Attente de fin de transmission
            ;
         *data++ = TWDR;                    // Lecture
   }

   //________________ Transmission de la condition d'arret _________________
   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);

   return 0;
   }


   /******************************************************************************/
   /*                                                                            */
   /*      ecriture d'un bloc de donnees en mode page dans l'eeprom I2C          */
   /*                                                                            */
   /* On ne peut ecrire qu'une seule page a la fois dans une eeprom et il faut   */
   /* s'assurer de ne pas depasser cette limite pour eviter le repliement. Cette */
   /* fonction se termine apres l'ecriture d'un bloc de donnees dans une seule   */
   /* page. On doit la rappeler plusieurs fois si la longueur d'un bloc de       */
   /* donnees depasse les limites d'une page.                                    */
   /*                                                                            */
   /* Les memoires eeprom i2c ne generent pas de signal d'acquittement pendant   */
   /* un cycle d'ecriture. Pour s'assurer que l'eeprom est libre pour l'ecriture */
   /* d'une ou de plusieurs nouvelles donnees, on transmet en boucle une         */
   /* condition de depart et un octet de controle d'ecriture et on attend un     */
   /* signal d'acquittement. On peut ensuite entreprendre la procedure           */
   /* d'ecriture. Cette verification est placee au debut de la fonction car le   */
   /* temps d'acces permet a la condition de stop precedente de se completer.    */
   /*                                                                            */
   /* A la premiere, etape on calcule la longueur maximum que l'on peut placer   */
   /* dans une page afin d'eviter le repliement dans l'ecriture des donnees      */
   /*                                                                            */
   /* A la deuxieme etape, on verifie si le cycle d'ecriture precedent est       */
   /* termine. On transmet :                                                     */
   /*   - La condition de depart                                                 */
   /*   - Une donnee de controle : code de la memoire + adresse physique         */
   /*                              + commande d'ecriture                         */
   /*   Si le cycle d'ecriture precedent est en cours on recommence la sequence  */
   /*                                                                            */
   /* A la troisieme etape, on transmet :                                        */
   /*   - La condition de depart                                                 */
   /*   - Une donnee de controle : code de la memoire + adresse physique         */
   /*                              + commande d'ecriture                         */
   /*                                                                            */
   /* A la quatrieme etape, on ecrit la donnee :                                 */
   /*   - Transmission de la donnee + commande de depart de transmission avec    */
   /*     ACK et attente de fin de transmission.                                 */
   /*   - Transmission de la condition d'arret pour demarrer le cycle d'ecriture */
   /*                                                                            */
   /* Parametres d'entree  : uint16_t address - adresse de debut de lecture      */
   /*                        uint8_t *data    - donnees a ecrire dans l'eeprom   */
   /*                        int length     - longueur du bloc de donnees      */
   /* Parametre de sortie  : uint8_t rv       - nombre de donnees ecrites        */
   /*                                                                            */
   /******************************************************************************/
   uint8_t ExternalMemory::write(const uint16_t address, const uint8_t data)
   {
   //______________ Attente de la fin d'un cycle d'ecriture ______________
   for ( ; ; )
   {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    // Condition de depart
      while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
         ;

      TWDR = m_peripheralAddress;       // Controle - bit 0 a 0, ecriture
      TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
      while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
         ;

      if (TWSR==0x18)
         break;               // 0x18 = cycle d'ecriture termine
   }

   //_______________ Transmission de la condition de depart ________________
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     // Condition de depart
   while ((TWCR & _BV(TWINT)) == 0)    // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR = m_peripheralAddress;        // Controle - bit 0 a 0, ecriture
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //______________ Transmission du poids fort de l'adresse ________________
   TWDR = address >> 8;                 // 8 bits de poids fort de l'adresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //_____________ Transmission du poids faible de l'adresse _______________
   TWDR = address;                      // 8 bits de poids faible de l'adresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //______________________ Transmission de la donnee ______________________
   TWDR = data;
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //________________ Transmission de la condition d'arret _________________
   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  // Demarrage du cycle d'ecriture

   return 0;
   }


   uint8_t ExternalMemory::write(const uint16_t address, const uint8_t *data,
                                 const uint8_t length)
   {
   uint8_t rv;
   uint16_t copieAdresse = address;
   uint8_t copieLongueur = length;
   const uint8_t* copieDonnee = data;
   do
   {
         rv = writePage(copieAdresse, copieDonnee, copieLongueur);
         copieAdresse += rv;      // On pointe une nouvelle page
         copieLongueur -= rv;     // On soustrait la partie ecrite precedemment
         copieDonnee += rv;       // On avance le pointeur de donnees
   }
   while (copieLongueur > 0);

   return 0;
   }


   uint8_t ExternalMemory::writePage(const uint16_t address, const uint8_t* data,
                                    const uint8_t length)
   {
   uint16_t addr_fin;
   uint8_t rv = 0;
   uint8_t copieLongueur = length;

   // Les operations suivantes permettent de tenir compte des limites
   // de grandeur d'une page afin d'eviter le repliement dans l'ecriture
   // des donnees

   if (address + length < (address | (PAGE_SIZE - 1)))
      addr_fin = address + length;
   else
      addr_fin = (address | (PAGE_SIZE - 1)) + 1;
   copieLongueur = addr_fin - address;


   //______________ Attente de la fin d'un cycle d'ecriture ______________
   for ( ; ; )
   {
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);    // Condition de depart
      while ((TWCR & _BV(TWINT)) == 0) ;   // Attente de fin de transmission
      TWDR = m_peripheralAddress;       // Controle - bit 0 a 0, ecriture
      TWCR = _BV(TWINT) | _BV(TWEN);       // R. a Z., interrupt. - Depart de transm.
      while ((TWCR & _BV(TWINT)) == 0)     // Attente de fin de transmission
         ;

      if (TWSR==0x18)
         break;               // 0x18 = cycle d'ecriture termine
   }

   //_______________ Transmission de la condition de depart ________________
   TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);     // Condition de depart
   while ((TWCR & _BV(TWINT)) == 0)       // Attente de fin de transmission
      ;

   //__________________ Transmission du code de controle ___________________
   TWDR = m_peripheralAddress;        // Controle - bit 0 a 0, ecriture
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //______________ Transmission du poids fort de l'adresse ________________
   TWDR = address >> 8;                  // 8 bits de poids fort de l'adresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //_____________ Transmission du poids faible de l'adresse _______________
   TWDR = address;                       // 8 bits de poids faible de l'adresse
   TWCR = _BV(TWINT) | _BV(TWEN);        // R. a Z., interrupt. - Depart de transm.
   while ((TWCR & _BV(TWINT)) == 0)      // Attente de fin de transmission
      ;

   //______________________ Transmission de la page ______________________
   for ( ; copieLongueur > 0; copieLongueur--)
   {
      TWDR = *data++;
      TWCR = _BV(TWINT) | _BV(TWEN);     // R. a Z., interrupt. - Depart de transm.
      while ((TWCR & _BV(TWINT)) == 0)   // Attente de fin de transmission
         ;
      rv++;                              // Compteur de donnees
   }

   //________________ Transmission de la condition d'arrret _________________
   TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); // Demarrage du cycle d'ecriture

   return rv;
   }
} // namespace lib