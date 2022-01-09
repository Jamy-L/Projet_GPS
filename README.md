# Projet GPS

Le but de ce projet est d'utiliser un module GPS recevant diverses informations (comme l'heure et la position GPS) afin de les traiter et de les afficher. Le module se connecte au LPC 804 par une liaison série. 
***

## Caractéristiques du module GPS
Le module m'a été fourni sans aucune référence, datasheet, ou quelconque information supplémentaire. Il se compose de deux parties :

### Récepteur
Le rôle du récepteur est simplement de recevoir les informations brutes envoyées par plusieurs satellites, et de les transmettre au microcontrôleur

![Recepteur](https://user-images.githubusercontent.com/46826148/148693371-52d713a7-7c34-477e-836f-40021d1814d5.jpg)

### Microcontrôleur
Le microcontrôleur doit recevoir les données fournies par le récepteur, et les traduire en une chaîne de caractères suivant la norme NMEA. En particulier, il doit calculer sa position ainsi que l'heure locale à partir des données reçues. De plus, il doit pouvoir gérer une liaison série.

![Microcontroleur](https://user-images.githubusercontent.com/46826148/148693378-79e73187-94e5-43fb-a915-be60871ada0a.jpg)

![Microcontroleur_2](https://user-images.githubusercontent.com/46826148/148693382-976f3a70-0be9-4a0a-9d29-5d8dc4db96d1.jpg)


### Connecteur
Le connecteur est un connecteur classique de liaison série, et comporte 4 broches : TX, RX, GND et VCC. Le module doit en effet être alimenté. Il faut donc impérativement s'assurer que le module est bien branché, sans quoi on peut facilement le griller ! 

![Connecteur](https://user-images.githubusercontent.com/46826148/148693387-f7c7c611-72e0-4e2e-9533-212770d686ed.jpg)


### Format des trames
J'ai étudié à l'oscilloscope l'allure des trames reçues afin de pouvoir déchiffrer les données envoyées par le module. Les résultats sont les suivants :

* Le Baud rate est de 9600 bits/seconde
* Un train comporte 8 bits
* Il n'y a pas de bit de parité
* Il y a un bit de stop

Sans rentrer dans le détail de toutes les trames, le module envoie de manière alternative des trames de type :

* GNGGA
* GPGSV
* BDGSV
* GNGLL
* GNVTG
* GPTXT
* GNRMC

On rappelle que les deux premières lettres correspondent au type d'équipement à l'origine du signal :

* GN: GPS + GLONASS
* GP: GPS
* BD: BEIDOU

Les trois autres lettres donnent le type de la trame, chacune ayant ses spécificités :
* GGA : Contient l'heure et la position, l'altitude, le nombre de satellites visibles.
* RMC : Contient la position, l'heure, la vitesse. Utilisé plutôt pour la navigation en mer.

Les autres trames bien moins utilisées, et il semblait donc logique de choisir la trame GNGGA qui contient toutes les informations utiles.

On rappelle de plus qu'une trame NMEA commence par un "$" et se termine par "*" suivi d'un checksum, permettant de vérifier si une erreur de transmission a été faite. En pratique, je n'ai jamais observé ce type d'erreur, et n'ai donc pas vérifié le checksum par la suite.
***

## Réception de la trame
La réception de la trame se fait en configurant une liaison série. Par soucis de simplicité, j'ai directement repris le code fournit par MCUXpresso, en retirant tout ce qui était inutile. Ainsi, la fonction d'interruption pour l'UART est :

```c
void UART0_IRQHandler() {
  unsigned char temp;

  temp = LPC_USART0->RXDAT;
  rx_buffer[rx_char_counter] = temp;        // Append the current character to the rx_buffer

  if ((temp=='*')&&(etat_lecture==marche)) indice_etoile=rx_char_counter;
  if (temp=='$') etat_lecture=marche;

  if ((rx_char_counter==indice_etoile+2)&&(etat_lecture==marche)) {   // Fin de chaine.
    rx_buffer[rx_char_counter+1] = 0x0A;    // Append a new line character to rx_buffer.
    rx_buffer[rx_char_counter+2] = 0x00;    // Append a NUL terminator character to rx_buffer to complete the string.

    handshake = true;                       // Set handshake for main()
    indice_etoile=-3;
    rx_char_counter = 0;                    // Clear array index counter
    etat_lecture=arret;
  }
  else {                                    // Current character is not CR, keep collecting them.
    rx_char_counter++;                      // Increment array index counter.

    if (rx_char_counter == RX_BUFFER_SIZE){  // If the string overruns the buffer, stop here before all hell breaks lose.
        rx_char_counter=0; //remise à zéro du chariot
        indice_etoile=-3;
        etat_lecture=arret;
    }
  }

  return;
}
```

Le buffer de lecture se remplit au fur et à mesure. Lorsqu'il atteint le caractère "*" final, il garde les 2 caractères suivants avant d'arriver en fin de chaine. Alors, il passe la variable globale "handshake" à _true_. Notons qu'on vérifie à chaque étape que le buffer ne dépasse pas sa taille allouée.
***

### Traitement de la chaine
L'arrivée d'une trame NMEA est désormais nottifiée par le passage à _true_ de la variable globale handshake. Il faut désormais détecter ce passage, et traiter la chaîne. Pour cela, le programme est bloqué dans un `while(handshake==false)`. Lorsqu'une trame est arrivée, on va copier la partie importante de la trame dans une chaine de caractères. Mais puisque `rx_buffer` est en permanence modifié par la liaison série, il est primordial de désactiver cette dernière durant la copie de la trame, puis de la réactiver.

```c
handshake = false;                                   // Clear handshake flag, will be set by ISR at end of user input
    while (handshake == false);                        // Wait here for handshake from ISR
    LPC_USART0->INTENSET &= ~RXRDY; //rx_buffer est occupé
    copier_nettoyer(rx_buffer, chaine_nettoyee);//copie et isole la ligne de rx_buffer dans chaine
    LPC_USART0->INTENSET |= RXRDY; //rx_buffer est dispo
```
La fonction `copier_nettoyer` a pour seul but de recopier `rx_buffer` dans `chaine_nettoyee`, en isolant le cœur de la chaine.

On vérifie en suivant si la trame est bien une trame de type GGA (cela aurait pu être fait avant, mais le code est plus modulable comme ceci) :
```c
if ((*(chaine_nettoyee+3)=='G')&&(*(chaine_nettoyee+4)=='G')&&(*(chaine_nettoyee+5)=='A')){ //trame GNGGA
        lcd_gohome();
        ordonner_chaine(chaine_nettoyee, chaine_heure, chaine_minute, chaine_seconde, chaine_latitude, chaine_longitude, chaine_direction_latitude, chaine_direction_longitude);
```
On fait appel à la fonction `ordonner_chaine` qui va extraire et isoler les informations utiles de la chaine. Notons que certaines informations peuvent être manquantes lorsque le module ne capte pas bien. Par exemple, l'emplacement de la latitude sera vide. La fonction fera alors passer la variable `succes_lecture` à `echec_heure`,` echec_position` ou `succes`.

L'affichage de la position et de l'heure se fera donc en conséquent.
***

## Conclusion
De manière surprenante, le module ne fournit que l'heure, mais pas la position. En regardant l'allure des trames dans la mémoire, j'ai constaté que le module ne transmet en effet jamais la position (et on notera aussi que l'heure est donnée au format d'été). Une grosse partie du projet a consisté à errer sur le plateau du Moulon au mois de décembre avec le LPC 804 à la main à la recherche d'une trame contenant une position. J'ai tout de même vérifié le fonctionnement du code en simulant une trame, en écrivant manuellement une fausse position dans la mémoire.

<img width="136" alt="Trame" src="https://user-images.githubusercontent.com/46826148/148693414-28d78402-de2d-4d75-98b6-db7733a5c908.png">


![LCD](https://user-images.githubusercontent.com/46826148/148693412-4e6e372d-d01c-45b8-ad22-c02e8b82676d.jpg)


***



