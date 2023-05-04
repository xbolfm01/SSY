# SSY
<h1> Bezdrátová LWM sieť </h1> 

<h3> Popis projektu </h3>
Cieľom projektu bolo vytvroriť bezdrôtovo senzorovú sieť s protokolom LWM. Sieť má jedného koordinátora, ktorý je pripojený k PC pomocou UART a jeho úlohou je prideľovať adresy ostatným zariadeniam, ktoré sa budú chcieť do siete pripojiť, ďalej je jeho úlohou vyčítať dáta zo senzorov, ktoré sú umiestnené na jednotlivých kitoch. 

<h3> Popis kódu a popis riešenia </h3>
<h4> TYPES </h4>
Na začiatku kódu v main.c, v časti DEFINES sú uvedené parametre, ktoré definujú typ jednotlivých zariadení, definujú endpointy pre (network, RSSI), taktiež sú v tejto časti definované typy správ, ktoré v komunikácii môžu nastať - žiadosť o pridelenie adresy, odpoveď od koordinátora, potvrdzovacia správa a chybná správa. Ďalej sa v tejto časti definovali vzdialenosti dvoch Routerov od koordinátora, ktoré sa natvrdo zadali do programu ako #DEFINE DISTANCE 1 a #DEFINE DISTANCE 2. 

<h4> DEFINES </h4>
V tejto časti sú definované jednotlivé štruktúry správ a stavov. 

<h4> PROTOTYPES </h4>
V tejto časti sú zadané hlavičky jednotlivých funkcií, ktoré sa používajú. Taktiež sa tu nachádzajú aj metódy, ktoré sú určené len (pomocou if defined) pre koordinátora alebo pre Router alebo pre koncové zariadenie. V tejto časti boli pridané funkcie appSendRSSI(odosielanie RSSI) a appRSSIConfirmation (potvrdenie o prijatí RSSI). 
<i> appSendRSSI </i>
Funkcia sa stará a odoslanie RSSI. Je v nej uvedený RSSI ENDPOINT (4), adresa, na ktorú sa RSSI posiela (posiela sa broadcastovo na adresu 0xFFFF). Správy sú označované ID (sec_ID), ktoré sa zvyšuje po každom odoslaní.

<h4> Proces fungovania programu </h4>
Správy, ktoré odosielajú koncové zariadenia a routere, sú smerované na koordinátora, ktorý tieto správy spracováva. V tomto programe bol hlavný zámer o pracovanie s údajom RSSI. 

<h5> Práca s RSSI </h5>
Informáciu o prijatom RSSI od zariadenia vypíše kooridnátor hláškou do konzole. O tento výpis sa stará metóda appRSSIConfirmation. 

<h5> Útlmový člen </h5>
Následne pomocou <i>#if define</i> koordinátor najskor kontroluje pomocou funkcie <i>existujuca_adresa</i> či údaje od zariadenia s danou adresou nemá uloženú v poli, pokial nemá, tak adresu uloží. Následne príjma 5 ďalších hodnôt RSSI od zariadenia. Tieto hodnoty sa zapisujú do poľa <i>pole_prijatych_RSSI[4]</i>, následne sa tieto hodnoty zoradia pomocou qsort a pokial ich má týchto hodnot aspon 5, tak z nich vypocita medián, respektíve zoberie hodnotu na indexe [2], keďže sa jedná a nepárny počet prvkov. 
Následne sa pomocou funkcie zoslabovaci_clen (éta) spočíta útlmový člen. Funkcia <i> zoslabovaci_clen </i> pracuje s dvomi hodnotami RSSI, ktoré sa získajú od uzlov, ktoré sú umiestnené v známych vzdialenostiach (DISTANCE_1 a DISTANCE_2). Zo vzorca sa vyjadrí výpočet éty ako rozdiel daných hodnot RSSI podelený desatnásobkom logaritmu, pričom v argumente logaritmu bude podiel vzdialeností (DISTANCE_1 a DISTANCE_2) dvoch známych zariadení, od  ktorých prijímame RSSI. V mojom konkrétnom prípade vyšla éta približne 4,65. Výpočet je ukázaný na priloženej fotografii. Pri volaní tejto funkcie v kóde sa ako paramtere predávajú údaje z poľa, ktoré drží informácie o prijatých RSSI.  

<h5> Prepočet z RSSI na vzdialenosť </h5>
Následne sa pomocou známej hodnoty zoslabovacieho člena može spočítat vzdialenost medzi koordinátorom a daným zariadením. Toto rieši funkcia <i>
calculateDistanceFromRSSI </i>. Funkcia pracuje so známymi hodnotami rssi (RSSIx) a vzdialenostou DISTANCE1 (dx), to znamená, že sa jedná o parametre pre zariadenie, ktorého vzdialenosť od koordinátora poznáme. Následne parameter rssi_2 (RSSId) predstavuje prijatú hodnotu RSSI od zariadenia, ktorého vzdialenosť počítame. Po dosadení do vzorca (obrázok 2) vyšlo, že zariadenie sa nachádza približne 1,82m od koordinátora, čo odpovedalo realite. Pri volaní tejto funkcie v kóde, sa používa definovaná vzdialenosť DISTANCE_1, ktorá predstavuje známu vzdialenosť, následne sa z poľa <i> pole_ulozenych_adres </i> vyberie RSSI pre známe zariadenia označené ako PRIMAR. Potom sa vyberie ďalší prijatý údaj a použje už známa vypočítaná hodnota zoslabovacieho člena. 

<h5> Príjimaný formát dát </h5>
Pomocou funkcie <i>appUartSendMessageHR</i> je zabezpečený čitatelný formát príjmaných dát. 

<h5> Ukážky z Putty </h5>
![Pridelenie adresy dalsiemu pripojenemu zariadeniu](https://user-images.githubusercontent.com/60688750/236337279-817e39aa-17f4-4202-b2f2-08a8c464cd77.png)

