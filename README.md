# Simulazione-di-un-sistema-a-N-corpi 

APPUNTI=

Almeno due transaltrion unit
zippare tutto insieme
consegnare anche il clang format, nella repositary del progetto
modi esperessivi di identificazione, scrivere nella stessa linga.
niente spazi e caratteri speciali nei nomi dei files, tutto in minuscolo (es:simulation.cpp) 
un unico namespace (inerente al progretto. anche pf se serve)
fare degli unit test, con Doctest ad esempio
compilare con CMake. guardare il file CMakelists.txt (cambiare # aggiungi l'eseguibile progetto.t)

i prof controllano che questi codici funzionino= 
- cmake -S . -B build -G"Ninja Multi-Config" 
- cmake --build build --config Debug 
- cmake --build build --config Debug --target test 
- cmake --build build --config Release 
- cmake --build build --config Release --target test 

fare pochi commenti, servono solo per aiutare i prof nei passaggi complicati:

RELAZIONE=
5 facciate

- titolo del progetto. 
- autore o autori del lavoro.
- data, corrispondente alle ultime modifiche al codice (giorno più, giorno meno).
- eventuali modifiche intercorse dalla consegna precedente.
- descrizione sintetica del tema del progetto, solo nel caso questo non sia uno dei temi da noi proposti.
- descrizione sintetica delle principali scelte progettuali e implementative.
- dichiarazione e giustificazione dell'eventuale uso di costrutti non introdotti a lezione.
- eventuali librerie esterne da installare con apt install.
- istruzioni dettagliate su come eseguire il programma o i programmi che fanno parte del progetto.
- descrizione precisa dei parametri di input e del formato di output, con degli esempi immediatamente usabili da noi per la valutazione.
- interpretazione dei risultati ottenuti.
- strategia di test per verificare che quanto ottenuto sia ragionevolmente esente da errori.
- dichiarazione di eventuale uso di sistemi di Intelligenza Artificiale generativa (usarla per fare i test).
- ogni altra informazione utile agli obiettivi sopra citati.

In formato pdf o markdown, in LaTeX 

MOTO DI N CORPI
parte grafica su FSML

PER FARE ANDARE LE COSE SU GITHUB SI FA 
- git pull   -> (scarichi gli aggiornamneti degli altri)
- git add    -> (aggiungi le modifiche)
- git commit -> (le salvi in modo tale che vengano mandate dopo)
- git push   -> (le mandi su git hub)
- (git status) -> (ti fa vedere che cosa aggiungi e invii) 

BOZZA PROGRAMMA=

1) ogni corpo deve essere una classe body:
- massa (intero)
- posizione (vettore) 
- velocità (vettore)   
- accellerazione (vettore)

2) i vettori li vedo o come std::vector o come struct a 2 D con x e y,
nella struct 2D dei corpi in x y dove si implementano gli operatore + - e || ||

3) implementi le funzioni fondamentali nella classe bodies 

4) serve un vettore di bodies che li contenga in modo tale che definendo la dimensione del vettore sappiamo
quanti corpi ci sono

5) fai dei cicli while per fare andare di ogni passo il moto dei corpi implementazione delle funzioni membro
nella classe 

6) funzione nella classe bodies che controlli che |v| <300'000 km/s 

7) funzione nella classe che controlli che m >= 0

8) funzione nella classe che calcola l'energia, un vettore con tutta l'energia all'interno che aumentiamo ad ogni iterazione 
dell'algoritmo e stampiamo alla fine per far vedere che l'energia è conservata
