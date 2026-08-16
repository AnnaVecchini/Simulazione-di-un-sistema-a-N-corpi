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
- git commit -m "messaggio" -> (le salvi in modo tale che vengano mandate dopo)
- git push   -> (le mandi su git hub)
- (git status) -> (ti fa vedere che cosa aggiungi e invii) 

BOZZA PROGRAMMA=

1) ogni corpo deve essere una classe body:
- massa (intero)
- posizione (vettore) 
- velocità (vettore)   
- accellerazione (vettore)

2) creo delle struct per i vettori a 2 D con x e y, e li implemento per posizione, velocità e accellerazione
nella struct 2D dei corpi in x y dove si implementano gli operatore + - e || ||

4) implementi le funzioni fondamentali nella classe bodies 

5) serve un vettore di bodies che li contenga in modo tale che definendo la dimensione del vettore sappiamo
quanti corpi ci sono

6) fai dei cicli while per fare andare di ogni passo il moto dei corpi implementazione delle funzioni membro
nella classe 

7) funzione nella classe bodies che controlli che |v| <300'000 km/s 

8) funzione nella classe che controlli che m >= 0

9) funzione nella classe che calcola l'energia, un vettore con tutta l'energia all'interno che aumentiamo ad ogni iterazione 
dell'algoritmo e stampiamo alla fine per far vedere che l'energia è conservata

10) fuori dalla calsse introdurre una costante ϵ=10^-12 (da mettere nell'accellerazione) e tutte le costanti della fisica che ci possono servire


# Teoria C++ usata nel progetto N-Body

Questo file riassume, in ordine cronologico rispetto a come li abbiamo introdotti,
tutti i concetti di C++ usati per costruire la simulazione, e spiega perché il
codice è stato suddiviso nei file che trovi nel progetto.

---

## 1. Data abstraction: `struct` per `TDvec`

`TDvec` è un semplice contenitore di due `double` (`x`, `y`), senza invarianti da
proteggere (qualunque coppia di numeri è una posizione/velocità/accelerazione
valida). Per questo è uno `struct` con membri pubblici, non una `class`: non
c'è nulla da nascondere.

## 2. Operator overloading

Per poter scrivere `a + b`, `a - b`, `a * s` con oggetti `TDvec` invece di dover
richiamare funzioni con nomi espliciti, abbiamo definito gli operatori come
**funzioni libere**:

```cpp
TDvec operator+(TDvec const& a, TDvec const& b) { return {a.x + b.x, a.y + b.y}; }
```

Abbiamo definito **due versioni** di `operator*` (`TDvec * double` e
`double * TDvec`) perché il C++ non rende automaticamente commutativo un
operatore: `a * s` e `s * a` sono due chiamate di funzione distinte, quindi
vanno dichiarate entrambe se si vuole poter scrivere la moltiplicazione in
entrambi gli ordini (es. `0.5 * a` nell'equazione di Velocity Verlet).

## 3. Incapsulamento: quando usare getter/setter e quando no

Nella classe `Body`:

- `m_` è **privata**, con un solo getter `m()` e **nessun setter**: la massa di
  un corpo non deve poter cambiare dopo la costruzione, quindi la protezione
  ha senso.
- `r`, `v`, `a` sono **pubblici**: un getter/setter che si limita a restituire
  o assegnare il valore, senza nessuna logica aggiuntiva, non protegge nulla e
  aggiunge solo codice inutile. Regola pratica: *"prefer a struct with public
  data members over a class with trivial getters/setters"*.

## 4. Costruttore: dichiarazione dentro, definizione fuori dalla classe

```cpp
class Body {
  ...
  Body(double m, TDvec r, TDvec v, TDvec a);  // dichiarazione
};

Body::Body(double m, TDvec r, TDvec v, TDvec a) : m_{m}, r{r}, v{v}, a{a} {
  ...  // definizione, con lo scope operator ::
}
```

La **member initialization list** (`: m_{m}, r{r}, ...`) inizializza i membri
nell'ordine in cui sono dichiarati nella classe, prima che il corpo del
costruttore venga eseguito.

## 5. Eccezioni: `throw` e class invariant

Il costruttore di `Body` verifica due condizioni fisiche (massa positiva,
velocità inferiore a quella della luce) e lancia `std::invalid_argument` se
non sono rispettate. `readBodiesFromFile` lancia invece `std::runtime_error`
se il file non può essere aperto — un errore diverso, non legato alla
validità di un singolo argomento ma a una risorsa esterna (il file) non
disponibile.

In `main.cpp`, un blocco `try { ... } catch (std::exception const& e) { ... }
catch (...) { ... }` cattura qualunque eccezione lanciata durante la
simulazione, stampa il messaggio (`e.what()`) e restituisce `EXIT_FAILURE`.

## 6. `const` e riferimenti: evitare copie inutili

Funzioni come `computeEnergy` prendono `std::vector<Body> const&`: un
riferimento costante, per **leggere** i dati senza copiarli e senza poterli
modificare per errore. `computeAccelerations` e `step`, che invece devono
modificare i corpi, prendono `std::vector<Body>&` (riferimento non costante).

## 7. Ciclo `for` con `if` invece di `continue`

Nel doppio ciclo di `computeAccelerations`, il caso `j == i` (un corpo non
esercita forza su se stesso) viene escluso racchiudendo tutto il corpo del
ciclo in un `if (j != i) { ... }`, invece di usare `continue`.

## 8. `const` vs `constexpr`, e `inline`

- `const`: il valore non cambia dopo l'inizializzazione, ma può essere noto
  solo a runtime.
- `constexpr`: implica `const`, **e in più** garantisce che il valore sia
  calcolabile dal compilatore in fase di compilazione. `G`, `eps`, `dt`,
  `c_light` sono tutti valori noti a priori, quindi `constexpr`.
- `inline` (davanti a `constexpr double G` e a `double t`): dato che
  `nbody.hpp` viene incluso da più file `.cpp` (translation unit diverse),
  serve a garantire che esista **un solo oggetto** condiviso in tutto il
  programma, invece di una copia separata per ogni file che include l'header.

## 9. Namespace

Tutte le entità del progetto (`TDvec`, `Body`, le funzioni, le costanti) sono
racchiuse in `namespace pf { ... }`, per evitare collisioni di nomi con altre
librerie o altre parti di un progetto più grande, seguendo la convenzione
usata nell'esempio del corso.

## 10. Compilation model: header (`.hpp`) e source (`.cpp`)

- **`nbody.hpp`** contiene l'**interfaccia**: dichiarazioni di funzioni,
  definizione delle classi con dichiarazioni dei metodi, costanti.
- **`nbody.cpp`** contiene l'**implementazione**: le definizioni vere e
  proprie di funzioni e metodi.
- **Include guard** (`#ifndef PF_NBODY_HPP` / `#define ...` / `#endif`)
  impedisce che l'header venga incluso più volte nella stessa translation
  unit, cosa che causerebbe una violazione della One-Definition Rule (ODR).
- Le funzioni **definite direttamente nell'header** (gli operatori di
  `TDvec`, `norm`) sono marcate `inline`, perché altrimenti, essendo incluse
  in più file `.cpp`, ne esisterebbero più definizioni identiche nel
  programma finale — cosa concessa dallo standard solo se sono `inline`.

## 11. File I/O: `std::ifstream`

`readBodiesFromFile` apre il file con `std::ifstream input{filename};` e
verifica l'apertura con `if (!input)` (uno stream si converte implicitamente
in un valore "falso" se è in stato di errore). La lettura vera e propria usa
lo stesso pattern visto per leggere una sequenza di numeri da uno stream:

```cpp
while (input >> m >> x >> y >> vx >> vy) {
  bodies.push_back(Body{m, TDvec{x, y}, TDvec{vx, vy}, TDvec{0., 0.}});
}
```

Il ciclo continua finché la lettura di tutti e 5 i valori ha successo; si
ferma automaticamente alla fine del file (o se il formato è malformato).

## 12. Unit testing con Doctest

`nbody.test.cpp` è un file **a parte**, con un proprio `main()` generato
automaticamente dalla macro `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` (per questo
non si può includere nello stesso eseguibile di `main.cpp`: avremmo due
`main()`, violazione della ODR). Ogni `TEST_CASE` verifica un comportamento
isolato (es. un `Body` con massa negativa deve lanciare un'eccezione) tramite
le macro `CHECK`, `REQUIRE`, `CHECK_THROWS_AS`. Da notare: le macro non sono
funzioni C++ normali — se l'espressione contiene virgole non protette da
parentesi (es. un'inizializzazione con più argomenti), va racchiusa in un
paio di parentesi extra, altrimenti il preprocessore la interpreta come più
argomenti separati della macro.

## 13b. Tolleranza "relativa a una scala tipica" invece che al valore iniziale

Per `isEnergyConserved` la tolleranza è relativa al valore iniziale
(`|E-E0| <= tolerance*|E0|`), il che funziona bene perché l'energia
meccanica del sistema non è mai (vicina a) zero. Per la quantità di moto e
il momento angolare questo approccio non va bene: nel caso Figure-8
entrambe le grandezze sono **esattamente zero** per simmetria (puoi
verificarlo: $\vec v_3=-(\vec v_1+\vec v_2)$ esattamente), quindi una
tolleranza relativa al valore iniziale sarebbe sempre zero, e qualunque
minima oscillazione numerica farebbe fallire il controllo.

Per questo `isMomentumConserved` e `isAngularMomentumConserved` calcolano
invece una **scala caratteristica** del sistema (rispettivamente
$\sum_i m_i|\vec v_i|$ e $\sum_i m_i|\vec r_i||\vec v_i|$, una stima
dell'ordine di grandezza tipico di quella quantità per i singoli corpi) e
confrontano la deviazione con quella, invece che con il valore iniziale
della grandezza totale.

## 13c. Riepilogo delle grandezze conservate calcolate

| Funzione | Cosa calcola |
|---|---|
| `computeEnergy` | $E = K + U$ |
| `computeMomentum` | $\vec P = \sum_i m_i \vec v_i$ |
| `computeAngularMomentum` | $L_z = \sum_i m_i(x_i v_{y,i} - y_i v_{x,i})$ |

## 13. Build system: CMake

Il progetto usa `CMakeLists.txt` per definire due eseguibili distinti:

- `n_body`: la simulazione vera e propria (`main.cpp` + `nbody.cpp`)
- `n_body.t`: i test (`nbody.test.cpp` + `nbody.cpp`), registrato con
  `add_test` in modo da poter essere lanciato con `ctest`/`cmake --build
  build --target test`

In modalità Debug vengono attivati anche l'**address sanitizer** e
l'**undefined-behaviour sanitizer**, due strumenti che rilevano a runtime
errori come accessi fuori dai limiti di un array o comportamento non
definito, utili per scoprire bug che altrimenti passerebbero inosservati.

---

## Perché questa suddivisione in file

| File | Ruolo |
|---|---|
| `nbody.hpp` | Interfaccia pubblica del componente: cosa espone, non come è implementato |
| `nbody.cpp` | Implementazione: come funziona davvero ogni pezzo |
| `main.cpp` | Un solo eseguibile "consumatore" dell'interfaccia: la simulazione |
| `nbody.test.cpp` | Un secondo eseguibile "consumatore": verifica automatica della correttezza |
| `initial_conditions.txt` | Dati, separati dal codice: si possono cambiare le condizioni iniziali senza ricompilare |
| `CMakeLists.txt` | Istruzioni per costruire entrambi gli eseguibili e registrare i test |

L'idea di fondo è la **separazione fra interfaccia e implementazione**: chi
usa `nbody.hpp` (sia `main.cpp` che `nbody.test.cpp`) sa *cosa* può fare con
`Body`, `step`, `computeEnergy`, ecc., senza dover conoscere *come* sono
implementate internamente. Se in futuro cambiassimo l'implementazione (es.
un algoritmo di integrazione diverso) senza cambiare le firme delle funzioni,
nè `main.cpp` nè `nbody.test.cpp` dovrebbero essere modificati.
