🦅 F-16 Flight Control System (FCS) & Simulink-FlightGear Integration

Questo progetto implementa una simulazione 6-DOF (Six Degrees of Freedom) non lineare di un caccia F-16 Fighting Falcon, interfacciando un modello matematico in MATLAB/Simulink con il motore grafico di FlightGear per la visualizzazione in tempo reale tramite input hardware (Joystick/HOTAS).
🚀 Panoramica del Progetto

L'F-16 è celebre per essere stato il primo caccia progettato con Stabilità Statica Rilassata (Relaxed Static Stability). Il suo baricentro è posizionato artificialmente all'indietro per massimizzare l'agilità in combattimento.
Questo rende l'aereo (il "Plant" o "Modello Nudo") intrinsecamente e fisicamente inguidabile da un essere umano senza l'ausilio di un computer di bordo.

L'obiettivo di questo progetto è progettare, testare e validare i sistemi di controllo del volo (FCS) necessari per stabilizzare il velivolo e permettere il pilotaggio, affrontando la natura fortemente accoppiata delle sue dinamiche attraverso un'Analisi MIMO (Multiple-Input Multiple-Output).
📐 Analisi MIMO dell'F-16

L'F-16 non può essere trattato come un insieme di sistemi isolati. Ogni input influenza molteplici output (es. il rollio induce imbardata avversa). Il sistema è definito da:

Vettore di Input (Comandi):

    Thrust (T): Spinta del motore (da 1000 a 60000 lbf).

    Elevator (δe​): Elevatore per il controllo del beccheggio (max ≈±25∘ o 0.4 rad).

    Aileron (δa​): Alettoni per il controllo del rollio.

    Rudder (δr​): Timone per il controllo dell'imbardata.

Vettore di Output/Stati (Sensori):

    Velocità (V,U,W)

    velocità angolari (p,q,r)

    Angoli di Eulero (ϕ,θ,ψ)

    angolo di attacco e di slide(alfa, beta)

    Altitudine (h)

🧠 Strategie di Controllo (FCS)

Per domare l'instabilità, il progetto divide il controllo in due canali principali, utilizzando tre diverse architetture di controllo per fini di ricerca e comparazione:
1. Controllo Longitudinale (Beccheggio / Pitch)

Il canale longitudinale è il più critico a causa del polo a parte reale positiva (instabilità) del modello a catena aperta. L'obiettivo è tracciare l'angolo di attacco (α) o il pitch rate (q), prevenendo lo stallo profondo.
2. Controllo Latero-Direzionale (Rollio e Imbardata / Roll & Yaw)

Unisce il controllo degli alettoni e del timone per coordinare le virate, minimizzando l'angolo di derapata (β) e gestendo l'accoppiamento cinematico (Dutch Roll).
Algoritmi Implementati:

    PID (Proportional-Integral-Derivative): L'approccio classico. Richiede il disaccoppiamento forzato del sistema MIMO in loop SISO (Single-Input Single-Output). Utile come baseline, ma limitato nelle manovre ad alto angolo di attacco.

    LQR (Linear Quadratic Regulator): Un controllore a feedback di stato ottimo. Gestisce perfettamente la natura MIMO dell'F-16, bilanciando in modo elegante la reattività dell'aereo con il consumo di energia degli attuatori, calcolando una matrice di guadagno K su un modello linearizzato attorno a un punto di trim (es. Mach 0.8 a 5000 ft).

    MPC (Model Predictive Control): Lo stato dell'arte. Guarda a un orizzonte temporale futuro per ottimizzare la traiettoria e, soprattutto, gestisce i vincoli fisici in modo nativo. Evita matematicamente che i comandi superino l'escursione massima delle alette (es. satura a 0.4 rad) o la spinta massima del motore, prevenendo crash numerici e stalli aerodinamici causati da comandi pilota troppo bruschi.

🔌 Setup dell'Interfaccia Hardware / Software

La simulazione in tempo reale ("Hardware-In-The-Loop" simulato) richiede il corretto instradamento dei segnali tra Joystick, Simulink e FlightGear.
Multiplexing dei Segnali (Cruciale)

Il modello Simulink (il blocco di volo) si aspetta che i segnali in ingresso siano rigorosamente vettorizzati in questo ordine prima di entrare nel Plant:

    Thrust (Spinta)

    Elevator (Elevatore - In radianti)

    Aileron (Alettoni - In radianti)

    Rudder (Timone - In radianti)

Collegamento con FlightGear

Per far sì che FlightGear diventi un puro "schermo" visivo senza interferire con la sofisticata fisica calcolata da Simulink, il suo motore aerodinamico interno (FDM) deve essere disabilitato.

Stringa di avvio per FlightGear:
Inserire nelle opzioni aggiuntive (Additional Options):
Plaintext

--fdm=null --native-fdm=socket,in,60,,5502,udp

    --fdm=null: Spegne la fisica interna di FlightGear.

    --native-fdm=...: Mette il simulatore in ascolto sulla porta UDP 5502, a 60Hz, in attesa dei pacchetti posizionali generati da Simulink.

🛠️ Come avviare la Simulazione

    Inizializzazione: Eseguire lo script MATLAB principale (es. init.m o setup.m) per caricare le costanti, la geometria, i coefficienti aerodinamici e calcolare il punto di Trim (altitudine iniziale e velocità in piedi/secondo).

    Avvio di FlightGear: Lanciare FlightGear con le opzioni di rete configurate come sopra. Attendere il caricamento dello scenario.

    Check Hardware: Assicurarsi che l'HOTAS sia collegato e centrato (un input asimmetrico al tempo T=0 su un velivolo instabile causa errori numerici istantanei).

    Avvio Simulink: Premere PLAY su Simulink. Il sistema inizierà a calcolare le equazioni di stato e trasmetterà le coordinate a FlightGear.
