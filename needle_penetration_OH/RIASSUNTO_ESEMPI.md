# QUICK OVERVIEW DEGLI ESEMPI

## Quick Haptics
- SPONGYCOW:elastica e deformabile



## High Level Haptics (HL)
**GRAFICAL EXAMPLES**
- **CONSTRAINTS**: definire "PATH forzato" a cui ancorare il tip
- **EVENTS**: esempi di callback (movimento, touch oggetto (e posizione touch), click bottone)
- **HL_DOP**: pop a 2 livelli in un punto **ALTA PRIORITA'**
- **SIMPLE RIGID BODY DYNAMICS**: tocchi un oggetto e l'oggetto si sposta
- **SIMPLE DEFORMABLE SURFACE**: toccare una superficie deformabile (si sente il reticolato, per irvin è la *friction e la stiffness* il problema, per Mirco sono i valori di *spring e damping*) **ALTA PRIORITA'**
- **SIMPLE SPHERE**: frizione sulla sfera **MEDIA PRIORITA'**
- **SHAPE MANIPULATION**: utile per il riposizionamento della camera tramite mouse

**CONSOLE**
- **CUSTOM FORCE EFFECT**: movimento viscoso in un liquido
- **CANNED FORCE EFFECT**: stick-slip friction nello spazio **ALTA PRIORITA'**
- **CUSTOM SHAPE**: sfera con frizione dentro e fuori. contact and constraint (spingendo in pratica entri nella sfera) **ALTA PRIORITA'**
- **EFFECT ATTRIBUTES**: diversi effetti di frizione, viscosità, molle nello spazio **MEDIA PRIORITA'**



## Low Level Haptics (HD):
**GRAFICAL EXAMPLES**
- **SLIDING CONTACT**: collisione tra 2 sfere  **ALTA PRIORITA'**

**CONSOLE**
- **ANCHORED SPRING**: attiva molla se premi il bottone
- **ERROR HANDLING**: cattura problemi legati a impulsi di forze o timeout
- **PREVENT WARM MOTORS**: riporta la temperatura e previene il surriscaldamento (controllare il codice)



### NOTA BENE
- Metti sempre il menù con il quit con il tasto destro

### TO DO
- Vedere se è possibile modificare i *parametri dinamici* della vacca (in realtà pare di sì)
