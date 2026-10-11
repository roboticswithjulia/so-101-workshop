# L'app

L'app té tres pestanyes: **Control**, **Programa** i **Guia** (aquesta).

## Connectar

- Prem **Connectar** a dalt de tot.
- A sota surt quins motors responen. Han de ser els 6: si en falta algun, mira «Si alguna cosa falla».

## Pestanya Control

- Cada articulació té un **control lliscant**. Arrossega'l i el braç es mou.
- També pots escriure els graus a la casella i prémer **Retorn**.
- **Tornar tot a zero** porta totes les articulacions a la posició zero.

## Motors lliures

Marca **Motors lliures** i els motors es queden sense força: pots moure el braç **amb les mans**.
És la manera més fàcil de trobar una posició.

> ⚠ Agafa el braç abans de marcar-ho: sense força, cau pel seu propi pes.

Desmarca-ho i els motors tornen a tenir força i aguanten el braç on és.

## Pestanya Programa

Aquí escrius el programa del robot.

- **Afegir la posició actual**: llegeix on és el braç i escriu una línia **anar_a(...)** al programa. Combina-ho amb «Motors lliures»: mou el braç a mà i prem el botó.
- **Desar posició inici**: la posició actual passa a ser on va **inici()**.
- **Executar el programa**: fa tot el programa de cop.
- **Pas a pas**: fa un sol pas i s'espera. Molt útil per trobar errors.
- **Pausa**: s'atura on és. Prem **Executar** per continuar.
- **Aturar**: para el programa i el braç es queda on era.
- **Desar** i **Carregar**: guarden el programa en un fitxer i el tornen a obrir.
- **Exemple**: torna a posar el programa d'exemple.

> ⚠ Prova sempre un programa nou amb **Pas a pas** abans d'executar-lo sencer.
