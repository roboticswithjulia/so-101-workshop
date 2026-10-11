# Reptes

## Repte 1: agafa i deixa

Teniu un objecte a la posició d'**inici**, marcada a la taula. L'heu de portar a la posició de **fi**.

1. Programeu el braç perquè **agafi** l'objecte a la posició d'inici.
2. Programeu el braç perquè el **deixi** a la posició de fi.
3. Ajunteu-ho: agafar a l'inici i deixar a la fi, en un sol programa.

Pista: per agafar un objecte, el braç s'hi acosta per sobre, baixa, tanca la pinça i torna a pujar.

```
inici()
obrir_pinca()
anar_a(base=-3, espatlla=27, colze=15, canell=-7)
anar_a(base=-3, espatlla=48, colze=14, canell=-25)
tancar_pinca()
anar_a(base=-3, espatlla=27, colze=15, canell=-7)
inici()
```

> ⚠ Els números d'aquest exemple són inventats. Trobeu els vostres amb «Motors lliures» i «Afegir la posició actual».

## Repte 2: munta i desmunta blocs

Feu una torre de **3 o 4 blocs** i després desmunteu-la.

- Cada bloc va una mica **més amunt** que l'anterior: el colze i l'espatlla canvien una mica a cada pis.
- Feu-ho bloc a bloc: primer la torre amunt, després avall.

## Jenga Robòtic

La competició final! El braç ha d'agafar una peça i posar-la **a dalt de la torre** sense que caigui.

- **+1 punt** per cada peça moguda bé.
- **−2 punts** si la torre cau.
- **Variant ràpida**: guanya el primer equip que fa 3 moviments correctes.

Si hi ha empat, guanya l'equip que ho ha fet en menys temps. Si cap equip acaba, guanya qui ha mogut més peces.

Consell: moveu-vos a poc a poc a prop de la torre (**segons=2** o més) i passeu per **aixecar()** entre peça i peça.
