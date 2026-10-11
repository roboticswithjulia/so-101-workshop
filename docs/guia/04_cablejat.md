# Cablejat

Els motors es connecten **en cadena**: la placa parla amb el motor 1, el motor 1 amb el 2, i així fins a la pinça.

## La cadena

```
Placa → Motor 1 → Motor 2 → Motor 3 → Motor 4 → Motor 5 → Motor 6
Base    Espatlla  Colze     Canell    Gir       Pinça
```

1. Comprova que la font d'alimentació està **apagada**.
2. Connecta el cable de 3 pins de cada motor al motor següent.
3. Connecta el cable del **motor 1** (la base) a la placa controladora.
4. Connecta la font de **7,4 V** a la placa.
5. Connecta el cable **USB** de la placa a la Raspberry Pi.
6. Ara sí: encén la font.

> ⚠ No alimentis mai els motors des dels pins de la Raspberry Pi. Fes servir sempre la font de 7,4 V.

> ⚠ Abans de tocar cap cable, apaga la font.

## Consells

- Els cables han de quedar **ben endollats**: un cable fluix fa que un motor deixi de respondre.
- Passa els cables per dins de les guies perquè no s'enganxin quan el braç es mou.
- Deixa una mica de cable sobrant a cada articulació: si queda tens, s'arrencarà quan giri.
- Mou el braç a mà (sense corrent) i comprova que cap cable s'estira.
