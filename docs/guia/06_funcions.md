# Les funcions

Un programa és una llista d'ordres. El robot les fa **d'una en una, de dalt a baix**.
Pots copiar cada exemple al teu programa amb el botó **Copiar al programa**.

## anar_a()

Mou el braç a una posició. Escriu només les articulacions que vols moure: les altres es queden on són.

```
anar_a(base=20, espatlla=30, colze=10)
```

Les articulacions són **base**, **espatlla**, **colze**, **canell** i **gir**.

## obrir_pinca() i tancar_pinca()

Obren i tanquen la pinça. Entre parèntesis pots posar quants graus.

```
obrir_pinca()
tancar_pinca(-20)
```

## inici()

Torna a la posició inici. Comença i acaba sempre els programes amb **inici()**.

```
inici()
```

## aixecar()

Va a una posició intermèdia, amb el braç amunt. Va bé per no xocar amb res entre dos punts.

```
aixecar()
```

## esperar()

S'espera uns segons sense moure's.

```
esperar(2)
```

## segons=

Totes les funcions accepten **segons=** per decidir quant s'espera abans del pas següent. Si el braç encara es mou quan comença el pas següent, posa-hi més segons.

```
anar_a(base=-30, espatlla=40, segons=2)
```
