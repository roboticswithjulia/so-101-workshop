# Si alguna cosa falla

No passa res: als enginyers de robòtica també els falla tot sovint. Busqueu el problema pas a pas.

## Cap motor respon

- La font de 7,4 V està encesa?
- El cable USB està connectat a la Raspberry Pi i a la placa?
- El cable entre la placa i el motor 1 està ben endollat?
- Prem **Connectar** una altra vegada.

## Un motor no respon

- Mira el cable que arriba a aquest motor i el que en surt: un d'ells està fluix.
- Si el motor 4 no respon, normalment tampoc ho fan el 5 i el 6: el problema és abans del 4.

> ⚠ Apaga la font abans de tocar cap cable.

## El braç tremola o fa soroll

- Potser està fent massa força: comprova que no xoca amb res.
- Baixa la **Velocitat** a la pestanya Control.
- Si el motor s'escalfa, atura't i avisa la instructora.

## Error al programa

- Llegeix el missatge de sota: diu a quina **línia** és l'error.
- Has escrit bé el nom de la funció? És **obrir_pinca**, sense ç.
- Has tancat tots els parèntesis?
- Els números decimals van amb **punt**, no amb coma: **2.5**, no 2,5.
- Les línies de dins d'un **for** van amb 4 espais al davant.

## El braç no va on vull

- Fes servir **Pas a pas** per veure quin pas falla.
- Torna a capturar la posició amb **Motors lliures** i **Afegir la posició actual**.
- Si el braç passa per on no toca, afegeix un pas intermedi o un **aixecar()**.

## El vídeo no s'obre

- Avisa la instructora: potser falta VLC o el fitxer del vídeo.
