# Coneix el robot

El SO-101 té **6 motors**. Cada motor mou una **articulació**, com les del teu braç.
Cada motor té un número (**ID**) de l'1 al 6.

- **1 · Base**: gira tot el braç a dreta i esquerra.
- **2 · Espatlla**: aixeca i abaixa el braç.
- **3 · Colze**: doblega el braç pel mig.
- **4 · Canell**: mou la mà amunt i avall.
- **5 · Gir del canell**: fa girar la mà.
- **6 · Pinça**: obre i tanca els dits.

## Els motors

- Són motors **Feetech STS3215**. No són motors normals: són **intel·ligents**.
- Cada motor sap en quina posició és i ho pot explicar a l'ordinador.
- Tots estan connectats en **cadena** amb el mateix cable, com els llums de Nadal.
- Per això cada motor necessita un ID diferent: així l'ordinador sap amb qui parla.

## Els graus

- Les posicions es mesuren en **graus** respecte de la **posició zero**.
- La posició zero és el braç estirat en una posició neutra.
- **0** vol dir «a la posició zero». Un número positiu gira cap a un costat i un de negatiu, cap a l'altre.
- Exemple: **base=30** vol dir «gira la base 30 graus».
