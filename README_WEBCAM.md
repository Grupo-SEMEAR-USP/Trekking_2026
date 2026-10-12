# Testes com a webcam do notebook

`main.py` agora usa webcam por padrao; nao importa nem abre RealSense.
Use a `.venv` ja instalada. Se precisar reinstalar, `instalar.ps1` instala apenas
as dependencias da webcam. A captura usa OpenCV, pede 640x480/30 fps e informa
a resolucao real devolvida pelo driver. A captura continua em uma thread, com
buffer de apenas um frame, para a inferencia nao acumular imagens antigas.

## Primeiro teste: apenas abrir a imagem

Desconecte a RealSense para ela nao aparecer como uma das webcams. Feche Camera
do Windows, Viewer, Teams e programas que estejam usando a webcam. No PowerShell,
dentro de Trekking_final:

```powershell
.\.venv\Scripts\python.exe testar_webcam.py
```

Q/Esc fecha. Este programa nao usa YOLO, nao abre serial nem salva imagens.
Se nao abrir, tente o outro backend; se abrir uma camera diferente, altere o indice:

```powershell
.\.venv\Scripts\python.exe testar_webcam.py --backend msmf
.\.venv\Scripts\python.exe testar_webcam.py --source 1
```

Indice 0 corresponde a camera padrao enumerada pelo backend, que pode mudar
quando uma camera USB e conectada. Os indices nao identificam permanentemente
o equipamento. No Windows, habilite acesso a camera para aplicativos da area
de trabalho. Nenhum driver da RealSense e necessario para a webcam integrada.

## Segundo teste: detectar cone

```powershell
.\.venv\Scripts\python.exe main.py
```

Use um cone inteiro no enquadramento. A imagem mostra caixa, estado e comando
calculado de servo. Sem calibracao, mostra `SEM_DISTANCIA_VALIDA` e PWM zero.
S para; Q/Esc sai. O modo visual nao abre a ESP32. Se precisou de outro indice
ou backend no primeiro teste, passe os mesmos argumentos para `main.py`.

## Calibrar distancia aproximada

Webcam nao tem profundidade. O programa aproxima a distancia pela relacao
`distancia = K / altura_da_caixa_em_pixels`. Para obter K:

1. Fixe a camera e coloque o cone inteiro, em pe, a **50 cm da lente da webcam**,
   medidos com trena. Use o mesmo cone dos testes e mantenha a camera nivelada.
2. Execute:

```powershell
.\.venv\Scripts\python.exe main.py --calibrate-distance 0.50
```

3. Mantenha cone e camera parados. Aguarde ao menos 20 deteccoes estaveis e
   pressione **C**, com a janela em foco. O terminal confirma o salvamento de
   `webcam_calibration.json`. Q fecha.
4. Rode `main.py` novamente. A tela indicara **frente ESTIMADA**. Confira com
   trena em varias distancias, incluindo a referencia final de parada.

A referencia pode ser de 0.30 a 2 metros: passe o valor efetivamente medido.
A calibracao e invalidada ao mudar resolucao, indice, backend ou modelo YOLO.
Recalibre tambem ao trocar camera, cone, zoom/foco ou posicionamento da camera.
Cone cortado pelas bordas nao fornece distancia valida e interrompe movimento.
Mudancas na caixa YOLO, inclinacao e foco causam erro. Nao e uma medida fisica
precisa e nao garante parada exata a 15 cm. Se o cone nao couber na imagem nessa
distancia, recue a camera ou use um sensor de distancia para a aproximacao final.

## Antes de mover o robo

A webcam integrada do notebook precisa estar **fixa ao robo**, olhando para a
trajetoria, para que a direcao e a distancia tenham relacao com o movimento.
Um notebook em uma mesa serve para o teste visual, mas nao para guiar o robo.

`config_webcam.json` e a configuracao usada por padrao. Ela e independente da
antiga `config.json` da RealSense. O recuo inicial da webcam e **0 m**, sem supor
que ela esteja nos mesmos 10 cm da D435i. Meça da lente ate a frente do robo e
atualize `camera_recess_m` (positivo quando a camera esta atras da frente).

Para buscar 15 cm entre frente e cone:

```text
referencia da lente = camera_recess_m + 0.15 m
corte inicial = referencia da lente + braking_margin_m
```

A margem inicial e 2 cm e o PWM maximo automatico e 12%. Verifique centro/limites
do servo, sentido dos motores e direcao usando `manual_usb.py` com rodas suspensas,
conforme o guia do firmware no README original. O sketch da ESP32 nao mudou.

Depois da montagem, calibracao e verificacao da distancia com trena, marque
`hardware_calibrated` e `webcam_distance_validated` como `true`. Rode:

```powershell
.\.venv\Scripts\python.exe main.py --live --port COM5
```

Troque COM5 pela porta da ESP32. Inicia parado; ESPACO inicia apenas com alvo e
distancia validos. S para e Q sai. Perda da captura/alvo ou distancia invalida
para e desarma. O watchdog do firmware permanece em 350 ms. Ajuste a margem de
frenagem por testes curtos em area livre. Nao ha desvio de obstaculos nesta versao.

## Manter RealSense como opcao

O codigo da RealSense foi preservado para retomada futura:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-realsense.txt
.\.venv\Scripts\python.exe main.py --camera realsense
```

Referencias de captura: [OpenCV VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
e [backends de video](https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html).
