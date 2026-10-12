# Margarete =) - RMA 2026 

Pacote reunido em 12/10/2026. O projeto executavel esta em `Trekking_final`.
O computador detecta o cone com YOLO e envia comandos por USB para a ESP32.
A ESP32 controla dois motores e o servo de direcao. Nao precisa de ROS.

## Arquivos principais

- `Trekking_final/main.py`: deteccao e aproximacao automatica.
- `Trekking_final/models/best.pt`: modelo YOLO ja presente no projeto.
- `Trekking_final/control.py`: calcula direcao e potencia de aproximacao.
- `Trekking_final/esp32/TrekkingUSB/TrekkingUSB.ino`: firmware para gravar na ESP32.
- `Trekking_final/config_webcam.json`: configuracao padrao para webcam.
- `Trekking_final/config.json`: configuracao alternativa para RealSense.
- `Trekking_final/manual_usb.py`: teste separado de servo e motores.
- `Trekking_final/README_WEBCAM.md`: calibracao e uso da webcam.
- `Trekking_final/README.md`: firmware, protocolo e opcao RealSense.

## Pinagem informada

| Placa | Funcao | Pino |
|---|---|---|
| ESP32 | PWM B | GPIO 32 |
| ESP32 | B IN 2 | GPIO 33 |
| ESP32 | B IN 1 | GPIO 25 |
| ESP32 | STBY | GPIO 26 |
| ESP32 | A IN 1 | GPIO 27 |
| ESP32 | A IN 2 | GPIO 14 |
| ESP32 | PWM A | GPIO 13 |
| ESP32 | SCL | GPIO 22 |
| ESP32 | SDA | GPIO 21 |
| ESP32 | Servo | GPIO 17 |
| ESP32 | Encoder B 1 / B 2 | GPIO 16 / 4 |
| ESP32 | Encoder A 1 / A 2 | GPIO 15 / 2 |
| Raspberry Pi / Jetson | SDA / SCL | pinos fisicos 3 / 5 |
| Raspberry Pi / Jetson | BIG LED | pino fisico 32 |

Os numeros do conector Raspberry Pi/Jetson acima foram registrados conforme
informados; confirme o modelo da placa antes de configurar o LED.
Nesta versao, comunicacao e por USB serial. I2C, LED e encoders nao sao usados.
Nao ha controle de velocidade por encoder; throttle representa PWM.

## Direcao Ackermann

Centro do servo: **34 graus**. Esquerda: **0 graus**. Direita: **68 graus**.
O firmware calcula `angulo_servo = 34 + 34 * steering`, limitado a 0..68.
Esses sao angulos do servo, nao medidas do angulo das rodas.
Cone a direita gera steering positivo; cone a esquerda gera negativo.
O robo avanca enquanto esterça, reduzindo PWM perto do alvo.
Confira sentido mecanico, pulsos e polaridade dos motores com rodas suspensas.

## Como iniciar no Windows

Abra PowerShell na subpasta `Trekking_final`:

```powershell
powershell -ExecutionPolicy Bypass -File .\instalar.ps1
.\.venv\Scripts\python.exe main.py
```

Esse primeiro comando de execucao abre somente visao, sem mover o robo.
Para testar o servo, grave o firmware atualizado pela Arduino IDE (ESP32 core 3.x),
feche o Monitor Serial e substitua COM5 pela porta real:

```powershell
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering 0 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering -0.2 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering 0.2 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --throttle 0.10 --seconds 1
```

Para webcam, calibre com o cone inteiro a 50 cm da lente, medidos com trena:

```powershell
.\.venv\Scripts\python.exe main.py --calibrate-distance 0.50
```

Aguarde 20 deteccoes estaveis e pressione C para salvar. Confira a estimativa
em varias distancias. Ajuste `camera_recess_m` com o recuo real da camera.
Depois dos testes fisicos, marque `hardware_calibrated` e
`webcam_distance_validated` como `true` em `config_webcam.json`.

```powershell
.\.venv\Scripts\python.exe main.py --live --port COM5
```

ESPACO inicia; S para; Q/Esc encerra. A janela precisa estar em foco.
A referencia inicial de parada e 15 cm da frente, com margem de 2 cm,
mas a webcam estima distancia pelo tamanho da caixa: valide com trena.
Perda breve de alvo interrompe motores e permite recuperacao do mesmo alvo
em ate 0.6 s. Falhas de distancia, imagens atrasadas e perdas prolongadas
exigem reinicio manual. A ESP32 corta motores apos 350 ms sem comando valido.
Use alimentacao apropriada de motor/servo e GND comum.

## Estado de verificacao desta entrega

37 testes Python aprovados em 12/10/2026. Acrescentada verificacao nativa
dos pulsos para centro e extremos do servo. A execucao dessa verificacao
ficou pendente: o compilador MinGW instalado falhou ao iniciar por erro de DLL.
O firmware alterado nao foi recompilado para a placa nem gravado nesta entrega.
Nenhum comando de movimento foi enviado ao robo.
Os resultados mais antigos em `validation/RESULTADOS.md` sao historicos.

O ambiente `.venv` deve ser criado nesta nova pasta. A calibracao da webcam
deve ser feita nesta instalacao. Este pacote controla aproximacao em caminho
livre; nao inclui desvio de obstaculos ou navegacao entre varios cones.
