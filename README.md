# Trekking_final — testes com notebook, D435i e ESP32 por USB

> **Atualizacao: webcam do notebook como padrao.** Comece pelo
> [guia da webcam](README_WEBCAM.md). As instrucoes abaixo documentam a versao
> RealSense, agora acessivel com `main.py --camera realsense`. O firmware USB
> e os testes manuais de motores/servo continuam os mesmos.

Projeto independente, sem ROS2. Os originais do ZIP e da pasta Trekking foram apenas lidos.
O notebook detecta o cone, mede profundidade alinhada e envia PWM dos motores e direcao
do servo. A ESP32 aplica os comandos e desliga a ponte se passar 350 ms sem comando valido.
Nao ha uma tarefa de heartbeat que continue movimento quando a visao trava.

## 1. Instalar no notebook

Abra PowerShell nesta pasta e execute:

```powershell
powershell -ExecutionPolicy Bypass -File .\instalar.ps1
```

O script cria `.venv` com Python 3.11 e instala dependencias. Nao modifica seu ambiente
de treinamento. Instale tambem o RealSense SDK/Viewer caso a D435i nao seja reconhecida.
Conecte-a por USB 3, feche o Viewer antes de executar Python e mantenha alimentacao
adequada para motor/servo, com GND comum com a ESP32. Motor e servo nao devem depender
da alimentacao USB do notebook. Confira os niveis eletricos dos encoders antes de usa-los.

## 2. Preparar Arduino IDE

Instale **esp32 by Espressif Systems 3.x** no Gerenciador de Placas. Abra
`esp32/TrekkingUSB/TrekkingUSB.ino`, selecione a sua placa ESP32 classica e porta COM.
O teste de compilacao usa **ESP32 Dev Module**, pacote **3.3.7**. Nenhuma biblioteca
externa de servo e necessaria: usa LEDC para motores a 2 kHz e servo a 50 Hz.

Confira as constantes do sketch antes de gravar:

| Funcao | GPIO |
|---|---:|
| PWM B / B IN2 / B IN1 | 32 / 33 / 25 |
| STBY | 26 |
| A IN1 / A IN2 / PWM A | 27 / 14 / 13 |
| Servo | 17 |
| SCL / SDA (reservados) | 22 / 21 |
| Encoder B1 / B2 (entradas, sem leitura nesta versao) | 16 / 4 |
| Encoder A1 / A2 (entradas, sem leitura nesta versao) | 15 / 2 |

O firmware inicia parado. Gravacao reinicia a placa e centraliza o servo.
O centro inicial **124 graus** vem de `SERVO_INITIAL_ANGLE=90` + `SERVO_OFFSET=34`
do ZIP, mas outros exemplos usam o offset de outra forma. E uma referencia a calibrar,
nao um centro mecanico confirmado. Faixa inicial limitada a **114..134 graus**.
Se necessario, desacople a articulacao para achar o centro sem forcar o mecanismo.
Depois ajuste `SERVO_CENTER_DEG`, `SERVO_MIN_DEG`, `SERVO_MAX_DEG`, `SERVO_SPAN_DEG`
e grave novamente. O sketch interpreta steering positivo como aumento do angulo.

Sentido positivo dos motores segue `pwm.c` do ZIP: A IN2 alto/IN1 baixo e
B IN1 alto/IN2 baixo. Ajuste `INVERT_A` ou `INVERT_B` se o respectivo motor girar
ao contrario. O sketch nao comanda marcha a re. A pinagem parece uma ponte com STBY,
mas o modelo do driver nao foi confirmado; confira se essa logica corresponde a ele.

## 3. Teste manual por USB, com rodas suspensas

Feche o Monitor Serial do Arduino IDE; ele disputa a porta com Python.
Liste as portas:

```powershell
.\.venv\Scripts\python.exe -m serial.tools.list_ports
```

Troque `COM5` pela sua porta. Teste primeiro so o servo, depois cada sentido
do servo e o giro dos dois motores. Cada comando dura no maximo 3 segundos:

```powershell
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering 0 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering 0.5 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --steering -0.5 --seconds 1
.\.venv\Scripts\python.exe manual_usb.py --port COM5 --throttle 0.10 --seconds 1
```

Zero PWM desabilita STBY. Uma parada nao garante frenagem mecanica instantanea;
a distancia percorrida apos o corte deve ser medida. O limite absoluto do firmware
e 25% de PWM e o limite inicial automatico e 18%. PWM e **potencia solicitada**,
nao velocidade em m/s. Encoders e PID nao foram portados para esta primeira versao.
Se o motor nao vencer o atrito em baixa potencia, ele pode parar antes do alvo;
nao foi adicionado impulso automatico para evitar saltos na aproximacao final.

## 4. Visao e distancia, sem movimentacao

```powershell
.\.venv\Scripts\python.exe main.py
```

O modo padrao nao abre a porta USB. Use um unico cone inicialmente. O programa
exige tres deteccoes consistentes para confirmar o alvo e evita trocar de alvo
durante o movimento. A caixa magenta marca a regiao usada para profundidade.
Amostras zero/invalidas e mistura ampla de profundidades sao rejeitadas.
A medida usa o percentil 20 da ROI: e uma heuristica conservadora, nao segmentacao
do cone nem uma medicao exata de sua base. Confira com trena especialmente perto.

A captura consulta os perfis anunciados pela camera e escolhe uma combinacao
RGB + profundidade compativel, preferindo profundidade de menor resolucao.
O terminal mostra a conexao USB e as resolucoes/fps realmente usadas.
Na D435i conectada como USB 2.1, depth 424x240 nao estava disponivel; o codigo
agora tambem considera 480x270 e 640x480, com 30 ou 15 fps. Os frames continuam
alinhados pelo SDK. A profundidade proxima precisa ser validada no perfil ativo.
Teste leituras em **50, 35, 30, 27 e 25 cm da camera** antes de ligar
movimento. Se a D435i nao fornecer profundidade valida perto de 25 cm, reposicione
a camera mais para tras, atualize o recuo na configuracao ou acrescente sensor
de curta distancia. Nao usamos estimativa pelo tamanho da caixa para continuar
avancando quando a profundidade falha.

`config.json` usa `cone_class=0`; os pesos copiados foram conferidos e a classe 0
e `cone`. `device="cpu"` funciona sem configurar CUDA. Se o ambiente tiver PyTorch
com CUDA e GPU compativel, pode usar `device="0"`. Imagens com idade de processamento
acima de 250 ms interrompem o movimento: nao aumente esse limite para compensar
um notebook lento; reduza a carga de inferencia ou configure GPU.

## 5. Ajustar a referencia de 15 cm

Assumimos camera olhando para frente, aproximadamente nivelada e instalada
**10 cm atras** do ponto mais a frente do robo:

```text
distancia aproximada da frente = profundidade Z da camera - 0.10 m
referencia desejada = 0.15 + 0.10 = 0.25 m da camera
corte inicial = 0.25 + margem de 0.02 = 0.27 m da camera
```

O corte inicial corresponde a cerca de 17 cm antes da inercia. A margem
`braking_margin_m=0.02` e um valor inicial para calibrar. Aproximacoes em baixa
velocidade, com trena, permitem ajustar essa margem para buscar **15 cm fisicos**.
Ela nao garante 15 cm exatos. Confira tambem a projecao da base do cone, o angulo
da camera, o ponto de referencia da frente e a latencia. A medicao e axial Z;
para camera muito inclinada ou cone lateral, a simples subtracao nao basta.

## 6. Aproximacao automatica

Depois dos testes manuais e da validacao com trena:

- Ajuste `steering_sign` para `1` ou `-1`, de modo que cone a direita resulte em
  rodas virando a direita. O ganho inicial e 0.8 e ha tolerancia perto do centro.
- Confirme sentido dos dois motores, centro/limites do servo e recuo real da camera.
- Em `config.json`, marque `hardware_calibrated` como `true`.
- Coloque o robo em uma area livre com um cone e execute:

```powershell
.\.venv\Scripts\python.exe main.py --live --port COM5
```

Ainda inicia parado. Na janela, **ESPACO** arma e inicia quando ha alvo/profundidade
validos; **S** para e **Q/Esc** encerra. Perda de alvo, ambiguidade, frame atrasado,
profundidade invalida, erro USB ou conclusao param e desarmam. Para retomar, e
necessario iniciar novamente. Desconectar USB ou travar Python faz o watchdog
da ESP32 desligar os motores em ate 350 ms do ultimo comando valido.

O robo vira com servo, portanto avanca devagar enquanto corrige a trajetoria;
nao tenta girar no proprio eixo. Cone muito lateral exige reposicionamento manual.
Nao existe navegacao entre obstaculos nesta versao. As teclas dependem da janela
estar em foco; mantenha acesso a um corte fisico de alimentacao durante os testes.

Os registros CSV ficam em `logs/`, com estado, distancia, PWM e direcao.

## Protocolo USB e origem

O protocolo de teste **T1** e textual, a 115200 baud, com CRC16-CCITT (inicial 0xffff)
e numero sequencial. Exemplos de corpos antes de `*CRC\n`:

```text
T1 1 STOP 0.0000 0.0000
T1 2 ARM 0.0000 0.0000
T1 3 DRIVE 0.1000 -0.2000
ACK T1 3 ARMED
```

ARM e STOP exigem campos zero. DRIVE exige sessao armada, sequencia crescente e
valores finitos dentro dos limites. Pacotes invalidos desarmam. O notebook exige
ACK com sequencia e estado esperados, sem retransmitir movimento antigo.

**Nao e compativel com o firmware do ZIP.** La, Python envia `<fff` para velocidades
das rodas/angulo e a estrutura de recepcao em `types.h` declara inteiros. Para
o teste independente nao reutilizamos esse contrato ambiguo nem velocidades
como se fossem PWM. Gravacao do novo sketch e necessaria antes de usar USB.

Referencias locais lidas no ZIP: `Esp32/Esp_Firmware/include/types.h`,
`src/pwm.c`, `src/initializers.c`, `src/task_core0.c`, `src/uart.c` e
`NoteEmbarcado/trekking_ws/src/robot_gazebo/src/uart_comm.py`.
Nenhum arquivo extraido foi executado ou modificado.

Fontes tecnicas: [LEDC Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html),
[alinhamento RealSense em Python](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py),
[distancia minima e qualidade D400](https://support.realsenseai.com/hc/en-us/articles/360059129453-Depth-accuracy-for-Intel-RealSense-D400-Series-Cameras).

## Verificacao de software

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

O teste nativo em `tests/native/firmware_test.cpp` executa o sketch com GPIO/Serial
simulados, cobrindo CRC, watchdog, sequencias repetidas e falhas do parser.
A compilacao real para ESP32 e o teste do modelo estao registrados em
`validation/RESULTADOS.md`. Verificacao de software nao substitui a calibracao
fisica; nenhum movimento real foi acionado na preparacao do projeto.
