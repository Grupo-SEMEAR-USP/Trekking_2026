# Correções do robô — ROS 2 e ESP32

A implementação integrada recebe o centro de um cone, avança em curva e para ao centralizá-lo horizontalmente. Usa a câmera para corrigir a direção, sem consultar a odometria. Os arquivos corrigidos serão aplicados à pasta Trekking_2026; o ZIP de correções contém os mesmos arquivos, com caminhos relativos à raiz.

## O que mudou

- Centralizador instalado no pacote ROS 2: pixels recentes, limite de esterçamento, tolerância, três imagens para confirmar e histerese para evitar oscilações.
- Visão publica um único alvo em `cone_center`, com timestamp anterior à inferência. Mantém associação local entre imagens; não identifica de forma persistente os quatro cones.
- Corrigido o recorte da profundidade com índices inteiros e a instalação do modelo OpenVINO. Pixels e coordenadas ópticas em metros usam tópicos distintos.
- Conversor Ackermann mantém as velocidades das rodas compatíveis com o ângulo realmente permitido. Não tenta girar com velocidade linear zero.
- ROS 2 e UART passam a usar `VelocityCommand` com Header; rodas em rad/s e direção equivalente das rodas em radianos.
- UART preserva fragmentos, recupera sincronismo, valida comandos, reconecta e exige telemetria. Pacotes com a mesma amostra do controle não renovam a validade da telemetria nem geram odometria duplicada.
- ESP passa a inicializar as duas tarefas, aplicar direção e LED, medir encoders e parar independentemente da recepção UART quando o comando expira.
- Corrigidos índice do ganho derivativo, conversão de tempo do PID, liberação de memória do wrapper e inicialização do servo.
- Odometria converte micrômetros para metros e publica velocidades. Continua sujeita à derrapagem na grama e não é usada pelo centralizador.
- Associação câmera/LiDAR usa TF e os índices reais do scan, com um executável funcional. Não envia metas automaticamente; depende das transformações reais dos sensores.
- Criado `real_robot.launch.py`, sem iniciar Nav2, Gazebo ou teleop junto do centralizador.

## Configuração confirmada e valores provisórios

| Item | Valor |
|---|---|
| LED | GPIO 23 |
| Servo reto | 34° |
| Limites do servo | 0° a 68° |
| Sentido | Aumentar o ângulo vira à direita |
| Velocidade de centralização | 0,10 m/s, provisória |
| Distância entre eixos | 0,30 m, precisa ser medida |
| Bitola traseira / raio da roda | 0,195 m / 0,0475 m, herdados do firmware |
| Contagem dos encoders | 1320 ticks por volta, herdada; confirmar quadratura e redução |
| Limite equivalente de direção | ±0,35 rad, provisório |
| Relação servo/rodas | 57,2958 graus de servo por radiano de roda, provisória |
| LED ativo | Nível alto, confirmar ligação |

O servo usa `34 - ganho * direção_ROS`. Um cone à direita gera direção ROS negativa e ângulo do servo acima de 34°. O limite provisório de ±0,35 rad usa aproximadamente 14° a 54° do servo; não precisa atingir os extremos mecânicos para testar.

A relação entre o servo e o ângulo das rodas não foi fornecida e não pode ser deduzida apenas de 0°/34°/68°. Meça-a antes de confiar no raio de curva. Ganhos PID e polaridade de motores/encoders também exigem ensaio.

Edite `NoteEmbarcado/trekking_ws/src/robot_gazebo/config/real_robot.yaml` e `Esp32/Esp_Firmware/include/hardware_config.h`. A geometria herdada e os ganhos PID estão em `include/types.h`. Mantenha o entre-eixos igual nos dois nós ROS e os limites compatíveis com o firmware. Os parâmetros ROS são lidos no início: reinicie os nós após alterar a configuração.

## Instalação no computador embarcado

Comandos abaixo são para Linux com ROS 2 Jazzy instalado. Há referência a Jazzy nos arquivos de build fornecidos; confirme que é a distribuição do computador real.

Na pasta `NoteEmbarcado/trekking_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
python -m pip install -r src/robot_gazebo/requirements-vision.txt
colcon build --packages-select robot_interfaces robot_gazebo --cmake-clean-cache
source install/setup.bash
colcon test --packages-select robot_gazebo
colcon test-result --verbose
```

As bibliotecas de visão devem estar disponíveis para a arquitetura do computador. A versão de Ultralytics é a registrada no modelo fornecido; os runtimes OpenVINO e RealSense ainda precisam ser homologados nele. Não carreguei nem executei o modelo nesta máquina.

Recompile `robot_interfaces` e todos os consumidores de `VelocityCommand`, porque a mensagem recebeu Header. Inicie os terminais com o novo workspace carregado. O código novo do computador e o firmware novo devem ser usados juntos.

Na pasta `Esp32/Esp_Firmware`, compile com:

```bash
pio run
```

A configuração fixa a plataforma localmente encontrada, `espressif32@7.1.3`, com ESP-IDF 6.1. A gravação na placa não foi executada nesta entrega.

## Teste sem acionar o robô

Para conferir só a saída do centralizador, sem iniciar a ponte UART:

```bash
ros2 run robot_gazebo centraliza_cone.py --ros-args -r cmd_vel:=cone_centering_test_cmd
```

Em outros terminais com o mesmo ambiente:

```bash
ros2 topic echo /cone_centering_test_cmd
ros2 topic pub --rate 15 /cone_center geometry_msgs/msg/PointStamped \
  "{header: auto, point: {x: 340.0, y: 120.0, z: 0.0}}"
```

Em uma imagem de 424×240: x=340 deve produzir giro negativo (direita); x=80, positivo (esquerda); x=211,5 deve produzir parada. Encerre um publicador antes de começar outro. Ao interromper os pixels, a saída deve zerar por timeout.

## Execução integrada

Após compilar/gravar a ESP e conferir direção, encoders e parada com as rodas suspensas:

```bash
ros2 launch robot_gazebo real_robot.launch.py port:=/dev/ttyACM0
```

A porta pode ser outra, por exemplo `/dev/ttyUSB0`. A imagem configurada é 424×240 a 15 fps; confirme que a RealSense oferece esse perfil. Se mudar a resolução, altere tanto `vision` quanto `centraliza_cone`. Use imagem frontal não espelhada.

Para receber pixels de outro detector, use `camera:=false`. Isso desliga a visão deste launch, mas mantém o controle habilitado. Para testar apenas a visão/telemetria sem o centralizador, use `centering:=false`; mantenha outros publicadores de `cmd_vel` desligados.

A cadeia é:

```text
cone_center -> centraliza_cone -> cmd_vel -> twist2ackermann
            -> velocity_command -> uart_comm -> ESP -> motores/servo
```

Só um controlador deve publicar em `cmd_vel` durante o ensaio.

O LED é comandado separadamente por `led_command` (Bool). Ver uma área amarela ou centralizar o cone não significa chegar a ele. Para testar o LED com o robô parado e a ponte ativa:

```bash
ros2 topic pub --once /led_command std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /led_command std_msgs/msg/Bool "{data: false}"
```

## Paradas e protocolo

A detecção expira após 0,4 s; o conversor expira `cmd_vel` após 0,3 s; a ponte expira `VelocityCommand` após 0,3 s e exige telemetria nova em até 0,5 s. A ESP zera tração e LED quando fica mais de 250 ms sem comando UART válido. Os prazos não representam uma latência total garantida: incluem períodos de execução e comunicação. Um comando novo pode retomar o movimento; não há botão de rearme por software.

Frames little-endian:

- Computador → ESP: AA + 3 float32 (roda esquerda, roda direita, direção em rad) + uint8 LED + soma dos 13 bytes do payload módulo 256 + BB. Total: 16 bytes.
- ESP → computador: AA + 3 int32 (x em µm, y em µm, orientação em mrad) + uint32 tempo em ms + soma dos 16 bytes do payload módulo 256 + BB. Total: 19 bytes.

A direção enviada não é o valor em graus do servo. Logs da aplicação ESP ficam desativados na UART0 para não misturar texto com os dados. O parser tolera ruído de inicialização.

## Validação realizada

- 27 testes automatizados passaram: centralização, sinais esquerda/direita, frames repetidos, atrasos, perda de câmera, Ackermann saturado, layout e fragmentação UART, odometria e conversão de unidades.
- Testes podem ser repetidos sem ROS: `python3 -m unittest discover -s src/robot_gazebo/test -v`, a partir do workspace.
- A compilação PlatformIO foi tentada. CMake falhou antes de compilar o projeto, no teste mínimo do compilador: erro Windows de CreateProcess/realocação de DLL; também houve acesso negado ao objeto de runtime MSYS. Portanto não há firmware compilado validado nesta entrega.
- ROS 2, câmera, LiDAR, serial e motores não foram executados neste Windows.

## O que continua pendente

A missão dos quatro cones ainda precisa de controle de distância, confirmação de chegada, piscar o LED uma vez por visita, identificação/troca do alvo, busca do próximo cone e prevenção de colisões. Centralizar não aproxima até uma distância definida: este nó para assim que alinha e pode voltar a corrigir se o alvo sair da tolerância.

No campo, a ordem horária é uma estratégia possível, mas requer definir a posição inicial e o critério para escolher o próximo cone. Um giro temporizado ou uma distância baseada só nos encoders não garante a ordem na grama.

O launch real não depende de Nav2 nem do URDF incompleto. Corrigi referências de pacote/mapa no launch legado, mas Nav2/Gazebo continuam fora da validação: faltam mapa, dimensões/TF medidos e revisão dos comportamentos de recuperação (incluindo giro parado do template). A fusão com LiDAR também depende dessas medições. Os arquivos ROS 1 da Jetson não foram migrados.

## Backup

`correcoes_trekking.zip` contém os arquivos novos e modificados, relativos à raiz de Trekking_2026. `originais_antes_das_correcoes.zip` guarda os arquivos anteriores e um manifesto que identifica os arquivos que não existiam. Não extraia o backup sobre alterações futuras sem conferir esse manifesto.

