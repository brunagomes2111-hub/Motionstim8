ROS2 JAZZY

Nota: Embora as trajetórias de referência sejam originalmente definidas em graus, o ROS 2 utiliza radianos nas mensagens
 sensor_msgs/msg/JointState. Assim, a comunicação entre os nós é efetuada em radianos e, antes do cálculo do erro, os valores são 
 convertidos para graus para preservar o comportamento do controlador PID original.

Este ficheiro contem:
    -> explicação a fundo de todos os packages
    -> parâmetros
    -> exemplos 

workspace: ros2_ws_2

ros2_ws_2/src/
│
├── logs -> informações dos pacientes e respetivos resultados experimentais
│
├── control_signals -> package responsável pela geração das referências e estimação dos estados articulares
│   ├── Angle_Estimation
│   │   ├── AngleEstimationNode.cpp -> publica a posição atual das articulações (atualmente simulada através de uma sinusoide)
│   │   └── include
│   │       └── control_signals
│   │           └── AngleEstimationNode.hpp
│   ├── CMakeLists.txt
│   ├── Generate_Reference
│   │   ├── generate_reference_node.cpp -> publica as trajetórias de referência de posição e de torque
│   │   ├── include
│   │   │   └── control_signals
│   │   │       ├── generate_reference_node.hpp
│   │   │       ├── position_trajectories.hpp -> cabeçalho das trajetórias de posição
│   │   │       └── torque_trajectories.hpp -> cabeçalho das trajetórias de torque
│   │   ├── position_trajectories.cpp -> implementação das trajetórias de posição
│   │   └── torque_trajectories.cpp -> implementação das trajetórias de torque
│   ├── package.xml
│   ├── src
│   │   └── control_signals_node.cpp -> executa os nós de geração de referências e estimação
│   └── Torque_Estimation
│       ├── include
│       │   └── control_signals
│       │       └── torque_estimation_node.hpp
│       └── torque_estimation_node.cpp -> publica/estima o torque articular
│
├── fes_bringup -> package responsável pela configuração do sistema e lançamento da arquitetura
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── controller_gains.yaml -> ficheiro de ganhos PID (atualmente não utilizado)
│   │   └── controllers.yaml -> configuração dos controladores do ros2_control
│   ├── include
│   │   └── fes_bringup
│   │       └── configuration_node.hpp
│   ├── launch
│   │   ├── fes_configuration.launch.py -> launch do ConfigurationNode
│   │   └── motionstim8.launch.py -> launch da restante arquitetura
│   ├── msg
│   │   └── Configuration.msg -> mensagem utilizada para configurar o sistema
│   ├── package.xml
│   ├── src
│   │   └── configuration_node.cpp -> solicita ao utilizador as configurações, cria os diretórios dos ensaios e publica a configuração
│   └── urdf
│       └── motionstim8.urdf -> descrição do hardware e parâmetros de estimulação
│
├── fes_control -> package responsável pelo algoritmo de controlo
│   ├── CMakeLists.txt
│   ├── controller_plugin.xml -> plugin do ros2_control
│   ├── include
│   │   └── fes_control
│   │       └── Controller.hpp
│   ├── package.xml
│   └── src
│       └── fes_controller.cpp -> implementação do controlador PID, anti-windup e registo dos resultados
│
├── motionstim8_driver -> package responsável pela comunicação de baixo nível com o MotionStim8 através da stimlib
│   ├── CMakeLists.txt
│   ├── include
│   │   └── motionstim8_driver
│   │       └── motionstim8_driver.hpp
│   ├── package.xml
│   ├── src
│   │   └── motionstim8_driver.cpp -> implementação da comunicação série com o estimulador
│   └── third_party
│       └── stimlib -> biblioteca fornecida pelo fabricante do MotionStim8
│           ├── doc.cfg
│           ├── global_list_example.txt
│           ├── mainfiledoc.doc
│           ├── motionstim8.cpp
│           ├── motionstim8.h
│           ├── serial_linux.cpp
│           ├── serial_linux.h
│           ├── serialport.h
│           └── single_pulse_example.txt
│
└── motionstim8_hardware -> package responsável pela interface entre o ros2_control e o MotionStim8
    ├── CMakeLists.txt
    ├── include
    │   └── motionstim8_hardware
    │       └── motionstim8_hardware.hpp
    ├── motionstim8_hardware_plugin.xml -> plugin da Hardware Interface
    ├── package.xml
    └── src
        └── motionstim8_hardware.cpp -> converte os comandos do controlador em parâmetros de estimulação, aplica limites de saturação e envia-os ao MotionStim8Driver


    Configuration Node         AngleEstimationNode
        │                              |
        ▼                              ▼
    /configuration              /joint_position
        │                              |
        ▼                              ▼
    FES Controller               FES Controller 
        │
    stim_command                GenerateReferenceNode
        │                              |
        ▼                              ▼
    MotionStim8Hardware         /reference_joint_state
        │                              |
        ▼                              ▼
    MotionStim8Driver             FES Controller 
        │
        ▼
    MotionStim8
    

É constituído por cinco packages:

    -control_signals
    -fes_bringup
    -fes_control
    -motionstim8_hardware
    -motionstim8_driver

// fes_bringup pkg //

    Foi criado um tipo de mensagem ROS 2 denominado Configuration.msg.

    O package fes_bringup é responsável pela configuração inicial do sistema e pelo lançamento da arquitetura.

    Contém o ConfigurationNode e a mensagem Configuration.msg, utilizada para publicar as opções selecionadas pelo utilizador, tais como:
        - articulações a controlar;
        - modo de controlo;
        - ganhos PID;
        - informações do paciente;
        - diretório onde serão armazenados os resultados experimentais.
        -coativação
        - PW modulation ou PA modulation

    O ConfigurationNode é ainda responsável por criar automaticamente a estrutura de diretórios dos trials e por ativar os controladores das articulações selecionadas através do controller_manager.


// control_signals pkg //

    É composto por três módulos:

        - Generate_Reference → gera e publica as trajetórias de referência de posição e de torque. As trajetórias de posição são armazenadas em graus e convertidas para radianos antes de serem publicadas.

        - Angle_Estimation → publica a posição atual das articulações (atualmente simulada através de uma sinusoide).

        - Torque_Estimation → publica ou estima o torque articular (atualmente simulada através de uma sinusoide).

// fes_control pkg //

    Implementa o controlador do sistema como um plugin do ros2_control.

    O controlador recebe a referência e a medição da articulação, converte os valores de posição de radianos para graus e calcula o erro antes de aplicar o controlador PID.

    Atualmente encontra-se implementado o modo de controlo em posição e em torque.

    O PID utiliza uma estratégia de anti-windup por integração condicional e a sua saída é limitada ao intervalo [-1,1] antes de ser enviada para a Hardware Interface.

// motionstim8_hardware pkg //

    A classe MotionStim8Hardware implementa a Hardware Interface do ros2_control, sendo responsável por fazer a ligação entre a arquitetura ROS 2 e o estimulador MotionStim8.

    É responsável por:

        - exportar as state_interfaces e command_interfaces;
        - receber os comandos normalizados do controlador no intervalo [-1,1] e convertê-los em parâmetros de estimulação;
        - determinar se a estimulação deve ser aplicada ao músculo responsável pela extensão ou pela flexão da articulação;
        - suportar modulação por Pulse Width (PW) ou Pulse Amplitude (PA);
        - calcular a coativação
        - registar os parâmetros de estimulação num ficheiro CSV.
        - aplicar os limites de saturação;
        - enviar os parâmetros de estimulação ao MotionStim8Driver.


// motionstim8_driver pkg //

    Este package implementa a comunicação de baixo nível com o estimulador MotionStim8.

    É responsável por:

        - estabelecer a comunicação série com o estimulador;
        - inicializar o MotionStim8;
        - enviar os parâmetros de estimulação;
        - terminar corretamente a comunicação quando necessário.

    A comunicação com o estimulador é realizada através da biblioteca stimlib.

####################   DESENVOLVIMENTO   #######################


   // fes_bringup //

    Launch

        Executa o ConfigurationNode e, após este publicar a configuração, inicia automaticamente os restantes nós através do evento RegisterEventHandler(OnProcessExit).

    Config

        Ficheiros utilizados pelo controller_manager, contendo a configuração dos controladores e os ganhos PID.

    URDF

        Contém a configuração do MotionStim8, incluindo:

            - canais de estimulação;
            - correntes;
            - larguras de pulso máximas;
            - parâmetros de coativação;
            - parâmetros gerais do hardware (serial_port, main_time, group_time, n_factor e simulation_mode).

    Configuration.msg

        Mensagem ROS 2 utilizada para configurar o sistema.

        Inclui:
            - articulações;
            - modo de controlo (Position/Torque);
            - ganhos PID;
            - ativação/desativação da coativação;
            - modo de modulação (PW ou PA);
            - informações do paciente;
            - diretório dos resultados experimentais.

    Node: ConfigurationNode

        Responsável por solicitar ao utilizador a configuração do ensaio, criar a estrutura de diretórios para guardar os resultados experimentais, publicar a mensagem /configuration e ativar os controladores das articulações selecionadas através do controller_manager.

        Class: ConfigurationNode

            Função: configureSystem()

                ->pede ao utilizador as configurações
                ->verifica se o paciente já existe;
                ->cria automaticamente a estrutura de diretórios;
                ->cria patient_info.txt;
                ->cria experiment_info.txt;
                ->publica a configuração.
            
            Função: activateSelectedControllers()

                            Tarefa: Diz ao controller_manager quais os controladores(juntas) que devem passar do estado inactive para active.
                                    Devolve bool.
                                    Adiciona controladores ás juntas que estão no selected_joints_(juntas que foram selecionadas)
                            Client of Service: switch_client_ (configuration_node)
                            Service: /controller_manager/switch_controller
                            Pedido: é uma mensagem do tipo SwitchController::Request

                            Exemplo do pedido:

                                SwitchController Request

                                    activate_controllers

                                    - knee_left_controller
                                    - ankle_right_controller

                            NOTA: o controller_manager vai tratar o pedido como BEST_EFFORT



// control_signals //

    Package responsável pela geração das referências e pela estimação dos estados articulares utilizados pelo controlador.

    Generate_Reference

        Node: generate_reference_node

        Class: GenerateReferenceNode

            Construtor

                Inicializa as trajetórias de referência de cada articulação, os publishers e o temporizador de publicação.

                Publica:
                    - /reference_joint_states (referências de posição)
                    - /reference_torque (referências de torque)

            Função: update_reference()

                Atualiza e publica, a 100 Hz, as trajetórias de referência de posição e de torque das articulações selecionadas.

               As referências de posição são convertidas de graus para radianos antes de serem publicadas, de acordo com a convenção do ROS 2.

    Angle_Estimation

        Node: angle_estimation_node

        Class: AngleEstimationNode

            Construtor

                Inicializa o publisher do tópico /joint_position.

            Função: timer_callback()

                Calcula e publica a posição atual das articulações.

                Atualmente, a posição é simulada através de uma sinusoide, sendo posteriormente utilizada pelo controlador.

    Torque_Estimation

    Node: torque_estimation_node

    Class: TorqueEstimationNode

        Construtor

            Inicializa o publisher do tópico /joint_torque.

        Função: timer_callback()

            Calcula e publica o torque estimado das articulações no tópico /joint_torques, sendo esta informação utilizada pelo controlador no modo de controlo em torque.

// fes_control //

    Plugin do ros2_control gerido pelo controller_manager, responsável pelo controlo de cada articulação.

    Class: Controller (herda de ControllerInterface)

        Função: on_init()

            Declara os parâmetros do controlador e inicializa as variáveis necessárias ao funcionamento do algoritmo de controlo.

        Função: command_interface_configuration()

            Define a interface de comando:

                <joint>/stim_command

            através da qual o comando calculado é enviado para a Hardware Interface.

        Função: state_interface_configuration()

            Define a interface de estado utilizada pelo controlador:

                <joint>/position

        Função: on_configure()

            Inicializa o controlador.

            Responsabilidades:

                - subscreve o tópico /configuration;
                - seleciona o modo de controlo (Position ou Torque);
                - cria os subscritores das referências e medições correspondentes ao modo selecionado;
                - recebe os ganhos PID;
                - inicializa o controlador PID com os ganhos recebidos e configura o anti-windup por integração condicional;
                - cria o ficheiro CSV para registo dos resultados experimentais.

        Função: on_activate()

            Ativa o controlador.

        Função: on_deactivate()

            Coloca o comando de estimulação a zero.

        Função: update()

            Executada periodicamente pelo controller_manager (100 Hz).

            Em cada ciclo:

                - calcula o erro entre a referência e a medição;
                - calcula o comando através do controlador PID;
                - limita a saída ao intervalo [-1,1];
                - escreve o comando na interface <joint>/stim_command;
                - regista os resultados experimentais no ficheiro CSV.


// motionstim8_hardware //

    Plugin do ros2_control responsável pela interface entre o controlador e o estimulador MotionStim8.

        Class: MotionStim8Hardware (herda de SystemInterface)

            Função: on_init()

                Inicializa a Hardware Interface.

                Responsabilidades:

                    - inicializa a classe base (SystemInterface);
                    - lê os parâmetros gerais do hardware definidos no URDF;
                    - cria a configuração de estimulação de cada articulação;
                    - inicializa os vetores de comandos e estados.
                    

            Função: on_configure()

                Configura a Hardware Interface.

                Responsabilidades:

                    - recebe a configuração do sistema através do tópico /configuration;
                    - recebe a posição atual das articulações através do tópico /joint_position;
                    - estabelece a comunicação série com o MotionStim8 (modo real);
                    - inicializa o estimulador;
                    - prepara a Hardware Interface para funcionamento.
                    - recebe o modo de controlo e o modo de modulação através de /configuration;
                    - recebe a posição atual das articulações através de /joint_position;
                    - inicializa o ficheiro de registo da estimulação.

            Função: export_state_interfaces()

                Exporta a interface de estado:

                    - <joint>/position

            Função: export_command_interfaces()

                Exporta a interface de comando:

                    - <joint>/stim_command

            Função: write()

            Executada periodicamente pelo controller_manager (100 Hz).

            Em cada ciclo:

                - lê o comando normalizado enviado pelo controlador e limita-o ao intervalo [-1,1];
                - determina se a estimulação é aplicada ao músculo agonista ou antagonista;
                - no modo PW, converte o comando normalizado em Pulse Width;
                - no modo PA, converte o comando normalizado em Pulse Amplitude;
                - calcula a coativação com base na posição articular quando o modo de controlo é Position;
                - aplica os pesos definidos para o PID e para a coativação;
                - aplica os limites máximos de Pulse Width ou Pulse Amplitude;
                - cria os vetores de Pulse Width, Pulse Amplitude e modo de estimulação;
                - em modo de simulação, calcula os parâmetros sem os enviar ao estimulador;
                - em modo real, envia os parâmetros para o MotionStim8 através do MotionStim8Driver;
                - regista os parâmetros de estimulação no ficheiro CSV.

            Função: read()

                Atualiza o estado das articulações.

                Atualmente não existem sensores ligados ao sistema, pelo que esta função apenas devolve OK.

            Função: on_deactivate()

                Termina a comunicação com o MotionStim8 e desativa a Hardware Interface.
                        
                Variáveis membro principais
                    
                    driver_ ->	Interface de comunicação com o MotionStim8.
                    joint_names_ ->	Lista das articulações controladas.
                    stim_configs_-> Configuração de estimulação de cada articulação (canais, correntes e PW máximas).
                    command_ ->	Comandos de estimulação recebidos dos controladores.
                    position_ -> Vetor das posições articulares.
                    serial_port_ ->	Porta série utilizada para comunicar com o estimulador.
                    main_time_	-> Parâmetro de inicialização do MotionStim8.
                    group_time_	-> Parâmetro de inicialização do MotionStim8.
                    n_factor_	-> Fator de frequência utilizado pelo estimulador.
                    simulation_mode_ -> Indica se o hardware funciona em modo de simulação.
                    configured_	-> Indica se a Hardware Interface foi corretamente configurada e inicializada.
                    coactivation_enabled_ -> Indica se a coativação está ativada.
                    control_mode_ -> Modo de controlo selecionado (Position ou Torque).
                    modulation_mode_ -> Modo de modulação selecionado (PW ou PA).
                    joint_position_ -> Posições articulares utilizadas no cálculo da coativação.

// motionstim8_driver //

        Class : MotionStim8Driver()

            Construtor da classe.

                Cria uma instância do driver, sem estabelecer qualquer ligação ao estimulador.

            Função : connect()

                Estabelece a comunicação com o MotionStim8.

                Responsabilidades:

                    procura dispositivos série disponíveis através da biblioteca stimlib;
                    percorre todas as portas encontradas;
                    tenta abrir cada porta série;
                    configura a comunicação série;
                    caso a ligação seja estabelecida com sucesso, marca o driver como ligado (connected_ = true).

                Devolve:

                    true caso a ligação seja estabelecida;
                    false caso não seja possível comunicar com o dispositivo.

            Função : initialize()

                Inicializa o MotionStim8.

                Recebe como parâmetros:

                    lista de canais de estimulação;
                    main_time;
                    group_time;
                    n_factor.

                Antes da inicialização são validados:
                    - frequência máxima de estimulação (500 Hz);
                    - relação entre main_time e group_time;
                    - número de canais suportado.

                Esta função invoca a função Send_Init_Param() da stimlib, responsável por configurar o estimulador para o modo de funcionamento pretendido.

                Caso a inicialização falhe, a ligação é considerada inválida e a função devolve false.

            Função : sendUpdate()

                Envia novos parâmetros de estimulação para o MotionStim8.

                Recebe três vetores:

                    pulse_width;
                    pulse_amplitude;
                    mode.

                Estes parâmetros são enviados ao estimulador através da função Send_Update_Parameter() da stimlib.

                Se ocorrer algum erro durante a transmissão, o driver encerra a ligação ao dispositivo.

            Função : isConnected()

                Indica o estado da ligação ao MotionStim8.

                Devolve:

                    true se existir comunicação ativa;
                    false caso contrário.

            Função : disconnect()

                Encerra a comunicação com o MotionStim8.

                Responsabilidades:

                    atualiza o estado interno (connected_ = false);
                    Para a estimulação com Send_Stop_Signal()
                    fecha a porta série através da função Close_serial() da stimlib.

            Variáveis membro principais
             
                stim_	-> Instância da biblioteca stimlib, utilizada para comunicar diretamente com o MotionStim8.
                connected_	-> Indica se existe uma ligação ativa com o estimulador.

            Antes do envio são verificadas as gamas de segurança:

                - Pulse Width: [0,500] µs;
                - Pulse Amplitude: [0,127] mA.


