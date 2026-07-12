ROS2 JAZZY

Este ficheiro contem:
    -> explicação a fundo de todos os packages
    -> parâmetros
    -> exemplos 

workspace: ros2_ws_2

Existem cinco packages:

// fes_bringup pkg //

    Criou-se um tipo de mensagem ROS2 chamado Configuration.msg

    O package motionstim8_configuration é responsável pela configuração inicial do sistema. 
    Contém um nó de configuração e a mensagem Configuration.msg, utilizada para publicar as opções selecionadas pelo utilizador, como as articulações a controlar, o modo de controlo e os ganhos PID, informações do paciente.
    Esta separação desacopla a configuração do processamento, tornando a arquitetura mais modular e facilitando a futura integração de uma interface gráfica.
   
// control_signals pkg //

    Composto por :

        -Generate_Reference -> nó que subscreve trajetórias de referência
        -Angle_Estimation -> nó que subscreve posição atual (neste momento, simulação de uma sinosoide)
    
// fes_control pkg //
    
    Calcula o PID em Posiçao ou Torque.


// motionstim8_hardware pkg //

    A classe MotionStim8Hardware implementa a Hardware Interface do ros2_control, sendo responsável por fazer a ligação entre a arquitetura ROS 2 e o estimulador MotionStim8

    Encarregue de:
         atribuir canais ás juntas. 
         se é agonista ou antagonista
         aplicar limites de saturação
    

// motionstim8_driver pkg //

        Este package implementa a interface entre o ROS2 e o estimulador MotionStim8. É responsável pela integração com o ros2_control através da classe MotionStim8Hardware, que expõe o estimulador como um dispositivo compatível com o framework.

        A comunicação com o estimulador é realizada através da biblioteca stimlib.

####################   DESENVOLVIMENTO   #######################

// fes_bringup //

    Launch: Vai executar todos os nós após o nó do motionstim8_configuration publicar /configuration.
            Para isso acontecer é usado o RegisterEventHandler OnProcesExit -> É um evento. Quando um processo termina, executa outras ações.

    Config: Ficheiro usado pelo controller_manager.
            Tem os controllers das juntas e o update_rate(frequência do ciclo do controller_manager)

    URDF:

        Parâmetros de estimulação:
            canal agonista;
            canal antagonista;
            largura de pulso máxima do agonista;
            largura de pulso máxima do antagonista;
            corrente do agonista;
            corrente do antagonista.
        
        Parâmetros gerais do hardware:
            porta série (serial_port);
            main_time;
            group_time;
            n_factor;
            simulation_mode (opcional).

    Configuration.msg
        -> tipo de mensagem 
        parâmetros:
            -joints -> (ankle_left, ankle_right, knee_left, knee_right)
            -control_mode -> (Position, Torque)
            -kp -> ganho proporcional
            -kd -> ganho derivativo
            -ki -> ganho integral
            -patient_name
            -sex
            -age
            -height
            -weight
            log_directory -> diretório onde serão armazenados os resultados do ensaio (logs/<paciente>/<data>/trialX).

    Node: configuration_node

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

    Generate_Reference
        Node: generate_reference_node

        Class : GenerateReferenceNode

            Construtor:

                Tarefa: Atribuição da junta á sua respetiva trajetória.

                        hip_left → Healthy_LH
                        knee_left → Healthy_LK
                        ankle_left → Healthy_LA
                        hip_right → Healthy_RH
                        knee_right → Healthy_RK
                        ankle_right → Healthy_RA

                Publish: /reference_joint_states -> trajetórias de referência de cada junta

                parametros:

                    index_	-> Índice do ponto atual da trajetória.
                    reference_pub_	-> Publisher do tópico /reference_joint_states.
                    trajectories_ -> Mapa que associa cada junta ao respetivo vetor de referência.
                    timer_	-> Temporizador que executa a publicação a 100 Hz(10ms).
                                
            Função : update_reference()

                Tarefa: Guarda no tópico /reference_joint_states a posiçao de referencia que vai ser continuamente atualizada de 10 em 10ms.
                        Preenche o vetor name com as 4 juntas.
                        Existe conversão de graus para rads pois o ROS2 trabalha com rads.
                        Esta informação é usada posteriormente no pkg motionstim8_controller.
                Criada msg do tipo JointStates:
                        name = [ ]
                        position = [ ]

                parametros:

                    msg  ->  Mensagem `JointState` publicada.                     
                    msg.name -> Lista das articulações.                                  
                    msg.position -> Valores de referência em radianos para cada articulação. 
                    index_ -> Determina o ponto atual da trajetória.                   
                    TrajectoryLength -> Número total de amostras da trajetória.                
                    reference_pub_ -> Publica a mensagem no tópico `/reference_joint_states`.  

                Topic: exemplo do que é guardado em /reference_joint_states

                            name
                            ---------
                            knee_left
                            ankle_right

                            position
                            ---------
                            0.319
                            -0.122

    
    Angle_Estimation

        Node: angle_estimation_node

        Class: AngleEstimation_node

            Construtor: 
                Tarefa:

                Publish: /joint_position -> Type: JointStates (para já é uma sinusoide)

            Função: timer_callback()
                Tarefa: Percorre todas as juntas e calcula a posição atual delas, através de uma sinusoide.
                        A msg depois é enviada para o tópico /joint_position.

                Cria msg do tipo JointState
                
                Nota: Antes de enviar a msg para o tópico, é associado um timestamp à mensagem através de msg.header.stamp = now(). 
                      Este regista o instante em que a posição articular foi gerada,

// fes_control //

    Plugin do ros2_control gerido pelo controller_manager e reutilizado para diferentes juntas.
        
        Class: Controller herda de ControllerInterface

            Função : on_init()
                Responsável por:
                    ->declarar os parâmetros do controlador (joint e output_scale);
                    ->inicializar as variáveis internas (desired_position_, current_position_ e command_);
                    ->preparar o controlador para a fase de configuração.

            Função : command_interface_configuration()
                Tarefa: Define quais as interfaces de comando que o controlador necessita.
                        Neste caso devolve apenas uma interface:

                            <joint>/stim_command

                        É através desta interface que o comando calculado pelo PID é enviado para a Hardware Interface(presente em motionstim8_hardware).

            Função : state_interface_configuration()
                Executada quando o controlador entra no estado Configured.

                    Responsabilidades:

                        lê o nome da articulação (joint);
                        abre o ficheiro CSV para registo dos resultados;
                        lê o parâmetro output_scale; (está no controller.yaml)
                        cria os três subscritores:
                            /reference_joint_states;
                            /joint_position;
                            /configuration; -> onde tem os ganhos e o modo de controlo
                        recebe os ganhos PID enviados pelo ConfigurationNode;
                        inicializa o controlador PID (control_toolbox::Pid);
                        prepara o controlador para iniciar o controlo.

            Função : on_configure()
            Função : on_activate()
            Função : on_deactivate()
                ->põe a 0.

            Função : update()
                Executada periodicamente pelo controller_manager (100 Hz).
                Implementa o algoritmo de controlo.

                Em cada ciclo:

                    verifica se o PID já foi inicializado;

                calcula o erro:

                    erro = referência − posição atual

                aplica o controlador PID (modo posição);
                normaliza a saída para o intervalo [-1,1];
                escreve o comando na command_interface;
                regista posição, referência, erro e saída do PID no ficheiro CSV;


// motionstim8_hardware //

        Plugin

            Class : MotionStim8Hardware herda de hardware_interface

                Função : on_init()

                    É executada quando a Hardware Interface é inicializada.

                    Responsabilidades:
                        -inicializa a classe base (SystemInterface);
                        -lê os parâmetros gerais do hardware definidos no URDF (serial_port, main_time, group_time, n_factor e simulation_mode);
                        -percorre todas as joints definidas no URDF;
                        -cria uma estrutura StimConfig para cada articulação controlada, contendo:
                        -canais agonista e antagonista;
                        -correntes;
                        -larguras de pulso máximas;
                        -armazena a configuração de cada articulação;
                        -inicializa os vetores command_ e position_.

                Função : on_configure()

                    Executada quando a Hardware Interface entra no estado Configured.

                    Responsabilidades:
                        -verifica se o sistema está em modo de simulação;
                        -estabelece a comunicação série com o MotionStim8 através do MotionStim8Driver;
                        -cria a lista de canais de estimulação;
                        -inicializa o estimulador com os parâmetros definidos no URDF;
                        -coloca a interface pronta para iniciar o funcionamento.

                Função : export_state_interfaces()

                    Exporta as interfaces de estado disponibilizadas pela Hardware Interface.

                    Neste projeto cria, para cada articulação controlada, uma interface:

                        -position

                    que pode ser utilizada pelos controladores para obter o estado da articulação.

                Função : export_command_interfaces()

                    Exporta as interfaces de comando utilizadas pelos controladores.

                    Para cada articulação cria uma interface:

                        -stim_command

                    onde os controladores escrevem o comando normalizado que será posteriormente convertido em parâmetros de estimulação.

                Função : write()

                    É executada em cada ciclo do controller_manager.

                    Implementa toda a conversão entre o comando do controlador e os parâmetros enviados ao MotionStim8.

                    Responsabilidades:

                        verifica se a Hardware Interface está configurada;
                        percorre todas as articulações;
                        lê o comando normalizado (stim_command);
                        determina se a estimulação deve ser aplicada ao músculo agonista ou antagonista;
                        calcula a largura de pulso correspondente;

                        cria os vetores:
                            pulse_width;
                            pulse_current;
                            mode;

                        em modo de simulação apenas apresenta informação de depuração;
                        em modo real envia os parâmetros para o MotionStim8 através do MotionStim8Driver.
                        
                Função : read()

                    Corresponde à função de leitura da Hardware Interface.

                    Atualmente não existe qualquer sensor ligado ao sistema, pelo que esta função apenas devolve OK, não atualizando o estado das articulações.

                Função : on_deactivate()

                    Executada quando a Hardware Interface é desativada.

                    Responsabilidades:

                        fecha a comunicação série com o MotionStim8 (modo real);
                        marca a interface como não configurada (configured_ = false).
                        
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
                    pulse_current;
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


