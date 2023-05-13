# turtlesim
ROS exercise

## Passos para a criação do WorkSpace & Pacote
1. `mkdir -p turtlebot_ws/src` 
2. `cd turtlebot_ws` 
3. `colcon build`
4. `source install/setup.bash`
5. `cd src`
6. `ros2 pkg create turtlebot_controller --build-type ament_python --dependencies rclpy`
7. No arquivo setup.py:
`
entry_points={
        'console_scripts': [
            'turtlebot_controller = turtlebot_controller.turtlebot_sim_controller:main'
        ],
    },
`
Após isso, deve-se voltar à pasta raiz, compilar e recarregar os pacotes do ROS2

## O código

### Descrição
O código fornecido é um script em Python que controla a navegação de um robô TurtleBot3 em um ambiente simulado usando o middleware ROS (Robot Operating System). O script lê uma lista de pontos de referência do usuário e navega o robô até cada um deles em sequência.

## Classe `TurtlebotSimController`
Essa classe herda da classe `rclpy.node.Node` fornecida pelo ROS e é responsável por controlar a navegação do robô.

### Atributos

- `position`: Instância da classe `Position` que representa a posição atual do robô.
- `setpoint`: Instância da classe `Position` que representa o próximo ponto de referência para onde o robô deve se movimentar.
- `roadmap`: Lista de nós (`MyNode`) que define a sequência de pontos de referência a serem alcançados.
- `twist_msg_`: Mensagem do tipo `Twist` utilizada para enviar comandos de movimentação ao robô.
- `publisher_`: Publicador para o tópico `/cmd_vel` responsável por enviar os comandos de velocidade ao robô.
- `subscription_`: Assinante para o tópico `/odom` que recebe informações de odometria do robô.
- `timer_`: Temporizador que chama periodicamente a função `navigate` para atualizar o movimento do robô.

### Métodos

- `navigate()`: Responsável por controlar o movimento do robô em direção ao próximo ponto de referência.
- `update_setpoint()`: Atualiza o próximo ponto de referência para o próximo nó da lista `roadmap`.
- `pose_callback(msg)`: Callback que recebe as informações de odometria do robô e atualiza a posição atual.
- `generateNodeList()`: Função que solicita ao usuário uma lista de pontos de referência e retorna a lista de nós `MyNode`.

## Classe `MyNode`
Essa classe representa um ponto de referência na forma de um nó. Cada nó possui coordenadas `x` e `y`.

## Classe `Position`
Essa classe herda da classe `Pose` fornecida pelo pacote `turtlesim.msg` e representa uma posição no espaço com coordenadas `x`, `y` e um ângulo `theta`.

## Função `main()`
A função principal do script. Inicializa o ROS, solicita ao usuário uma lista de pontos de referência, cria uma instância da classe `TurtlebotSimController` e executa o ciclo principal do ROS.
