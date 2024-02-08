# TG em carro autônomo

## Pré-requisitos

## Como rodar o projeto

Primeiro, execute o container com o comando abaixo.

```sh
docker compose up --force-recreate --build -d
```

Agora que o container foi devidamente configurado, você pode acessa-lo usando o comando abaixo.

```sh
docker exec -it tg_workspace-sim-1 /bin/bash
```

Finalmente, para executar o simulador rode.

```sh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Se todos os passos até aqui deram certo, o simulador deve estar rodando. Agora, basta usar um novo terminal do container para rodar os pacotes de controle. 

## Criando novos pacotes

Pacotes criados para controlar o carrinho na simulação devem ser criados na pasta `packages`. Para criar um novo pacote, basta navegar até essa pasta e, no terminal do container, rodar o comando

```sh
ros2 pkg create --build-type ament_python NOME_DO_PACOTE
```

Para que o pacote funcione corretamente, devem ser instaladas as suas dependências usando o comando abaixo na pasta do seu pacote

```sh
rosdep install -i --from-path . --rosdistro humble -y
```

Finalmente, o último passo antes de rodar seu pacote é builda-lo. Para isso, rode o comando abaixo

```sh
colcon build --packages-select NOME_DO_PACOTE
```

depois dar source nos scripts de instalação

```sh
source install/setup.bash
```

Uma vez criado e buildado o pacote, para rodar basta usar o comando `run` do ros.

```sh
ros2 run NOME_DO_PACOTE NOME_DO_SCRIPT
```

