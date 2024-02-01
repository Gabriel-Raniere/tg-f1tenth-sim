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

## Rodando outros pacotes

Para abrir um novo terminal no container será necessário rodar o commando 

```sh
docker run -it tg_container
```

