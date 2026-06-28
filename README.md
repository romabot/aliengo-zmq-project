# Aliengo MuJoCo + ZeroMQ PD controller

Проект показывает внешний обмен между MuJoCo-симуляцией и ПД-контроллером робота Aliengo.

```text
MuJoCo simulator  -- state bytes -->  ZeroMQ  -->  Robust controller
MuJoCo simulator  <-- command bytes -- ZeroMQ  <--  Robust controller
```

## Что реализовано

- MuJoCo-сцена с Aliengo из `robot.xml`.
- Quasi-2D ограничение: фиксируются боковое смещение `Y` и `yaw`.
- Внешний контроллер в отдельном Python-процессе.
- Обмен через ZeroMQ: `PUB/SUB`, два TCP-канала.
- ПД-регулятор суставов: `tau = KP * (q_des - q) - KD * qd`.
- Передача состояния: время, dt, состояние базы, углы/скорости суставов, bias-компоненты.
- Передача команды: 12 моментов моторов + 2 стабилизирующих момента корпуса.
- CSV-лог и построение графиков.

## Структура

```text
aliengo_zmq_project_v2/
├── robot.xml
├── requirements.txt
├── README.md
├── run_controller.sh
├── run_simulation.sh
├── logs/
└── src/
    ├── common/
    │   ├── constants.py
    │   ├── kinematics.py
    │   └── protocol.py
    ├── controllers/
    │   └── robust_controller_zmq.py  # внутри только PD, имя оставлено для совместимости запуска
    ├── simulation/
    │   └── aliengo_sim_zmq.py
    └── tools/
        └── plot_logs.py
```

## Важно про mesh-файлы

В архив включен `robot.xml`, но в нем есть ссылки на mesh-файлы:

```text
meshes/meshes\trunk.obj
meshes/meshes\hip.obj
meshes/meshes\thigh.obj
meshes/meshes\thigh_mirror.obj
meshes/meshes\calf.obj
```

## Установка

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Запуск

Откройте два терминала в корне проекта.

Терминал 1:

```bash
python -m src.controllers.robust_controller_zmq --verbose
```

Терминал 2:

```bash
python -m src.simulation.aliengo_sim_zmq --xml robot.xml --duration 16
```

Без окна MuJoCo Viewer:

```bash
python -m src.simulation.aliengo_sim_zmq --xml robot.xml --duration 16 --no-viewer
```

После завершения появится лог:

```text
logs/sim_log.csv
```

Построить графики:

```bash
python -m src.tools.plot_logs --log logs/sim_log.csv
```

Сохранить графики:

```bash
python -m src.tools.plot_logs --log logs/sim_log.csv --output logs/gait_graphs.png
```

## Где именно ZMQ

В контроллере:

```python
ctx = zmq.Context()

sock_state = ctx.socket(zmq.SUB)
sock_state.connect(args.state_endpoint)
sock_state.setsockopt(zmq.SUBSCRIBE, b"")

sock_cmd = ctx.socket(zmq.PUB)
sock_cmd.bind(args.cmd_endpoint)
```

В цикле контроллера:

```python
msg = sock_state.recv(zmq.NOBLOCK)
state = unpack_state(msg)

torques, roll_torque, pitch_torque = controller.compute(state)
cmd_msg = pack_command(state["seq"], torques, roll_torque, pitch_torque)
sock_cmd.send(cmd_msg, zmq.NOBLOCK)
```

В симуляторе аналогично: `PUB` отправляет состояние, `SUB` принимает команду.
