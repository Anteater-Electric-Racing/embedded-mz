# Architecture

```mermaid
flowchart TD
    BMS(Orion BMS) -->|CAN| RASPI
    IMS(Elcon Charger) <-->|CAN| BMS
    IMS(Elcon Charger) -->|CAN| RASPI
    IMU(MPU6050 IMU) -->|I2C| RASPI
    GPS(NEO-6M GPS) -->|UART| RASPI

    B([Brake Sensors]) -->|Analog| TS-CCM
    L([Linear Potentiometers]) -->|Analog| TS-CCM
    PDL([Pedal Input]) -->|Analog| TS-CCM
    
    TS-CCM("Teensy 4.1 (CCM)") <--> |CAN| TS-PCC("Teensy 4.0 (PCC)")
    INV("Inverter (DTI HV-550)") <--> |CAN| TS-CCM

    style TS-CCM fill:#FFE0B2,stroke:#333,color:#000
    style TS-PCC fill:#FFE0B2,stroke:#333,color:#000

    TH([Thermistors]) --> |Analog| TS-PCC

    PCC(Precharge Circuit) --> |Analog / Digital| TS-PCC
    
    subgraph RASPI[Raspberry Pi]
        R(["Raspi Logger"]) -->|HTTP| I(TDengine)
        R -->|MQTT| D("Raspi Dashboard")
    end
    subgraph Graphana[Wireless Grafana]
        I -->|HTTP| F(Full History Preview)
        R -->|MQTT| P(Real Time Preview)
    end
```

# Development Environment

## fsae-daq
### Accessing fsae-daq from base station
Run Grafana:
```bash
docker run -d -p 3000:3000 --name=grafana grafana/grafana-enterprise
```
### Setting up development environment
- Open the project in VSCode
- Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for VSCode
- Open the command palette (Super+Shift+P) and run `Dev Containers: Rebuild and Reopen in Container`
- The project will now be running in a container with all the necessary dependencies (rust) and services (grafana, influxdb)
- Now run `cargo run --bin fsae-daq` to start the logging service
- You can view the dashboard at `http://localhost:3000` (default username: admin, password: admin)
- You can view the influxdb database (`http://localhost:8086`) or MQTT broker (`localhost:1883`), by adding it as a data source in Grafana
