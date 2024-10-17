# Laboratório Virtual de Controle de CubeSats

Este repositório contém scripts para:

* Análise da dinâmica de Cubesat 6U

* Projeto da lei de controle para Cubesat 6U

* Projeto do algoritmo de controle para Cubesat 6U

## Organização de pastas

| Pasta                    | Descrição                                                                                                                  |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| raiz                     | Contém scripts atualmente em desenvolvimento, que se encontram em estado de evolução, de acordo com análises exploratórias |
| [aircraft](./aircraft)   | Contém modelos de "aeronave" que serão utilizados no Flight Gear                                                           |
| [engine](./engine)       | Contém modelos de motores utilizados pela "aeronave"                                                                       |
| [scripts](./scripts)     | Contém scripts de inicialização para o Flight Gear                                                                         |
| [reference](./reference) | Contém manuais e documentos usados de referência para esse trabalho                                                        |

## Softwares utilizados

A análise do sistema de controle do Cubesat 6u foi realizada por meio de três ferramentas:

| Ferramenta                                  | Aplicação                                                                                                           |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| [Flight Gear](https://www.flightgear.org/); | Software gráfico de simulação de voo                                                                                |
| [JSBSim](https://jsbsim.sourceforge.net/);  | Software/Biblioteca que implementa um modelo matemático/físico de dinâmica de voo para simulação de veículos aéreos |
| [Python](https://www.python.org/);          | Linguagem de programação multipropósito. Neste caso é usado para interagir com FlightGear e JSBsim                  |
| [Blender](https://www.blender.org/);        | Aplicação de criação 3D gratuita de código aberto, que suporta AC3D.                                                |

## Listagem dos scripts

Segue uma descrição dos scripts que fazem parte da simulação:

| Script                | Descrição                                                                                                           |
| --------------------- | ------------------------------------------------------------------------------------------------------------------- |
| cubesat_design.ipynb  | Projeto preliminar que define dados do veículo espacial, rodas de reação e missão.                                  |
| dynamic_simulation.py | Simulação da missão do cubesat 6U com modelo não linear, em que as equações do movimento são integradas pelo JSBSim |
