# HobbyProject_SSS

The project is being made on L476RG. The HAL driver files and EWARM build files have been ignored (see ```.gitignore``` file) due to their large size. They can be generated using STM32CubeX. Generate the STM32CubeMX project file ```qpro.ioc```. Make sure L4 HAL and CMSIS drivers packages are installed in STM32CubeMX.

## Assumed motor orientation and layout.

```
Top View

 M1 _         _ M2
   (_) front (_)
     \ .___. /
      \|   |/       
       |   |
       |   |
      /|___|\
    _/       \_
   (_)       (_)
 M3             M4
 ```
 
Anti-clockwise angle is assumed to be positive and clockwise negative, in the meanwhile.
Note, that this assumption is currently a placeholder as Sensor Fusion
output data may be different.

## System Control Loop 

```
(Y,P,R)  ________________(Y',P',R')_________________ (M1,M2,M3,M4)__________      _________
------->| PID controller |------>| Mapping function |-------->| Quad System |--->| Sensors |
    ^   |________________|       |__________________|         | Response    |    |_________|
    |                                                         |_____________|         |
    |_________________________________________________________________________________|

```

## Roadmap
- [x] Implement PID Controller
- [ ] Implement the mapping function
- [ ] Cross-check sensor output data format and frequency
- [ ] Integrate Sensor Fusion
- [ ] Implement a basic control software
