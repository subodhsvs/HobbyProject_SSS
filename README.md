# HobbyProject_SSS

The project is being made on L476RG. The HAL driver files and EWARM build files have been ignored (see ```.gitignore``` file) due to their large size. They can be generated using STM32CubeX. Generate the STM32CubeMX project file ```qpro.ioc```. Make sure L4 HAL and CMSIS drivers packages are installed in STM32CubeMX.

## Assumed motor orientation and layout.

### Top View

```
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

## Correction Stratergy

 |error| M1 | M2 | M3 | M4 |
 |:---:|:--:|:--:|:--:|:--:|
 | Y+  | +  | -  | -  | +  |
 | Y-  | -  | +  | +  | -  |
 | P+  | +  | +  |-/x |-/x |
 | P-  |-/x |-/x | +  | +  |
 | R+  | +  |-/x | +  |-/x |
 | R-  |-/x | +  |-/x | +  |
     
   
   
 **x** = don't care / no correction  
 **+** = positive correction = m\*(error)  
 **-** = negative correction = m\*(error)  
 **m** = slope of error vs correction line = ERROR_AMPLIFICATION_FACTOR  
 
 
### The error amplification factor
 The error obtained from sensor readings is mapped to the final pulse width values by using a straight line function, y=mx.
 Here, 
 
 **y** = correction in pulsewidth, which is a signed number  
 **m** = ERROR_AMPLIFICATION_FACTOR  
 **x** = error
 
 
 - *example*: If roll error obtained is 0.0, then correction to be done in roll is **y = m\*0.0 = 0.0**. If roll error obtained is -1.2, then correction to be done in roll is **y = m\*(-1.2)**.


## Roadmap

- [x] Implement PID Controller
- [ ] Implement the mapping function
- [ ] Cross-check sensor output data format and frequency
- [ ] Integrate Sensor Fusion
- [ ] Implement a basic control software
