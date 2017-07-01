# HobbyProject_SSS

## Roadmap

- [x] Implement PID Controller
- [x] Implement the mapping function
- [x] Cross-check sensor output data format and frequency
- [x] Integrate Sensor Fusion
- [ ] Check working of Sensor Fusion ISR for TIMfx (TIM3)
- [ ] Debug time period for ISRs using GPIO toggling **\***
- [ ] Improve mapping function
- [ ] Integrate RC remote throttle input

For TODOs inside code, search ```TODO``` in all files (```Ctrl+Shift+F```) in all project files in EWARM.

**\*** *The GPIO pins PC2 and PC3 are used for this purpose. PC2 toggles at the entry and exit of PID controller ISR and PC3 toggles at the entry and exit of Sensor Fusion ISR.*

## Vital information

```
Nucleo Board  : L476RG
Sensor Shield : X-NUCLEO-IKS01A1
```

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
 M4             M3
 ```
 
 M1,M3 = Clockwise  
 M2,M4 = Counter-clockwise

Anti-clockwise angle is assumed to be positive and clockwise negative.

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
 | Y+  | +  | -  | +  | -  |
 | Y-  | -  | +  | -  | +  |
 | P+  | +  | +  |-/x |-/x |
 | P-  |-/x |-/x | +  | +  |
 | R+  | +  |-/x |-/x | +  |
 | R-  |-/x | +  | +  |-/x |
     
   
   
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
