### robot-prototype

- Based on [linorobot (firmware)](https://github.com/linorobot/linorobot/tree/master/teensy/firmware).
- Adapted for 2WD


### Math 

#### Revolution Per Minute (RPM)
<img src="https://render.githubusercontent.com/render/math?math=RPM = \frac{Pulses}{PPR * t} ">

- <img src="https://render.githubusercontent.com/render/math?math=Pulses"> - encoder pulses per time *t*
- <img src="https://render.githubusercontent.com/render/math?math=PPR"> - pulses per revolution
- <img src="https://render.githubusercontent.com/render/math?math=t"> - elapsed time (minutes)

#### Revolution Per Second (RPS)
<img src="https://render.githubusercontent.com/render/math?math=RPS = \frac{RPM}{60} ">

#### Angular Velocity (rad/s)
<img src="https://render.githubusercontent.com/render/math?math=V_{angular} = 2 * \pi * RPS">

#### Linear Velocity (m/s)
<img src="https://render.githubusercontent.com/render/math?math=V_{linear} = 2 * \pi * r * RPS">
<img src="https://render.githubusercontent.com/render/math?math=V_{linear} = V_{angular} * r">


#### Wheel Position (degrees)
<img src="https://render.githubusercontent.com/render/math?math=Position_{wheel} = \frac{position_{encoder}}{PPR} * 360 ">

- <img src="https://render.githubusercontent.com/render/math?math=PPR"> - pulses per revolution

#### References
- [Convert RPM to m/s](https://www.youtube.com/watch?v=qK86kDlBq_8)
