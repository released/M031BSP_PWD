# M031BSP_PWD
 M031BSP_PWD

update @ 2020/11/24

1. init PB3 , to control entry low power mode ( power down or idle) , connect to external button

2. init PA7 , PC3 , to wake up MCU when in low power mode

3. TIMER3 user LIRC , to indicator PB14 (LED) , when under low power mode

4. use different timing (250 ms in normal mode , 1000ms in low power mode) for LED flashing

5. below is terminal screen capture

![image](https://github.com/released/M031BSP_PWD/blob/master/entry.jpg)

![image](https://github.com/released/M031BSP_PWD/blob/master/wake2.jpg)


