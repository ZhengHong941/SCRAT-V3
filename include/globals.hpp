#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

//base
#define lfb_port 8
#define lft_port 7
#define lbb_port 5
#define lbt_port 6
#define rfb_port 16
#define rft_port 17
#define rbb_port 14
#define rbt_port 13

#define twl_port 4
#define twr_port 1

#define imu_port 3

//value of pi
#define pi 3.14159265358979

//omniwheel diameter is about 28 mm
#define tw_diameter 28.5
// distance between omniwheel is about 232 to 235 mm
#define base_diameter 232

#define base_error 1
#define base_max_rpm 600
#define turn_max_rpm 400

//base pid - forward / backward
#define base_kp 1.3 // 0.8, 0.76
#define base_ki 0 // 0.003
#define base_kd 0 // 5, 6

//base pid - turning
#define turn_kp 2.8 // 2
#define turn_ki 0
#define turn_kd 0 // 15

//flipper
#define flipper_motor 11
#define flipper_roller_motor 2
#define flipperrot_port 12

#define flipper_targetUp 196 //degrees
#define flipper_targetDown 305 //degrees
#define flipper_kp 3
#define flipper_kd 0
#define flipper_ki 0

//front rollers
#define front_roller_motor 18

//cata
#define fc_motor 10 // 10
#define bc_motor 19 // 19
#define catarot_port 9 // 9

//cata pid
#define cata_kp 6
#define cata_ki 0 //0.009
#define cata_kd 50
#define Catadelay 100
// #define allowedError 2
// #define cata_target 80 //63 //130
#define cata_power 0

// #define loading_pos 0
// #define slipping_pos 355
// #define fire_pos 80

//port 20 clip on brain broken
// down = 100

#endif