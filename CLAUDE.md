我想在這個rl_sar框架中實現我自已機械狗的rl控制

電機控制線程和imu接收線程是基於~/RoboTamerSdk4Qmini改寫里的改寫而成的

~/rl_sar/src/rl_sar/src/rl_real_opus.cpp目前已實作出一個基礎的電機控制線程和imu接收線程

接下來要加入傳統的步態控制

通過在fsm中加入一個state，get up後按3進入

初步先在sim中實現，然後再加入到opus中