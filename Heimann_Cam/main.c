#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/wait.h>

#include "heimann_drv.h"

extern int sensor_main(int argc,char *argv[]);

int main(int argc, char *argv[]) {
    pid_t pid = fork();

    if (pid < 0) {
        perror("fork failed");
        exit(EXIT_FAILURE);
    }

    if (pid == 0) {
        // 子进程：执行 sensor_main
        printf("[Child] sensor_main starting...\n");
        exit(sensor_main(argc, argv)); // 确保子进程退出时正确返回
    } else {
        // 父进程：可以执行别的逻辑，或者等待子进程
        printf("[Parent] sensor_main forked, PID = %d\n", pid);

        int status;
        waitpid(pid, &status, 0);  // 等待子进程结束
        if (WIFEXITED(status)) {
            printf("[Parent] sensor_main exited with status %d\n", WEXITSTATUS(status));
        } else {
            printf("[Parent] sensor_main terminated abnormally.\n");
        }
    }

    return 0;
}

// int main(int argc,char *argv[])
// {
//     int sensor_fd, eeprom_fd;

//     // if(argc!=3){
//     //     printf("Wrong use !!!!\n");
//     //     printf("Usage: %s [sensor-i2c6] [eeprom-i2c5]\n",argv[0]);
//     //     return -1; 
//     // }

//     // sensor_fd = open(argv[1], O_RDWR); // open file and enable read and  write
//     // eeprom_fd = open(argv[2], O_RDWR);
//     // if (sensor_fd < 0){
//     //     printf("Can't open sensor_fd %s\n",argv[1]); // open i2c dev file fail
//     //     exit(1);
//     // } else {
//     //     printf("sensor_fd: %s open successfully\n",argv[1]);
//     // }
//     // if (eeprom_fd < 0){
//     //     printf("Can't open eeprom_fd %s \n",argv[2]); // open i2c dev file fail
//     //     exit(1);
//     // } else {
//     //     printf("eeprom_fd: %s open successfully\n",argv[2]);
//     // }

//     sensor_init(sensor_fd, eeprom_fd);

//     close(sensor_fd);
//     close(eeprom_fd);

//     return 0;

// }