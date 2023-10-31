#include<stdio.h>
#include <time.h>
#include"physics.h"

#define N 3
#define WIDTH 100
#define HEIGHT 100

int main() {

    environment env[2];
    env[0] = init_environment(N);
    memcpy(&(env[1]), &(env[0]), sizeof(env[0]));
    char *window = env[0].init_window();
    
    int i = 0;
    double delta_t;
    struct timespec now, last_render;
    timespec_get(&last_render, TIME_UTC);
    while(!env[i].collision()) {
        timespec_get(&now, TIME_UTC);
        delta_t = 1000 * (now.tv_sec - last_render.tv_sec) + (now.tv_nsec - last_render.tv_nsec) / 1000000;
        if(delta_t > 1000/12) {
            env[i].render(window);
            fprintf(stdout, "\033[H\033[J");
            pprint_environment(&(env[i]));
            fprintf(stdout, window);
            timespec_get(&last_render, TIME_UTC);
            step(&(env[i]), &(env[(i+1)%2]), 0.00000001*delta_t);
            i = (i + 1) % 2;
        }
    }
    
    return 0; 
}