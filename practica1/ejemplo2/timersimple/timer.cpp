#include "timer.h"

timer::timer(): QThread()
{

}

void timer::setPeriod(int p){

   ms = 0;
}

void timer::run(){
    while(1){
        msleep(ms);
        emit timeout();
    }
}
