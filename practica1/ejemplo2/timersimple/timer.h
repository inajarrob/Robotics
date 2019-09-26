#ifndef TIMER_H
#define TIMER_H

#include <QObject>
#include <QThread>
#include <cstdio>
#include <iostream>
#include <QMutex>


class Timer : public QThread
{

    Q_OBJECT
    int ms = 1000;
    QMutex mutex;
    bool isCounting = true;


public:
    Timer(){};
    ~Timer(){};

    void start(){
        mutex.lock();
        setPeriod(ms);
        isCounting = true;
        QThread::start();
        mutex.unlock();
    }

    void stop(){
        mutex.lock();
        isCounting = false;
        mutex.unlock();
    }

    bool isActive(){
        return isCounting;
    }

    void setPeriod(int _ms){
        ms = _ms;
    }

    bool getPeriod(){
        return ms;
    }

    void run()
    {
        while(isCounting){

            msleep(getPeriod());
            emit timeout();
        }
    }

public slots:
    // select the period
    //void setPeriod(int p);

protected:


signals:
    // sent when the timer finishes to count
    void timeout();
};

#endif // TIMER_H
