#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{

    setupUi(this);
    show();
    connect(&thread, SIGNAL(timeout()), this, SLOT(contador()));
    thread.connect(button, SIGNAL(clicked()), this, SLOT(pararContador()));
    thread.start();

}

ejemplo1::~ejemplo1()
{}


// Contador del programa
void ejemplo1::contador()
{
    qDebug() << "counting";

    lcdNumber->display(++cont);
    //trick++;
}

// Pulsamos boton
void ejemplo1::pararContador()
{
    ++stop_v;
    if(stop_v % 2 != 0){
        thread.stop();
        //thread.terminate();
        qDebug() << "Timer stopped";
    }
    else{

        thread.start();
        qDebug() << "Timer working";
        contador();
    }

}

