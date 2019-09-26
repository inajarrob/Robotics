#include "ejemplo1.h"
int stop = 0;

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
    show();
    connect(&timer, SIGNAL(timeout()), this, SLOT(contador()));
    timer.start(500);
    connect(button, SIGNAL(clicked()), this, SLOT(pararContador()));
}

ejemplo1::~ejemplo1()
{}


// Contador del programa
void ejemplo1::contador()
{
    lcdNumber->display(++cont);
    trick++;
}

// Pulsamos boton
void ejemplo1::pararContador()
{
    ++stop;
    if(stop % 2 != 0){
        timer.stop();
        qDebug() << "Timer stopped";
    }
    else{
        timer.start(500);
        qDebug() << "Timer working";
        contador();
    }

}

