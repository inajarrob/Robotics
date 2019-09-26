#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"


class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
    public:
        ejemplo1();
        ~ejemplo1();
    
    public slots:
        // Metodos implementados para ejercicio 2
        void contador();
        void pararContador();

    private:
        Timer thread;
        int cont = 0;
        int trick = 5; // ????
        int stop_v = 0;
};

#endif // ejemplo1_H
