#ifndef REFSIGGENERATOR_H
#define REFSIGGENERATOR_H

#include <QtCore/QObject>
#include <QtCore/QDebug>
#include <cmath>

class RefSigGenerator : public QObject
{
    Q_OBJECT
public:
    explicit RefSigGenerator(QObject *parent = 0);
    ~RefSigGenerator();
    double slopstep_prim_gen();
    double sin_wave_prim_gen();
    void gen_run();
    void reset();

public slots:

public:
    double ref_signal;
    QString ref_signal_type;
    double peak_amp;
    double start_smp;
};

#endif // REFSIGGENERATOR_H
