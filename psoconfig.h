#ifndef _PSOCONFIG_H_
#define _PSOCONFIG_H_

#include "libs.h"

struct PsoConfig {
    int tamanhoEnxame;
    int tamanhoParticula;
    int maxIteracoes;
    int maxCongelamento;

    float minQualidade;
    float coeficienteInercia;
    float influenciaParticula;
    float influenciaVizinhanca;
    float velocidadeMaxima;

    float dominioMin;
    float dominioMaxX;
    float dominioMaxY;

    float (*funcaoObjetivo)(vector<float>&);
};

#endif // PSOCONFIG

