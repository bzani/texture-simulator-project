#ifndef _PARTICULA_H_
#define _PARTICULA_H_

#include "psoconfig.h"

using namespace std;

class Particula {
public:
	
    /*bool operator<(Particula o) {
		return (o.qualidade < qualidade);
    }*/

	Particula(PsoConfig config) {
		this->config = config;

		for (int i = 0; i < config.tamanhoParticula; i++) {
            // dominio da solucao do problema
            float valorSolucao = 0;
            if (i%2 == 0)
                valorSolucao = config.dominioMaxX * rand() / (RAND_MAX + 1.0) + config.dominioMin;
            else
                valorSolucao = config.dominioMaxY * rand() / (RAND_MAX + 1.0) + config.dominioMin;
			vetor.push_back(valorSolucao);
			pBest.push_back(valorSolucao);

			// inicializa o vetor de velocidades
			float valorVelocidade = config.velocidadeMaxima * rand() / (RAND_MAX + 1.0);
			velocidade.push_back(valorVelocidade);
		}

		qualidade = config.funcaoObjetivo(vetor);
	}

	void imprime() {
        cout << endl << "(pBest) ";
		for (int i = 0; i < pBest.size(); ++i) {
			cout << pBest[i] << " ";
		}
		cout << " | (Qual.) " << (int)qualidade;
        cout << endl;
	}

	void atualizaVelocidade(vector<float> lBest) {
		for (int i = 0; i < vetor.size(); i++) {
            //float r = 0.5; // random(1);
            float r = 1.0 * rand() / (RAND_MAX + 1.0);
			float valorVelocidade =
						config.coeficienteInercia * velocidade[i]
						+ config.influenciaParticula * r * (pBest[i] - vetor[i])
						+ config.influenciaVizinhanca * r * (lBest[i] - vetor[i]);
			velocidade[i] = (valorVelocidade > config.velocidadeMaxima) ? config.velocidadeMaxima : valorVelocidade;
		}
	}

	float atualizaVetor() {
		for (int i = 0; i < vetor.size(); i++)
			vetor[i] += velocidade[i];
		float fitness = config.funcaoObjetivo(vetor);
		if (qualidade > fitness) {
			qualidade = fitness;
			pBest = vetor;
		}
		return qualidade;
	}

	vector<float> getPBest() { return pBest; }
	float getQualidade() { return qualidade; }

private:
	vector<float> vetor;
	vector<float> velocidade;
	vector<float> pBest;
	float qualidade;
	PsoConfig config;
};

#endif
